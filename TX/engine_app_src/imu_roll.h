/*
 * imu_roll.h -- LSM6DSO16IS accelerometer roll reader for the H.265 horizon-lock
 * (app_m5 "hz" build). Header-only, plain C/POSIX + Linux I2C dev ioctl (no glib,
 * no gst), so it can be linked into the capture_encoder child WITHOUT pulling extra deps.
 *
 * ===========================================================================
 *  WHAT / WHY
 * ===========================================================================
 *  The CSI camera module carries an ST LSM6DSO16IS 6-axis IMU on I2C bus 4,
 *  address 0x6a (WHO_AM_I reg 0x0F == 0x22). We read ONLY the accelerometer at a
 *  low rate (~100 Hz) on a dedicated thread, OFF the 60 fps encode critical path,
 *  and maintain the latest ROLL ANGLE (degrees, float) about the camera optical
 *  axis -- the rotation that tilts the horizon IN the image. The board does NOT
 *  rotate the picture (keeps the A55 light); it only ships the angle inside the
 *  per-frame H.265 SEI and the RX (PC GPU) rotates the decoded frame + boxes.
 *
 * ===========================================================================
 *  GEOMETRY (mount-agnostic, baseline-calibrated)
 * ===========================================================================
 *  Measured at the current rig pose, gravity sits at roughly (ax,ay,az) =
 *  (-8.88, -2.77, -2.83) m/s^2; on a FLAT bench (camera looking straight up/down)
 *  it was ~(+9.93, -0.13, +0.12) -> gravity collapses onto X. That identifies the
 *  IMU **X axis as the camera optical (bore-sight) axis**, so Y and Z are the two
 *  axes lying IN the image plane. Roll about the optical axis is therefore the
 *  rotation of the gravity projection in the (Y,Z) plane.
 *
 *  We do NOT assume any axis is "up-in-image". Instead we snapshot the in-plane
 *  gravity at the current (level) pose as the baseline (gy0, gz0) and report roll
 *  as the SIGNED angle from that baseline to the live (gy, gz):
 *
 *      roll = atan2( gz*gy0 - gy*gz0,  gy*gy0 + gz*gz0 )            [rad]
 *
 *  This is the 2D rotation that carries the baseline in-plane gravity onto the
 *  current one -- i.e. the horizon tilt about the optical axis -- and it is
 *  identically 0 at the calibrated pose regardless of how the module is mounted.
 *  (Equivalent to atan2(gy,gz)-atan2(gy0,gz0) but wrap-safe in one call.)
 *
 *  SIGN CAVEAT: which physical bank direction yields a positive roll, and hence
 *  which way the RX must counter-rotate, depends on the IMU axis orientation in
 *  the module and cannot be fully resolved headless. The RX rotates by -roll; if
 *  the first physical tilt test shows the horizon rotating the WRONG way, flip
 *  IMU_ROLL_SIGN below (or the RX --horizon-sign flag) -- a one-line change.
 * ===========================================================================
 */
#ifndef IMU_ROLL_H
#define IMU_ROLL_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

/* IMU_ROLL_MATH_ONLY: include ONLY the pure roll math (no Linux I2C / threads),
 * so the formula can be unit-tested on any host (host_sei_test). The board build
 * leaves it undefined and gets the full I2C reader + thread. */
#ifndef IMU_ROLL_MATH_ONLY
#include <atomic>
#include <thread>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif

/* ---- LSM6DSO16IS register map (subset) ---- */
#define LSM_I2C_BUS_DEFAULT   4
#define LSM_I2C_ADDR_DEFAULT  0x6a
#define LSM_WHO_AM_I          0x0F
#define LSM_WHO_AM_I_VAL      0x22
#define LSM_CTRL1_XL          0x10   /* accel: ODR[7:4] FS[3:2] */
#define LSM_CTRL2_G           0x11   /* gyro  (left powered down) */
#define LSM_CTRL3_C           0x12   /* BDU / IF_INC */
#define LSM_STATUS            0x1E   /* bit0 = XLDA (new accel data) */
#define LSM_OUTX_L_A          0x28   /* 0x28..0x2D = accel X/Y/Z, signed 16-bit LE */

/* CTRL1_XL = 0x40 -> ODR 104 Hz, +/-2 g (matches imu_read.py, |a|~9.8 verified) */
#define LSM_CTRL1_XL_104HZ_2G 0x40
#define LSM_CTRL3_C_BDU_IFINC 0x44   /* BDU=1, IF_INC=1 */
#define LSM_ACC_MG_PER_LSB    0.061f /* +/-2 g sensitivity */
#define LSM_G                 9.80665f

/* Flip if the RX horizon rotates the wrong way after the first physical tilt
 * test (see SIGN CAVEAT above). +1 default. */
#ifndef IMU_ROLL_SIGN
#define IMU_ROLL_SIGN (+1.0f)
#endif

/* ---- pure roll math (no I2C; unit-testable on host) ---------------------- */

/* Signed roll (radians) from the live in-plane gravity (gy,gz) relative to the
 * baseline (gy0,gz0). 0 at the calibrated pose. See header geometry note. */
static inline float imu_roll_from_plane(float gy, float gz, float gy0, float gz0) {
    float num = gz * gy0 - gy * gz0;   /* cross  (sine term)   */
    float den = gy * gy0 + gz * gz0;   /* dot    (cosine term) */
    return atan2f(num, den);
}

/* Degrees wrapper, with the sign convention applied. (Local PI -- M_PI is not
 * guaranteed by the C++ standard / some toolchains gate it behind a macro.) */
static inline float imu_roll_deg(float gy, float gz, float gy0, float gz0) {
    const float IMU_PI = 3.14159265358979323846f;
    return IMU_ROLL_SIGN * imu_roll_from_plane(gy, gz, gy0, gz0) * (180.0f / IMU_PI);
}

/* ---- live reader thread -------------------------------------------------- */
#ifndef IMU_ROLL_MATH_ONLY

class ImuRoll {
public:
    /* roll in centi-degrees (int16) for the SEI wire format, and as float deg. */
    std::atomic<int>   roll_cdeg{0};     /* latest roll * 100, clamped int16 range */
    std::atomic<bool>  valid{false};     /* true once WHO_AM_I matched + calibrated */
    std::atomic<float> amag{0.0f};       /* latest |accel| (m/s^2) for sanity/HUD */

    ImuRoll() = default;
    ~ImuRoll() { stop(); }

    /* Open the bus, verify WHO_AM_I, configure the accel, calibrate the current
     * pose as roll=0 baseline, then spin a ~odr_hz reader thread. Returns 0 on
     * success, -1 if the IMU is absent/unreadable (caller may run without it). */
    int start(int bus = LSM_I2C_BUS_DEFAULT, int addr = LSM_I2C_ADDR_DEFAULT,
              int calib_samples = 64) {
        if (running_.load()) return 0;
        bus_ = bus; addr_ = addr;
        if (open_bus() != 0) return -1;
        /* WHO_AM_I with bounded retry: the IMU shares I2C bus 4 with the CSI
         * camera (addr 0x48), whose v4l2/media-ctl init burst (cam-60fps.sh runs
         * just before us) can momentarily collide on the bus and return 0xff for
         * a single read. Retry a few times before declaring the IMU absent --
         * verified on the real board (i2cget WHO_AM_I==0x22 once the cam init
         * settles). ~10 * 25 ms = 0.25 s worst case, off the 60 fps path. */
        unsigned char who = 0;
        bool who_ok = false;
        for (int attempt = 0; attempt < 10; ++attempt) {
            if (rd_reg(LSM_WHO_AM_I, &who) == 0 && who == LSM_WHO_AM_I_VAL) {
                who_ok = true; break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        if (!who_ok) {
            std::fprintf(stderr, "[imu] WHO_AM_I=0x%02x (want 0x%02x) after retries -- IMU absent, horizon-lock disabled\n",
                         who, LSM_WHO_AM_I_VAL);
            close_bus(); return -1;
        }
        /* configure: BDU + auto-increment, accel 104 Hz +/-2 g, gyro off */
        wr_reg(LSM_CTRL3_C, LSM_CTRL3_C_BDU_IFINC);
        wr_reg(LSM_CTRL1_XL, LSM_CTRL1_XL_104HZ_2G);
        wr_reg(LSM_CTRL2_G, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        /* baseline calibration: average the in-plane gravity at the current pose */
        double sy = 0, sz = 0, sm = 0; int got = 0;
        for (int i = 0; i < calib_samples; ++i) {
            float ax, ay, az;
            if (read_accel(&ax, &ay, &az) == 0) {
                sy += ay; sz += az; sm += std::sqrt(ax*ax + ay*ay + az*az); ++got;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        if (got < calib_samples / 2) {
            std::fprintf(stderr, "[imu] calibration read failed (%d/%d) -- horizon-lock disabled\n",
                         got, calib_samples);
            close_bus(); return -1;
        }
        gy0_ = (float)(sy / got);
        gz0_ = (float)(sz / got);
        float base_mag = (float)(sm / got);
        std::fprintf(stderr, "[imu] LSM6DSO16IS ok: baseline gy0=%.3f gz0=%.3f |a|=%.3f m/s^2 -> roll=0\n",
                     gy0_, gz0_, base_mag);

        running_.store(true);
        valid.store(true);
        odr_hz_ = 50;   /* 50 Hz: plenty for horizon-lock, halves the bus transactions
                         * competing with the camera ISP on this shared I2C bus */
        thr_ = std::thread(&ImuRoll::loop, this);
        return 0;
    }

    void stop() {
        if (!running_.exchange(false)) { if (fd_ >= 0) close_bus(); return; }
        if (thr_.joinable()) thr_.join();
        /* leave the accel powered down (clean state) */
        if (fd_ >= 0) { wr_reg(LSM_CTRL1_XL, 0x00); close_bus(); }
        valid.store(false);
    }

    /* current roll as int16 centi-degrees (for SEI). 0 if not valid. */
    int16_t roll_cdeg_i16() const {
        int v = roll_cdeg.load();
        if (v >  32767) v =  32767;
        if (v < -32768) v = -32768;
        return (int16_t)v;
    }

private:
    int  fd_ = -1, bus_ = LSM_I2C_BUS_DEFAULT, addr_ = LSM_I2C_ADDR_DEFAULT;
    int  odr_hz_ = 100;
    float gy0_ = 0.0f, gz0_ = 1.0f;       /* baseline in-plane gravity */
    std::atomic<bool> running_{false};
    std::thread thr_;

    int open_bus() {
        char path[32];
        std::snprintf(path, sizeof(path), "/dev/i2c-%d", bus_);
        fd_ = ::open(path, O_RDWR);
        if (fd_ < 0) { std::fprintf(stderr, "[imu] open %s failed\n", path); return -1; }
        if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
            std::fprintf(stderr, "[imu] I2C_SLAVE 0x%02x failed\n", addr_);
            ::close(fd_); fd_ = -1; return -1;
        }
        return 0;
    }
    void close_bus() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

    /* All register access uses I2C_RDWR (a single, kernel-locked combined transfer)
     * instead of separate write()+read(). The IMU shares this I2C bus with the CSI
     * camera ISP (addr 0x48); a non-atomic write(reg)+read(val) releases the adapter
     * lock between the two transactions, letting a camera transaction interleave --
     * which corrupts the read and can wedge the bus (stuck 0x00 until reboot, the
     * observed failure during streaming). With I2C_RDWR the write-reg + repeated-start
     * + read is ONE uninterruptible transfer; the kernel adapter lock simply queues
     * it behind any in-flight camera transaction, so we read cleanly in the gaps. */
    int wr_reg(unsigned char reg, unsigned char val) {
        unsigned char b[2] = { reg, val };
        struct i2c_msg m; m.addr = (unsigned short)addr_; m.flags = 0; m.len = 2; m.buf = b;
        struct i2c_rdwr_ioctl_data x; x.msgs = &m; x.nmsgs = 1;
        return (ioctl(fd_, I2C_RDWR, &x) >= 0) ? 0 : -1;
    }
    /* burst read n bytes starting at reg (IF_INC auto-increments the reg ptr) as one
     * atomic write-reg + repeated-start read. */
    int rd_burst(unsigned char reg, unsigned char* buf, int n) {
        struct i2c_msg m[2];
        m[0].addr = (unsigned short)addr_; m[0].flags = 0;        m[0].len = 1;               m[0].buf = &reg;
        m[1].addr = (unsigned short)addr_; m[1].flags = I2C_M_RD; m[1].len = (unsigned short)n; m[1].buf = buf;
        struct i2c_rdwr_ioctl_data x; x.msgs = m; x.nmsgs = 2;
        return (ioctl(fd_, I2C_RDWR, &x) >= 0) ? 0 : -1;
    }
    int rd_reg(unsigned char reg, unsigned char* val) { return rd_burst(reg, val, 1); }

    static int16_t s16(unsigned char lo, unsigned char hi) {
        return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    }

    /* one accel sample in m/s^2 (returns 0 on success). */
    int read_accel(float* ax, float* ay, float* az) {
        unsigned char d[6];
        if (rd_burst(LSM_OUTX_L_A, d, 6) != 0) return -1;
        float k = LSM_ACC_MG_PER_LSB / 1000.0f * LSM_G;
        *ax = s16(d[0], d[1]) * k;
        *ay = s16(d[2], d[3]) * k;
        *az = s16(d[4], d[5]) * k;
        return 0;
    }

    void loop() {
        const auto period = std::chrono::microseconds(1000000 / (odr_hz_ > 0 ? odr_hz_ : 100));
        /* light low-pass (EMA) so encode-thread snapshots are steady; cheap. */
        float roll_f = 0.0f; bool seeded = false;
        const float alpha = 0.30f;   /* ~ a few-sample smoothing at 100 Hz */
        while (running_.load()) {
            float ax, ay, az;
            if (read_accel(&ax, &ay, &az) == 0) {
                float r = imu_roll_deg(ay, az, gy0_, gz0_);
                if (!seeded) { roll_f = r; seeded = true; }
                else         { roll_f = roll_f + alpha * (r - roll_f); }
                int cd = (int)lroundf(roll_f * 100.0f);
                roll_cdeg.store(cd);
                amag.store(std::sqrt(ax*ax + ay*ay + az*az));
            }
            std::this_thread::sleep_for(period);
        }
    }
};

#endif /* IMU_ROLL_MATH_ONLY */

#endif /* IMU_ROLL_H */
