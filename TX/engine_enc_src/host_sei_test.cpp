/*
 * host_sei_test.cpp -- HOST-side (no board) validation of the REAL C code used on
 * the board for horizon-lock:
 *   (1) the roll math in imu_roll.h (imu_roll_from_plane / imu_roll_deg) against
 *       synthetic accel vectors tilted by a known angle;
 *   (2) the ver==2 SEI builder in sei_box.h (sei_build_nal_h265 + the new
 *       SeiBoxHeader) -- builds an AU and writes it to a file so the Python RX
 *       parser (stream_viewer._parse_sei_boxes) can confirm cross-language agreement.
 *
 * Build (any host g++):
 *   g++ -std=c++17 -DIMU_ROLL_MATH_ONLY -I. host_sei_test.cpp -o host_sei_test
 * Run:
 *   ./host_sei_test /tmp/sei_au.bin   # writes the AU; prints PASS/FAIL for math
 */
#ifndef IMU_ROLL_MATH_ONLY
#define IMU_ROLL_MATH_ONLY
#endif
#include "imu_roll.h"
#include "sei_box.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

static const float PI = 3.14159265358979323846f;
static int g_fail = 0;
static void check(const char* name, bool cond, const char* extra = "") {
    std::printf("%s %s %s\n", cond ? "PASS" : "FAIL", name, extra);
    if (!cond) g_fail++;
}

int main(int argc, char** argv) {
    /* ---- (1) roll math ---- */
    std::printf("=== (1) C roll math (imu_roll.h) ===\n");
    /* baseline like the real rig: in-plane gravity (gy0,gz0) */
    float gy0 = -2.77f, gz0 = -2.83f;
    check("roll==0 at baseline", std::fabs(imu_roll_deg(gy0, gz0, gy0, gz0)) < 1e-3f);
    const int angles[] = { -45, -30, -10, 5, 15, 30, 60, 90 };
    for (int t : angles) {
        float a = t * PI / 180.0f;
        float gy = gy0 * std::cos(a) - gz0 * std::sin(a);
        float gz = gy0 * std::sin(a) + gz0 * std::cos(a);
        float r = imu_roll_deg(gy, gz, gy0, gz0);
        char buf[64]; std::snprintf(buf, sizeof(buf), "got %.4f", r);
        char nm[32];  std::snprintf(nm, sizeof(nm), "roll(%+d)", t);
        check(nm, std::fabs(r - t) < 2e-2f, buf);   /* float32 tolerance */
    }
    /* flat-bench baseline (gravity ~ on one axis), 30 deg synthetic */
    {
        float by0 = 0.0f, bz0 = -9.8f, a = 30.0f * PI / 180.0f;
        float gy = by0 * std::cos(a) - bz0 * std::sin(a);
        float gz = by0 * std::sin(a) + bz0 * std::cos(a);
        float r = imu_roll_deg(gy, gz, by0, bz0);
        char buf[64]; std::snprintf(buf, sizeof(buf), "got %.4f", r);
        check("roll 30 from flat baseline", std::fabs(r - 30.0f) < 2e-2f, buf);
    }

    /* ---- (2) build a ver==2 SEI AU and dump it for the Python parser ---- */
    std::printf("\n=== (2) C SEI ver==2 build (sei_box.h) ===\n");
    SeiBoxRec recs[3];
    sei_fill_rec(&recs[0], 14, 30000.0f / 32767.0f, 100, 200, 50, 80); /* person */
    sei_fill_rec(&recs[1], 6,  20000.0f / 32767.0f, 0,   0,   0,  256); /* zeros -> escapes */
    sei_fill_rec(&recs[2], 11, 12345.0f / 32767.0f, -300, 1000, 64, 64);

    unsigned char payload[sizeof(SeiBoxHeader) + 3 * sizeof(SeiBoxRec)];
    SeiBoxHeader h;
    h.frame_seq = 1234; h.n = 3;
    h.ver = SEI_BOX_VER; h.flags = SEI_FLAG_ROLL_VALID; h.roll_cdeg = 2000; /* +20.00 */
    std::memcpy(payload, &h, sizeof(h));
    std::memcpy(payload + sizeof(h), recs, 3 * sizeof(SeiBoxRec));

    check("SeiBoxHeader is 10 bytes", sizeof(SeiBoxHeader) == 10);
    check("SeiBoxRec is 11 bytes", sizeof(SeiBoxRec) == 11);

    unsigned char nal[ (16 + sizeof(SeiBoxHeader) + 3*sizeof(SeiBoxRec)) * 2 + 64 ];
    size_t nlen = sei_build_nal_h265(payload, sizeof(payload), nal);
    check("NAL built", nlen > 0);
    check("HEVC SEI header 4e 01", nlen > 6 && nal[4] == 0x4E && nal[5] == 0x01);

    const char* outp = (argc > 1) ? argv[1] : "sei_au.bin";
    FILE* f = std::fopen(outp, "wb");
    if (f) { std::fwrite(nal, 1, nlen, f); std::fclose(f);
             std::printf("wrote %zu-byte AU -> %s\n", nlen, outp); }
    else   { std::printf("FAIL could not write %s\n", outp); g_fail++; }

    std::printf("\n%s (%d failures)\n", g_fail ? "FAILURES" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
