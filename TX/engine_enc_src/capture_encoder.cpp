/*
 * capture_encoder.cpp -- standalone H.265 (HEVC) encoder child for app_yolox_cam (Design B).
 *
 * Links ONLY gstreamer + mmngr (NOT DRP-AI/TVM/OpenCV), so gst_init runs in a
 * clean process. THIS process OWNS the camera capture buffers (mmngr), so the
 * VSP/codec read native buffers -- the earlier cross-process import of an
 * app-owned dma-buf returned zeros (green frame). At startup it allocates the
 * ring, hands the fds (+phys) to the app via SCM_RIGHTS; the app QBUFs them to
 * the camera (kernel dma-buf import) and the camera DMAs frames straight in.
 *
 * Buffers are NON-CACHED (MMNGR_VA_SUPPORT): camera-DMA write, app CPU read (DRP
 * copy) and child VSP read are all coherent through physical memory, no flush.
 *
 *   capture_encoder <sockfd> <host> <port> <bitrate>
 */
#include "stream_enc.h"
#include "enc_ipc.h"
#include "sei_box.h"          /* app_m5: box payload layout (SeiBoxHeader/Rec) */
extern "C" {
#include "mmngr_user_public.h"
#include "mmngr_buf_user_public.h"
}

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <time.h>
#include <atomic>

/* ===== Phase1 AQC: 3-signal conservative bitrate ladder =====================
 * REPLACES the old loss-only AIMD. The controller adjusts omxh265enc
 * target-bitrate at RUNTIME (no pipeline restart) across a small ladder of
 * discrete levels, using THREE signals. Resolution is ALWAYS 1920x1080 (user
 * rule: never downscale); the lowest levels may additionally step the frame
 * rate 60->30 (fps_div=2, the user-set floor) by dropping frames in THIS child
 * (see the main loop). Signals:
 *   (1) board WiFi tx-retries/s  -- self-sampled: `iw dev <wlx> station dump`
 *   (2) board WiFi signal dBm    -- self-sampled from the same dump
 *   (3) RX-reported loss%         -- the FbMsg the PC already sends on FB_PORT
 *
 * CONSERVATIVE control ("stability first"): step DOWN one level when ANY signal
 * is unhealthy and stays unhealthy for ~2-3s; step UP one level only after ALL
 * signals are healthy for ~15-20s. Down-fast / up-slow hysteresis + a minimum
 * dwell between any two changes prevents thrashing. Levels + thresholds + the
 * healthy top level are all env-overridable (defaults below).
 *
 * The PC (stream_viewer.py) still sends a 16-byte FbMsg ~1 Hz (loss + recv kbps);
 * we consume its loss for signal (3). recv_kbps is used as a capacity guard. */
#pragma pack(push,1)
struct FbMsg { uint32_t magic; uint16_t ver; uint16_t loss_milli; uint32_t recv_kbps; uint32_t seq; };
#pragma pack(pop)
#define FB_MAGIC      0x4644424Bu  /* 'FDBK' (LE) */
#define FB_VER        1
/* D3 P0-1b/d: accept legacy v1 AND v2. v2 does not change the 16B wire layout --
 * it repurposes recv_kbps as [flags:4|kbps:28] (kbps<=268Mbps, plenty). Bit0 of
 * the flags nibble = REQ_IDR (RX asks TX for an on-demand keyframe). */
#define FB_VER_MAX      2
#define FB_FLAG_REQ_IDR 0x1u
#define FB_PORT       50013

/* ---- AQC ladder configuration (all env-overridable) ---------------------- *
 * Levels HIGHEST (healthiest) first. Each level is {kbps, fps_div}. L0 defaults
 * to 8000 (headroom for motion, no quantization macroblocking -- the prior 1km
 * "blocks" were loss gray-outs, not quantization). Config-tunable via
 * YOLO_AQC_LEVELS (comma list; see aqc_parse_levels for the "@30" suffix).
 * fps_div=1 -> 60fps, fps_div=2 -> 30fps (drop every 2nd frame in this child).
 * 30fps is the user-set FLOOR: fps_div>2 is never generated, and resolution
 * stays 1920x1080 at every level. The two lowest defaults trade fps for
 * bits/frame (at 1500/1000kbps a 30fps stream spends 2x the bits per frame
 * vs 60fps, holding 1080p usable on a thin link). */
#define AQC_MAX_LEVELS 8
struct AqcLevel { int kbps; int fps_div; };
static AqcLevel g_levels[AQC_MAX_LEVELS] = {
    {8000,1}, {5000,1}, {3000,1}, {2000,1}, {1500,2}, {1000,2}
};
static int   g_n_levels = 6;

/* Current fps divisor: written by the AQC thread on a level change, read by the
 * main FrameMsg loop (which drops every 2nd frame when >=2). Atomic because the
 * two threads share it; no other ordering is needed (a frame dropped one tick
 * late/early around a switch is harmless). */
static std::atomic<int> g_cur_fps_div{1};

/* CONSERVATIVE thresholds (an UNHEALTHY reading trips the down-path). */
static double g_thr_retries_ps = 15.0;   /* tx retries/s above this = unhealthy   */
static double g_thr_signal_dbm = -80.0;  /* signal below this (more negative) = bad */
static double g_thr_loss_pct   = 0.3;    /* RX loss% above this = unhealthy        */

/* HEAL margins = a dead-band so a link parked AT a threshold cannot walk the ladder
 * down to the floor and get stuck: the UP-path requires CLEARLY better than the DOWN
 * thresholds. Between heal and threshold the level is stable (neither up nor down). */
static double g_heal_retries_frac = 0.7;  /* up needs retries/s <= this * thr         */
static double g_heal_loss_frac    = 0.5;  /* up needs loss%     <= this * thr         */
static double g_heal_signal_db    = 4.0;  /* up needs signal    >= thr + this (dB)    */

/* hysteresis timing (down fast, up slow) + anti-thrash dwell */
static double g_down_hold_s    = 2.5;    /* unhealthy must persist this long to step DOWN */
static double g_up_hold_s      = 18.0;   /* ALL healthy this long to step UP one level     */
static double g_min_dwell_s    = 2.0;    /* minimum time between ANY two level changes      */

/* fps_div bitrate compensation (BOARD-VALIDATION SWITCH, default OFF).
 * YOLO_AQC_FPSDIV_BR_COMP=1 -> at an fps_div=2 level the encoder target is
 * scaled by fps_div (label*2). Why this exists, when to turn it on, and why
 * it defaults OFF: see the caveat block on aqc_apply_level below. */
static int g_fps_br_comp = 0;

/* D3 P0-1b capacity guard (the 4th bad signal). We clamp AIR OCCUPANCY
 * (video kbps * FEC_N/FEC_K) -- not video goodput -- to cap*frac. FEC geometry
 * defaults to 8/12 but board_tx.sh passes the active profile's real k/n via
 * YOLO_FEC_K / YOLO_FEC_N. cap fraction defaults to 85 (%) = the 0.85 GCC/WebRTC
 * borrow, a calibration knob (YOLO_AQC_CAP_FRAC, D3 §4). recv_kbps freshness
 * window = 1.2s: STRICTER than the 3s fb_stale -- a recv_kbps older than this
 * must not clamp, and after a dropout a fresh FbMsg has to reseed recv_kbps
 * before the capacity signal re-arms (never clamp on a pre-dropout frozen value).
 *
 * GATED (YOLO_AQC_CAP=1, default OFF): cap=recv_kbps is DELIVERED GOODPUT, not
 * link capacity (bpir3_fb.py measures received bytes/span). On a saturated
 * healthy link goodput ~= offered air, so bad_cap would be ~always-true and
 * ratchet the ladder to the floor -- a regression on a clean link (the exact
 * "flags-off must stay bit-equivalent" case). D3 §10 F1/F12 flag this
 * goodput!=capacity model as UNVALIDATED (待外場坐實). So the whole capacity law
 * (bad_cap + cap_allows_up) sits behind YOLO_AQC_CAP: OFF = bit-equivalent to
 * origin (native 3-signal AQC, no capacity term); the edge profile turns it on
 * for field A/B calibration of the 0.85 knob. Flip the default only once F12 is
 * validated on-board. (Deviates from D3 §3's "常駐無 gate" intent -- adopted per
 * the P0-1 safety review CRITICAL; recorded for the orchestration layer.) */
static int    g_fec_k       = 8;
static int    g_fec_n       = 12;
static int    g_cap_frac    = 85;
static double g_cap_fresh_s = 1.2;
static int    g_cap_on      = 0;   /* D3 P0-1b: capacity-law master switch (default OFF) */

#define FB_DROUT_SEC  3            /* no RX feedback for >3s -> treat as unhealthy (loss) */

/* MUST match define.h: CAP_BUF_NUM(MIPI)=8, camera 1920x1080 YUY2.
 * Buffers are allocated at the PADDED DRP-AI input size (1920x1920, square --
 * DRPAI_INPUT_PADDING) and pre-filled with the gray YOLOX letterbox padding.
 * The camera DMAs the 1920x1080 frame into the top rows each frame; the bottom
 * stays gray. DRP-AI reads the whole padded buffer's phys directly (no CPU copy);
 * the encoder reads only the top 1920x1080 frame. */
#define CAP_BUF_NUM 8
#define CAP_W       1920
#define CAP_H       1080
#define CAP_PAD_H       1920
#define CAP_FRAME_SIZE  (CAP_W * CAP_H * 2)   /* encoder reads this (top rows) */
#define CAP_BUF_SIZE    (CAP_W * CAP_PAD_H * 2)   /* mmngr alloc: frame + gray pad */

namespace {
struct ChildBuf { int dbuf_fd; uint32_t phys; uint32_t size; MMNGR_ID id; };
ChildBuf g_buf[CAP_BUF_NUM];

struct RelCtx { int sock; uint32_t index; };

/* Fires once the encoder released the buffer -> tell the app to re-QBUF it. */
void child_on_release(void* user) {
    RelCtx* c = static_cast<RelCtx*>(user);
    if (c) { enc_send_rel(c->sock, c->index); delete c; }
}

int alloc_buffers() {
    for (int i = 0; i < CAP_BUF_NUM; i++) {
        MMNGR_ID id; unsigned int phys = 0; void* virt = nullptr; int dfd = -1;
        int r = mmngr_alloc_in_user_ext(&id, CAP_BUF_SIZE, &phys, &virt,
                                        MMNGR_VA_SUPPORT, NULL);   /* non-cached */
        if (r != R_MM_OK || !virt) {
            std::fprintf(stderr, "[capture_encoder] mmngr_alloc[%d] r=%d\n", i, r); return -1;
        }
        /* gray YOLOX letterbox padding (YUYV: Y=114,U=128,V=128); the camera
         * overwrites the top frame rows, the rest stays gray for DRP-AI. */
        uint8_t* p = (uint8_t*)virt;
        for (size_t k = 0; k + 3 < CAP_BUF_SIZE; k += 4) {
            p[k] = 114; p[k+1] = 128; p[k+2] = 114; p[k+3] = 128;
        }
        r = mmngr_export_start_in_user_ext(&id, CAP_BUF_SIZE, phys, &dfd, NULL);
        if (r != R_MM_OK || dfd < 0) {
            std::fprintf(stderr, "[capture_encoder] mmngr_export[%d] r=%d\n", i, r); return -1;
        }
        g_buf[i].dbuf_fd = dfd; g_buf[i].phys = (uint32_t)phys;
        g_buf[i].size = (uint32_t)CAP_BUF_SIZE; g_buf[i].id = id;
    }
    return 0;
}

/* ---- AQC controller context ---- */
struct FbCtx {
    char peer_ip[64];          /* accept RX feedback only from the PC (argv[2]) */
    int  init_bitrate;         /* the app's requested start bitrate (bps, argv[4]) */
    char wlx[64];              /* board WiFi NIC for `iw dev <wlx> station dump`   */
    std::atomic<bool> running{true};
};
FbCtx g_fb;

/* Parse YOLO_AQC_LEVELS into g_levels/g_n_levels (kbps, descending).
 * BACKWARD-COMPATIBLE: the legacy pure-kbps form "8000,5000,3000,2000" still
 * parses unchanged (every level fps_div=1 = 60fps). A "@30" suffix marks a
 * 30fps level (fps_div=2), e.g. "8000,5000,3000,2000,1500@30,1000@30". Only
 * @30 is accepted -- any other @fps is warned about and the suffix ignored
 * (60->30 is the only step the user allows; 30fps is the floor). As before,
 * fewer than 2 valid entries -> the whole env is ignored, defaults kept.
 * MALFORMED entries ('3000@abc', '1500@30x', '1500@30.5', plain junk): warn
 * naming the offending token, then RESYNC to the next ',' -- one bad entry
 * must never eat the rest of the ladder. (The old code left p stuck in the
 * garbage, the next strtol failed, and every later level -- including the
 * 30fps ones -- vanished silently if >=2 levels had already parsed.) */
static void aqc_parse_levels(const char* s) {
    AqcLevel tmp[AQC_MAX_LEVELS]; int n = 0;
    const char* p = s;
    while (*p && n < AQC_MAX_LEVELS) {
        const char* tok = p;                   /* entry start, for warnings */
        char* end = nullptr;
        long v = strtol(p, &end, 10);
        if (end == p) {
            /* no number at the entry start at all: warn + resync. (A leading
             * ','/' ' cannot reach here -- the separator skip below consumes
             * them -- except a leading separator in s itself, which the skip
             * handles after the no-op resync.) */
            if (*p != ',' && *p != ' ') {
                int tl = 0; while (tok[tl] && tok[tl] != ',' && tl < 24) tl++;
                std::fprintf(stderr,
                    "[capture_encoder] AQC levels: unparsable entry '%.*s' skipped\n",
                    tl, tok);
                while (*p && *p != ',') p++;   /* resync to next entry */
            }
            while (*p == ',' || *p == ' ') p++;
            continue;
        }
        p = end;
        int fdiv = 1;
        if (*p == '@') {                       /* optional fps suffix */
            char* e2 = nullptr;
            long fps = strtol(p + 1, &e2, 10);
            if (e2 != p + 1) {
                if (fps == 30) fdiv = 2;       /* the only allowed step: 60->30 */
                else std::fprintf(stderr,
                    "[capture_encoder] AQC levels: '@%ld' on %ldkbps ignored (only @30 "
                    "supported; 30fps is the floor)\n", fps, v);
                p = e2;
            } else {                           /* '@' with no number after it */
                std::fprintf(stderr,
                    "[capture_encoder] AQC levels: empty '@' on %ldkbps ignored\n", v);
                p++;
            }
        }
        /* RESIDUE GUARD: anything left in this entry that is not a separator
         * ('3000@abc' tail, '1500@30x', '1500@30.5') would stall the scan.
         * Warn, keep the part that DID parse, and resync to the next ','. */
        if (*p && *p != ',' && *p != ' ') {
            int tl = 0; while (tok[tl] && tok[tl] != ',' && tl < 24) tl++;
            std::fprintf(stderr,
                "[capture_encoder] AQC levels: trailing garbage in entry '%.*s' "
                "(kept as %ldkbps@%dfps), resyncing to next entry\n",
                tl, tok, v, fdiv >= 2 ? 30 : 60);
            while (*p && *p != ',') p++;
        }
        if (v > 0) { tmp[n].kbps = (int)v; tmp[n].fps_div = fdiv; n++; }
        while (*p == ',' || *p == ' ') p++;
    }
    if (n >= 2) { for (int i = 0; i < n; i++) g_levels[i] = tmp[i]; g_n_levels = n; }
}

static double aqc_envd(const char* name, double dflt) {
    const char* e = getenv(name);
    if (!e || !*e) return dflt;
    char* end = nullptr; double v = strtod(e, &end);
    return (end != e) ? v : dflt;
}

/* Load AQC config from the environment (all optional). */
static void aqc_load_config() {
    if (const char* e = getenv("YOLO_AQC_LEVELS")) aqc_parse_levels(e);
    g_thr_retries_ps = aqc_envd("YOLO_AQC_RETRIES_PS", g_thr_retries_ps);
    g_thr_signal_dbm = aqc_envd("YOLO_AQC_SIGNAL_DBM", g_thr_signal_dbm);
    g_thr_loss_pct   = aqc_envd("YOLO_AQC_LOSS_PCT",   g_thr_loss_pct);
    g_down_hold_s    = aqc_envd("YOLO_AQC_DOWN_HOLD_S", g_down_hold_s);
    g_up_hold_s      = aqc_envd("YOLO_AQC_UP_HOLD_S",   g_up_hold_s);
    g_min_dwell_s    = aqc_envd("YOLO_AQC_MIN_DWELL_S", g_min_dwell_s);
    g_heal_retries_frac = aqc_envd("YOLO_AQC_HEAL_RETRIES_FRAC", g_heal_retries_frac);
    g_heal_loss_frac    = aqc_envd("YOLO_AQC_HEAL_LOSS_FRAC",    g_heal_loss_frac);
    g_heal_signal_db    = aqc_envd("YOLO_AQC_HEAL_SIGNAL_DB",    g_heal_signal_db);
    g_fps_br_comp = (aqc_envd("YOLO_AQC_FPSDIV_BR_COMP", 0.0) != 0.0) ? 1 : 0;
    g_fec_k    = (int)aqc_envd("YOLO_FEC_K", g_fec_k);            /* D3 P0-1b */
    g_fec_n    = (int)aqc_envd("YOLO_FEC_N", g_fec_n);            /* D3 P0-1b */
    g_cap_frac = (int)aqc_envd("YOLO_AQC_CAP_FRAC", g_cap_frac); /* D3 P0-1b: 0.85 校準旋鈕 */
    g_cap_on   = (aqc_envd("YOLO_AQC_CAP", 0.0) != 0.0) ? 1 : 0; /* D3 P0-1b: gate, default OFF */
    if (g_fec_k <= 0) g_fec_k = 1;   /* guard div-by-zero on air-occupancy calc */
}

/* Apply a ladder level: live-set the encoder bitrate and publish the fps
 * divisor to the frame loop (which does the actual 60->30 frame dropping).
 *
 * fps_div=2 WIRE-RATE CAVEAT (UNVERIFIED ON THE BOARD -- must validate): the
 * appsrc caps stay 60/1 by design (mid-stream caps renegotiation on this OMX
 * is forbidden), and the Allegro CBR rate control most likely budgets its
 * bits-per-frame from that CONFIGURED framerate. If so, feeding real 30fps
 * halves the WIRE rate with UNCHANGED bits/frame: L4 {1500,2} really sends
 * ~750kbps, the intended "2x bits/frame at 1080p" gain of the 30fps step
 * silently fails, and L3->L4 becomes a 2000->~750kbps quality cliff. The
 * fail direction is link-SAFE (less air time, nothing breaks) but the level
 * labels lie.
 * BOARD VALIDATION: force the 30fps levels (e.g. YOLO_AQC_TEST_LOSS_PCT=1.0)
 * and read the "AQC 30fps wire probe" log (~5s cadence in fb_thread), which
 * compares the RX-reported recv_kbps against the level label:
 *   wire ~= label/2 -> theory confirmed: run with YOLO_AQC_FPSDIV_BR_COMP=1.
 *     That scales the encoder target by fps_div HERE, so the per-frame budget
 *     doubles and the wire rate matches the label again;
 *   wire ~= label   -> the encoder tracks the real input rate: keep comp OFF
 *     and just record the measured behavior (comp ON would then OVERSHOOT the
 *     label 2x on an already-degraded link -- link-UNSAFE).
 * Default OFF because undershooting a sick link is the acceptable failure and
 * overshooting is not. Do NOT "fix" this by touching the caps framerate
 * mid-stream (renegotiation risk on this OMX).
 *
 * GOP NOTE (considered, deliberately SKIPPED): interval-intraframes counts
 * FRAMES, so at fps_div=2 the fixed GOP=10 spans ~0.33s of wall time instead
 * of ~0.17s (heal latency doubles; the I-frame FRACTION of the stream is
 * unchanged). Widening the GOP at 30fps would trim I-overhead further, but
 * this Allegro OMX applies interval-intraframes only at component
 * configuration -- per gst-inspect only target-bitrate is "changeable in
 * PLAYING" -- and rebuilding the pipeline mid-flight is forbidden, so no
 * runtime GOP change is attempted. */
static void aqc_apply_level(int lvl) {
    int bps = g_levels[lvl].kbps * 1000;
    if (g_fps_br_comp) bps *= g_levels[lvl].fps_div;   /* see caveat above */
    stream_enc_set_bitrate(bps);
    g_cur_fps_div.store(g_levels[lvl].fps_div, std::memory_order_release);
}

/* Snapshot the RX-reported quality, written by the feedback socket, read by the
 * ladder tick. Plain doubles guarded by relaxed atomics (single writer/reader,
 * value coherence is all we need). */
static std::atomic<double> g_rx_loss_pct{0.0};   /* last RX loss%  */
static std::atomic<int>    g_rx_recv_kbps{0};    /* last RX recv kbps (capacity) */
static std::atomic<double> g_last_fb_walltime{0.0};

static double now_mono() {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* Board WiFi self-sample: `iw dev <wlx> station dump` -> tx retries + signal.
 * Returns true on success and fills *retries (cumulative) and *signal_dbm. */
static bool aqc_sample_wifi(const char* wlx, long* retries, double* signal_dbm) {
    if (!wlx || !*wlx) return false;
    char cmd[160];
    std::snprintf(cmd, sizeof(cmd),
                  "iw dev %s station dump 2>/dev/null", wlx);
    FILE* fp = popen(cmd, "r");
    if (!fp) return false;
    char line[256]; bool got_r = false, got_s = false;
    while (fgets(line, sizeof(line), fp)) {
        long v; double d;
        if (!got_r && std::sscanf(line, " tx retries: %ld", &v) == 1) {
            *retries = v; got_r = true;
        } else if (!got_s && std::sscanf(line, " signal: %lf", &d) == 1) {
            *signal_dbm = d; got_s = true;
        }
    }
    pclose(fp);
    return got_r || got_s;
}

/* ---- The AQC feedback + ladder thread ----
 * Runs at ~2 Hz. Each tick: drain any pending RX FbMsg (updating loss/recv), then
 * self-sample the board WiFi, evaluate the three signals, and step the ladder ONE
 * level at a time with the conservative hysteresis + dwell. */
void* fb_thread(void* arg) {
    FbCtx* c = static_cast<FbCtx*>(arg);
    aqc_load_config();

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { std::perror("[capture_encoder] fb socket"); return nullptr; }
    int one = 1; setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr; std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET; addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(FB_PORT);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        std::fprintf(stderr, "[capture_encoder] fb bind :%d failed: %s -- AQC disabled\n",
                     FB_PORT, std::strerror(errno));
        close(fd); return nullptr;
    }
    /* short recv timeout: the ladder tick cadence is set by wall clock, not by
     * counting recv timeouts. */
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 250000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    in_addr_t allowed = inet_addr(c->peer_ip);   /* INADDR_NONE -> accept any */
    uint32_t last_seq = 0; bool have_seq = false; int stale_run = 0;

    /* --- test-injection hooks (for validating the ladder on a healthy link) ---
     * When set, these FORCE the corresponding signal so the down/up path can be
     * exercised without real RF degradation. -999 = "unset" sentinel. */
    double inj_retries = aqc_envd("YOLO_AQC_TEST_RETRIES_PS", -999.0);
    double inj_signal  = aqc_envd("YOLO_AQC_TEST_SIGNAL_DBM", -999.0);
    double inj_loss    = aqc_envd("YOLO_AQC_TEST_LOSS_PCT",   -999.0);

    /* D3 P0-1d: on-demand IDR -- DEFAULT OFF (YOLO_ONDEMAND_IDR must be exactly 1).
     * TX-side local throttle (>= same min interval as the encoder property) is a
     * second layer on top of omxh265enc's min-force-key-unit-interval. */
    bool   ondemand_idr = (getenv("YOLO_ONDEMAND_IDR") &&
                           atoi(getenv("YOLO_ONDEMAND_IDR")) == 1);
    double idr_min_s    = aqc_envd("YOLO_IDR_MIN_INTERVAL_MS", 1112.0) / 1000.0;
    double last_idr_req_t = 0.0;

    /* start at the level nearest the app's requested bitrate (usually L0). */
    int lvl = 0;
    { int best = 0; int bestd = 1 << 30; int want = c->init_bitrate / 1000;
      for (int i = 0; i < g_n_levels; i++) {
          int d = g_levels[i].kbps > want ? g_levels[i].kbps - want : want - g_levels[i].kbps;
          if (d < bestd) { bestd = d; best = i; } }
      lvl = best; }

    long   prev_retries = -1; double prev_rt = now_mono();
    double unhealthy_since = 0.0;   /* mono time the current unhealthy run began (0=healthy) */
    double healthy_since   = now_mono();  /* mono time the current all-healthy run began */
    double last_change     = 0.0;   /* mono time of the last level change (0=never) */
    double last_wifi_t     = 0.0;
    double last_probe_t    = 0.0;   /* last "30fps wire probe" log (see below) */
    double last_cong_down_t = 0.0;  /* D3 P0-1b: mono time of last congestion (loss/cap) DOWN */
    int    last_good_lvl    = lvl;  /* D3 P0-1b: highest-bitrate level confirmed healthy.
                                     * TELEMETRY ONLY (logged, never gates). D3 §4 names the
                                     * probe ceiling "上次已知良好階", but the real ceiling is
                                     * cap_allows_up (true capacity, when YOLO_AQC_CAP=1); a hard
                                     * index block here would stall legitimate recovery when the
                                     * link genuinely improves past the old level. Deliberate
                                     * deviation from §4 literal -- see safety review LOW#3/spec F1. */
    long   cur_retries     = 0;   double cur_retries_ps = 0.0;
    double cur_signal      = 0.0; bool   have_signal = false;

    /* apply + announce the starting level up front (so telemetry shows it).
     * The ladder dump echoes the parsed config ("@30" = 30fps level) so a bad
     * YOLO_AQC_LEVELS is visible in the log, not silently defaulted. */
    aqc_apply_level(lvl);
    { char lad[160]; int off = 0;
      for (int i = 0; i < g_n_levels && off < (int)sizeof(lad) - 14; i++)
          off += std::snprintf(lad + off, sizeof(lad) - off, "%s%d%s",
                               i ? "," : "", g_levels[i].kbps,
                               g_levels[i].fps_div >= 2 ? "@30" : "");
      std::fprintf(stderr,
        "[capture_encoder] AQC start level=L%d target=%dkbps@%dfps ladder=%s "
        "(peer=%s wlx=%s) thr{retries/s>%.0f sig<%.0f loss>%.2f%%} "
        "hold{down=%.1fs up=%.1fs dwell=%.1fs} fpsdiv_br_comp=%d\n",
        lvl, g_levels[lvl].kbps, 60 / g_levels[lvl].fps_div, lad, c->peer_ip,
        c->wlx[0] ? c->wlx : "(none)", g_thr_retries_ps, g_thr_signal_dbm,
        g_thr_loss_pct, g_down_hold_s, g_up_hold_s, g_min_dwell_s,
        g_fps_br_comp); }
    if (inj_retries > -900 || inj_signal > -900 || inj_loss > -900)
        std::fprintf(stderr, "[capture_encoder] AQC TEST-INJECT retries/s=%.1f sig=%.1f loss=%.2f%% "
                     "(-999=off)\n", inj_retries, inj_signal, inj_loss);

    while (c->running.load()) {
        /* ---- drain RX feedback (non-blocking-ish, 250ms) ---- */
        FbMsg m; struct sockaddr_in src; socklen_t sl = sizeof(src);
        ssize_t n = recvfrom(fd, &m, sizeof(m), 0, (struct sockaddr*)&src, &sl);
        if (n == (ssize_t)sizeof(m) && m.magic == FB_MAGIC &&
            (m.ver == FB_VER || m.ver == FB_VER_MAX) &&   /* D3 P0-1b/d: v1 + v2 */
            (allowed == INADDR_NONE || src.sin_addr.s_addr == allowed)) {
            bool accept = true;
            if (have_seq && m.seq <= last_seq) {
                if (++stale_run < 3) accept = false;       /* brief reordering */
                else std::fprintf(stderr, "[capture_encoder] feedback sender resync (seq %u<=%u)\n",
                                  m.seq, last_seq);
            }
            if (accept) {
                stale_run = 0; last_seq = m.seq; have_seq = true;
                g_rx_loss_pct.store(m.loss_milli / 1000.0);
                /* D3 P0-1b/d: v2 packs recv_kbps as [flags:4|kbps:28]; v1 keeps
                 * the plain u32 kbps (backward compatible). */
                uint32_t rk = m.recv_kbps, fb_flags = 0;
                if (m.ver == FB_VER_MAX) { fb_flags = rk >> 28; rk &= 0x0FFFFFFFu; }
                g_rx_recv_kbps.store((int)rk);
                g_last_fb_walltime.store(now_mono());
                /* D3 P0-1d: on-demand IDR -- RX asked (REQ_IDR) AND switch on AND
                 * TX-side throttle elapsed (double-safety with the encoder prop). */
                if (ondemand_idr && (fb_flags & FB_FLAG_REQ_IDR)) {
                    double tnow_idr = now_mono();
                    if (last_idr_req_t == 0.0 || tnow_idr - last_idr_req_t >= idr_min_s) {
                        stream_enc_request_idr();
                        last_idr_req_t = tnow_idr;
                        std::fprintf(stderr, "[capture_encoder] on-demand IDR requested (REQ_IDR)\n");
                    }
                }
            }
        }

        /* ---- self-sample the board WiFi ~1 Hz ---- */
        double tnow = now_mono();
        if (tnow - last_wifi_t >= 1.0) {
            last_wifi_t = tnow;
            long r = -1; double s = 0.0;
            if (aqc_sample_wifi(c->wlx, &r, &s)) {
                if (r >= 0) {
                    if (prev_retries >= 0) {
                        double dt = tnow - prev_rt;
                        if (dt > 0.1) cur_retries_ps = (r - prev_retries) / dt;
                        if (cur_retries_ps < 0) cur_retries_ps = 0;   /* counter reset */
                    }
                    prev_retries = r; prev_rt = tnow; cur_retries = r;
                }
                if (s != 0.0) { cur_signal = s; have_signal = true; }
            }
        }

        /* ---- read the current three signals (test-inject overrides win) ---- */
        double retries_ps = (inj_retries > -900) ? inj_retries : cur_retries_ps;
        double signal_dbm = (inj_signal  > -900) ? inj_signal
                            : (have_signal ? cur_signal : 0.0);
        double loss_pct    = (inj_loss    > -900) ? inj_loss  : g_rx_loss_pct.load();
        /* feedback dropout for >FB_DROUT_SEC counts as unhealthy (assume loss). */
        double fb_age = tnow - g_last_fb_walltime.load();
        bool   fb_stale = (g_last_fb_walltime.load() > 0.0 && fb_age > FB_DROUT_SEC);

        /* ---- 30fps wire probe (~5s cadence at fps_div=2 levels only) ----
         * Instruments the fps_div wire-rate question (see aqc_apply_level):
         * label vs encoder target vs RX-measured recv_kbps in one line, so
         * the board validation reads the verdict straight off the log.
         * wire ~= label/2 with comp=0 -> set YOLO_AQC_FPSDIV_BR_COMP=1. */
        if (g_levels[lvl].fps_div >= 2 && tnow - last_probe_t >= 5.0) {
            last_probe_t = tnow;
            std::fprintf(stderr,
                "[capture_encoder] AQC 30fps wire probe: L%d label=%dkbps enc_target=%dkbps "
                "wire(rx)=%dkbps fb_stale=%d comp=%d\n",
                lvl, g_levels[lvl].kbps,
                g_levels[lvl].kbps * (g_fps_br_comp ? g_levels[lvl].fps_div : 1),
                g_rx_recv_kbps.load(), fb_stale ? 1 : 0, g_fps_br_comp);
        }

        /* an UNHEALTHY reading = ANY signal bad (conservative). signal only counts
         * when we actually have a reading (have_signal or injected). */
        bool bad_retries = (retries_ps > g_thr_retries_ps);
        bool bad_signal  = ((inj_signal > -900) || have_signal) &&
                           (signal_dbm < g_thr_signal_dbm);
        bool bad_loss    = (loss_pct > g_thr_loss_pct) || fb_stale;
        /* D3 P0-1b: capacity = the 4th bad signal. Compare AIR OCCUPANCY
         * (video kbps * FEC_N/K, 64-bit to avoid overflow) against cap*frac,
         * where cap = latest recv_kbps. Freshness gate: a recv_kbps older than
         * g_cap_fresh_s (1.2s, stricter than the 3s fb_stale) must NOT clamp; a
         * dropout must be reseeded by a fresh FbMsg before the signal re-arms
         * (g_last_fb_walltime is bumped only when recv_kbps is, so age==both). */
        int     cap_kbps  = g_rx_recv_kbps.load();
        bool    cap_fresh = (g_last_fb_walltime.load() > 0.0) &&
                            (tnow - g_last_fb_walltime.load() <= g_cap_fresh_s);
        int64_t air_kbps  = (int64_t)g_levels[lvl].kbps * g_fec_n / g_fec_k;
        bool    bad_cap   = g_cap_on && cap_fresh && cap_kbps > 0 &&
                            air_kbps > (int64_t)cap_kbps * g_cap_frac / 100;
        bool unhealthy   = bad_retries || bad_signal || bad_loss || bad_cap;
        /* HEALTHY (for the up-path) = retries low AND loss ~0 AND signal ok/absent
         * AND feedback fresh. */
        /* HEAL band: the up-path demands CLEARLY better than the down thresholds so a
         * link hovering right at a threshold stabilizes at its current level instead
         * of walking down to L3 and never climbing back. */
        bool healthy = (retries_ps <= g_heal_retries_frac * g_thr_retries_ps) &&
                       (loss_pct <= g_heal_loss_frac * g_thr_loss_pct) && !fb_stale &&
                       (((inj_signal > -900) || have_signal)
                            ? (signal_dbm >= g_thr_signal_dbm + g_heal_signal_db)
                            : true);

        /* ---- edge tracking for the hysteresis timers ---- */
        if (unhealthy) { if (unhealthy_since == 0.0) unhealthy_since = tnow; }
        else unhealthy_since = 0.0;
        if (healthy)   { /* keep healthy_since running */ }
        else healthy_since = tnow;   /* reset the up-timer on any non-healthy tick */

        bool dwell_ok = (last_change == 0.0) || (tnow - last_change >= g_min_dwell_s);

        /* D3 P0-1b: probe-up capacity gate -- the NEXT level up (lvl-1) must still
         * fit air occupancy <= cap*frac. If cap is stale/unknown, do NOT block
         * (the freshness gate: never let a dropout lock the ladder to the floor). */
        bool cap_allows_up = true;
        if (g_cap_on && lvl > 0 && cap_fresh && cap_kbps > 0) {
            int64_t next_air = (int64_t)g_levels[lvl - 1].kbps * g_fec_n / g_fec_k;
            cap_allows_up = next_air <= (int64_t)cap_kbps * g_cap_frac / 100;
        }
        /* D3 P0-1b: after a congestion (loss/cap) DOWN, hold-down for the up-path
         * is > up_hold (one extra dwell) -- damps the probe limit cycle where
         * peak->loss->drop keeps re-entering the congestion band (risk D1). */
        bool post_cong_ok = (last_cong_down_t == 0.0) ||
                            (tnow - last_cong_down_t) >= (g_up_hold_s + g_min_dwell_s);

        /* ---- DOWN: fast. Unhealthy held >= down_hold and dwell satisfied ---- */
        if (dwell_ok && lvl < g_n_levels - 1 && unhealthy_since > 0.0 &&
            (tnow - unhealthy_since) >= g_down_hold_s) {
            lvl++;
            aqc_apply_level(lvl);
            last_change = tnow; unhealthy_since = 0.0; healthy_since = tnow;
            if (bad_loss || bad_cap) last_cong_down_t = tnow;   /* D3 P0-1b */
            std::fprintf(stderr,
                "[capture_encoder] AQC level=L%d target=%dkbps@%dfps DOWN (retries/s=%.1f sig=%.1f "
                "loss=%.2f%% fb_stale=%d bad_cap=%d)\n",
                lvl, g_levels[lvl].kbps, 60 / g_levels[lvl].fps_div,
                retries_ps, signal_dbm, loss_pct, fb_stale ? 1 : 0, bad_cap ? 1 : 0);
        }
        /* ---- UP: slow. All healthy held >= up_hold AND capacity allows the next
         * step AND the post-congestion hold-down elapsed AND dwell satisfied ---- */
        else if (dwell_ok && lvl > 0 && healthy && cap_allows_up && post_cong_ok &&
                 (tnow - healthy_since) >= g_up_hold_s) {
            /* D3 P0-1b: this level just held healthy for up_hold => confirmed good;
             * record it for telemetry (only lower the index, i.e. only ever remember
             * a HIGHER-bitrate confirmed level). NOT used for gating -- see decl. */
            if (lvl < last_good_lvl) last_good_lvl = lvl;
            lvl--;
            aqc_apply_level(lvl);
            last_change = tnow; healthy_since = tnow; unhealthy_since = 0.0;
            std::fprintf(stderr,
                "[capture_encoder] AQC level=L%d target=%dkbps@%dfps UP (retries/s=%.1f sig=%.1f "
                "loss=%.2f%% last_good=L%d)\n",
                lvl, g_levels[lvl].kbps, 60 / g_levels[lvl].fps_div,
                retries_ps, signal_dbm, loss_pct, last_good_lvl);
        }
    }
    close(fd);
    return nullptr;
}
} /* namespace */

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr, "usage: capture_encoder <sockfd> <host> <port> <bitrate>\n");
        return 2;
    }
    int         sock    = std::atoi(argv[1]);
    const char* host    = argv[2];
    int         port    = std::atoi(argv[3]);
    int         bitrate = std::atoi(argv[4]);
    std::fprintf(stderr, "[capture_encoder] start sock=%d -> %s:%d br=%d\n", sock, host, port, bitrate);

    if (alloc_buffers() != 0) return 1;

    /* handshake: hand every capture buffer (fd + phys) to the app */
    for (int i = 0; i < CAP_BUF_NUM; i++) {
        BufMsg bm; bm.index = (uint32_t)i; bm.size = g_buf[i].size; bm.phys_addr = g_buf[i].phys;
        if (enc_send_buf(sock, bm, g_buf[i].dbuf_fd) != 0) {
            std::fprintf(stderr, "[capture_encoder] send_buf[%d] failed\n", i); return 1;
        }
    }
    std::fprintf(stderr, "[capture_encoder] sent %d capture buffers to app\n", CAP_BUF_NUM);

    if (stream_enc_init(host, port, bitrate, CAP_W, CAP_H, 60) != 0) {
        std::fprintf(stderr, "[capture_encoder] stream_enc_init failed\n"); return 1;
    }

    /* app_m5: SEI box embedding. When YOLO_SEI=1 the app forwards the latest
     * det[] over the socketpair as BoxSetMsg; we inject it as an SEI NAL into
     * every encoded AU. The UDP-JSON sidecar still runs (fallback) unless the
     * app disables it. */
    const bool sei_on = (getenv("YOLO_SEI") && atoi(getenv("YOLO_SEI")) != 0);
    if (sei_on) {
        stream_enc_enable_sei();
        std::fprintf(stderr, "[capture_encoder] YOLO_SEI=1 -> SEI box embedding active\n");
    }

    /* Phase1: launch the AQC ladder thread (RX loss + board WiFi retries/signal
     * -> live bitrate level). The board WiFi NIC comes from YOLO_WLX (the same
     * cfg.TX_IFACE the supervisor uses); if unset, WiFi self-sampling is skipped
     * and the ladder runs on RX-loss (+ any test injection) alone. */
    pthread_t fb_tid; bool fb_started = false;
    std::strncpy(g_fb.peer_ip, host, sizeof(g_fb.peer_ip) - 1);
    g_fb.init_bitrate = bitrate;
    g_fb.wlx[0] = '\0';
    if (const char* w = getenv("YOLO_WLX"))
        std::strncpy(g_fb.wlx, w, sizeof(g_fb.wlx) - 1);
    g_fb.running.store(true);
    if (pthread_create(&fb_tid, nullptr, fb_thread, &g_fb) == 0) fb_started = true;
    else std::fprintf(stderr, "[capture_encoder] WARN: AQC thread create failed -- fixed bitrate\n");

    /* per frame: app sends the ring index (FrameMsg); when SEI is on it also
     * sends the latest box set (BoxSetMsg). enc_recv_msg() demuxes both off the
     * shared SOCK_STREAM. On a box set we rebuild the SEI payload [hdr||recs]
     * and hand it to stream_enc (injected into every subsequent AU). */
    FrameMsg m; BoxSetMsg bm; int is_box; int r;
    SeiBoxRec recs[SEI_BOX_MAX];
    unsigned char payload[sizeof(SeiBoxHeader) + SEI_BOX_MAX * sizeof(SeiBoxRec)];
    /* fps_div frame counter: FREE-RUNNING parity over all received frames,
     * deliberately DECOUPLED from AQC level switches (never reset). Buffers
     * already handed to stream_enc always come back via child_on_release, and
     * the drop decision is made per frame right here at receipt, so a switch
     * mid-stream can never leak an in-flight buffer -- every frame gets exactly
     * one disposition: push (released later by gst) or immediate RelMsg. */
    uint64_t frame_parity = 0;
    while ((r = enc_recv_msg(sock, &is_box, &m, &bm, recs, SEI_BOX_MAX,
                             sizeof(SeiBoxRec))) == 1) {
        if (is_box) {
            uint16_t n = bm.n < SEI_BOX_MAX ? bm.n : SEI_BOX_MAX;
            SeiBoxHeader h; h.frame_seq = bm.frame_seq; h.n = n;
            /* app_m5 "hz": carry the IMU roll (ver/flags/roll_cdeg) straight from
             * the app's BoxSetMsg into the SEI header so every injected AU stamps
             * the current horizon angle. */
            h.ver = bm.ver; h.flags = bm.flags; h.roll_cdeg = bm.roll_cdeg;
            std::memcpy(payload, &h, sizeof(h));
            std::memcpy(payload + sizeof(h), recs, (size_t)n * sizeof(SeiBoxRec));
            stream_enc_set_sei_payload(payload, sizeof(h) + (size_t)n * sizeof(SeiBoxRec));
            continue;
        }
        if (m.index >= CAP_BUF_NUM) continue;
        /* 1080p30 ladder step (fps_div=2): drop every 2nd (even) frame HERE in
         * the child -- the frame is never pushed to appsrc, the buffer goes
         * straight back to the app via RelMsg for re-QBUF. The zero-copy
         * protocol and app_m5 need no change: the app keeps sending every frame
         * at 60fps and only sees releases arrive. PTS stays the real MONOTONIC
         * timestamp, so the caps' 60/1 framerate is left alone (proven OK:
         * caps may claim 60fps while fewer frames are actually fed). */
        frame_parity++;
        if (g_cur_fps_div.load(std::memory_order_acquire) >= 2 &&
            (frame_parity & 1) == 0) {
            enc_send_rel(sock, m.index);   /* dropped frame: recycle immediately */
            continue;
        }
        RelCtx* ctx = new RelCtx{sock, m.index};
        stream_enc_push(g_buf[m.index].dbuf_fd, CAP_FRAME_SIZE, m.pts_ns,
                        child_on_release, ctx);   /* encode the top 1080 rows only */
    }
    std::fprintf(stderr, "[capture_encoder] parent closed (r=%d), stopping\n", r);
    g_fb.running.store(false);
    if (fb_started) pthread_join(fb_tid, nullptr);
    stream_enc_stop();
    return 0;
}
