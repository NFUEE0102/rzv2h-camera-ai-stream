/*
 * aqc_selftest.cpp -- standalone deterministic test of the Phase1 AQC ladder
 * decision logic. It reproduces the EXACT ladder step logic + thresholds +
 * hysteresis timers from capture_encoder.cpp's fb_thread() and drives it through a
 * scripted signal timeline (virtual clock) so the full DOWN 8000 -> ... ->
 * 1000@30fps then UP progression + the anti-thrash dwell can be verified with
 * zero hardware / camera dependency. Part 2 checks the fps_div=2 frame-drop
 * parity (the 60->30 step) exactly as the capture_encoder main loop implements it.
 * Part 3 checks the YOLO_AQC_FPSDIV_BR_COMP target-bitrate scaling (the
 * board-validation switch for the caps-60/1 CBR wire-rate question). Part 4
 * checks the YOLO_AQC_LEVELS parser, in particular the malformed-entry
 * resync (one bad entry must never silently discard the rest of the ladder).
 *
 * The stepping + parsing code below is copied 1:1 from capture_encoder.cpp (same
 * variable names, same comparisons, same order) so this test is a faithful
 * check of the shipped controller. If the two ever diverge, update BOTH.
 *
 * Build (x86 host or board): g++ -std=c++17 -O2 aqc_selftest.cpp -o aqc_selftest
 */
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <cmath>

/* ---- ladder config (mirrors capture_encoder.cpp defaults) ----
 * Each level is {kbps, fps_div}: resolution is fixed 1920x1080 at every level;
 * fps_div=1 -> 60fps, fps_div=2 -> 30fps (the user floor, >2 never allowed). */
#define AQC_MAX_LEVELS 8
struct AqcLevel { int kbps; int fps_div; };
static AqcLevel g_levels[AQC_MAX_LEVELS] =
    {{8000,1},{5000,1},{3000,1},{2000,1},{1500,2},{1000,2}};
static int    g_n_levels = 6;
static double g_thr_retries_ps = 15.0;
static double g_thr_signal_dbm = -80.0;
static double g_thr_loss_pct   = 0.3;
/* heal band (mirrors capture_encoder.cpp): the UP path demands CLEARLY better than the
 * DOWN thresholds so a link parked at a threshold stabilizes instead of
 * walking down and never climbing back. */
static double g_heal_retries_frac = 0.7;
static double g_heal_loss_frac    = 0.5;
static double g_heal_signal_db    = 4.0;
static double g_down_hold_s    = 2.5;
static double g_up_hold_s      = 18.0;
static double g_min_dwell_s    = 2.0;

/* applied-level recorder (stub for capture_encoder.cpp's aqc_apply_level, which does
 * stream_enc_set_bitrate + publishes fps_div to the frame loop). The
 * g_fps_br_comp scaling mirrors capture_encoder.cpp's YOLO_AQC_FPSDIV_BR_COMP
 * board-validation switch (default OFF): when ON, an fps_div=2 level's
 * encoder target is label*fps_div, compensating a CBR rate control that
 * budgets bits/frame from the fixed 60/1 caps framerate. */
static int    g_fps_br_comp     = 0;
static int    g_applied_bps     = 0;
static int    g_applied_fps_div = 1;
static int    g_apply_calls     = 0;
static bool   g_seen_fps30      = false;
static void apply_level(int lvl) {
    int bps = g_levels[lvl].kbps * 1000;
    if (g_fps_br_comp) bps *= g_levels[lvl].fps_div;   /* see comment above */
    g_applied_bps     = bps;
    g_applied_fps_div = g_levels[lvl].fps_div;
    if (g_applied_fps_div >= 2) g_seen_fps30 = true;
    g_apply_calls++;
}

/* ==== YOLO_AQC_LEVELS parser, copied 1:1 from capture_encoder.cpp aqc_parse_levels
 * (same names, same order) for part 4. If the two ever diverge, update BOTH. */
static void aqc_parse_levels(const char* s) {
    AqcLevel tmp[AQC_MAX_LEVELS]; int n = 0;
    const char* p = s;
    while (*p && n < AQC_MAX_LEVELS) {
        const char* tok = p;                   /* entry start, for warnings */
        char* end = nullptr;
        long v = strtol(p, &end, 10);
        if (end == p) {
            /* no number at the entry start at all: warn + resync. */
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

/* part-4 helpers: reset the ladder to defaults, parse, compare. */
static void reset_levels() {
    static const AqcLevel d[AQC_MAX_LEVELS] =
        {{8000,1},{5000,1},{3000,1},{2000,1},{1500,2},{1000,2}};
    std::memcpy(g_levels, d, sizeof(d));
    g_n_levels = 6;
}
static bool expect_ladder(const char* env, const int* kbps, const int* fdiv, int n) {
    reset_levels();
    aqc_parse_levels(env);
    bool ok = (g_n_levels == n);
    for (int i = 0; ok && i < n; i++)
        ok = (g_levels[i].kbps == kbps[i] && g_levels[i].fps_div == fdiv[i]);
    std::printf("parse '%s' -> %d levels [", env, g_n_levels);
    for (int i = 0; i < g_n_levels; i++)
        std::printf("%s%d%s", i ? "," : "", g_levels[i].kbps,
                    g_levels[i].fps_div >= 2 ? "@30" : "");
    std::printf("] %s\n", ok ? "ok" : "BAD");
    return ok;
}

/* a scripted signal reading at a virtual time */
struct Signal { double retries_ps; double signal_dbm; double loss_pct; bool have_signal; bool fb_stale; };

int main() {
    /* controller state (mirrors fb_thread) */
    int    lvl = 0;
    double unhealthy_since = 0.0;
    double healthy_since   = 0.0;   /* set on first tick */
    double last_change     = 0.0;
    int    min_lvl_reached = 0;     /* deepest (worst) level seen, for the pass check */

    apply_level(lvl);
    std::printf("t=%.1f  START level=L%d target=%dkbps@%dfps\n",
                0.0, lvl, g_levels[lvl].kbps, 60 / g_levels[lvl].fps_div);

    /* --- scripted timeline (virtual clock, 0.5s ticks) ---
     * Phase A (0-20s):  UNHEALTHY (loss 1.0% > 0.3) -> expect DOWN L0->..->L5
     *                   (downs land at t=3,6,9,12,15; L4/L5 are the 30fps
     *                   levels, so fps_div must switch to 2 on the way down).
     * Phase B (20s+):   ALL HEALTHY (loss 0, sig -30) -> expect UP L5->..->L0
     *                   (one up per up_hold=18s: t=48,66,84,102,120).
     * Then a brief 1-tick glitch inside the healthy run to confirm NO thrash. */
    const double DT = 0.5;
    const double T_END = 140.0;   /* the UP climb from L5 at up_hold=18s ends ~t=120 */
    healthy_since = 0.0;
    bool first = true;

    for (double tnow = 0.0; tnow <= T_END; tnow += DT) {
        Signal s;
        if (tnow < 20.0) {
            /* Phase A: sustained loss */
            s = {0.0, -30.0, 1.0, true, false};
        } else if (tnow >= 30.0 && tnow < 30.5) {
            /* a single-tick glitch DURING the healthy up-climb: retries spike.
             * With down_hold=2.5s this must NOT trigger a DOWN (anti-thrash) --
             * one bad tick is far short of the hold, and it resets the up-timer. */
            s = {40.0, -30.0, 0.0, true, false};
        } else {
            /* Phase B: all healthy */
            s = {0.0, -30.0, 0.0, true, false};
        }

        if (first) { healthy_since = tnow; first = false; }

        /* ==== EXACT logic copied from capture_encoder.cpp fb_thread ==== */
        double retries_ps = s.retries_ps;
        double signal_dbm = s.have_signal ? s.signal_dbm : 0.0;
        double loss_pct   = s.loss_pct;
        bool   fb_stale   = s.fb_stale;

        bool bad_retries = (retries_ps > g_thr_retries_ps);
        bool bad_signal  = (s.have_signal) && (signal_dbm < g_thr_signal_dbm);
        bool bad_loss    = (loss_pct > g_thr_loss_pct) || fb_stale;
        bool unhealthy   = bad_retries || bad_signal || bad_loss;
        bool healthy = (retries_ps <= g_heal_retries_frac * g_thr_retries_ps) &&
                       (loss_pct <= g_heal_loss_frac * g_thr_loss_pct) && !fb_stale &&
                       (s.have_signal ? (signal_dbm >= g_thr_signal_dbm + g_heal_signal_db)
                                      : true);

        if (unhealthy) { if (unhealthy_since == 0.0) unhealthy_since = tnow; }
        else unhealthy_since = 0.0;
        if (healthy)   { /* keep healthy_since running */ }
        else healthy_since = tnow;

        bool dwell_ok = (last_change == 0.0) || (tnow - last_change >= g_min_dwell_s);

        if (dwell_ok && lvl < g_n_levels - 1 && unhealthy_since > 0.0 &&
            (tnow - unhealthy_since) >= g_down_hold_s) {
            lvl++;
            apply_level(lvl);
            last_change = tnow; unhealthy_since = 0.0; healthy_since = tnow;
            if (lvl > min_lvl_reached) min_lvl_reached = lvl;
            std::printf("t=%.1f  AQC level=L%d target=%dkbps@%dfps DOWN (retries/s=%.1f sig=%.1f loss=%.2f%%)\n",
                        tnow, lvl, g_levels[lvl].kbps, 60 / g_levels[lvl].fps_div,
                        retries_ps, signal_dbm, loss_pct);
        }
        else if (dwell_ok && lvl > 0 && healthy &&
                 (tnow - healthy_since) >= g_up_hold_s) {
            lvl--;
            apply_level(lvl);
            last_change = tnow; healthy_since = tnow; unhealthy_since = 0.0;
            std::printf("t=%.1f  AQC level=L%d target=%dkbps@%dfps UP (retries/s=%.1f sig=%.1f loss=%.2f%%)\n",
                        tnow, lvl, g_levels[lvl].kbps, 60 / g_levels[lvl].fps_div,
                        retries_ps, signal_dbm, loss_pct);
        }
    }

    /* ==== part 2: fps_div frame-drop parity (copied 1:1 from the capture_encoder main
     * loop). frame_parity FREE-RUNS across the level switch (never reset), and
     * every frame gets EXACTLY ONE disposition -- push (released later by gst)
     * or immediate RelMsg -- so a mid-stream switch can never leak a buffer. */
    int pushed = 0, released_now = 0;
    uint64_t frame_parity = 0;
    int fps_div = 1;
    for (int f = 0; f < 20; f++) {
        if (f == 10) fps_div = 2;      /* AQC drops to a 30fps level mid-stream */
        frame_parity++;
        if (fps_div >= 2 && (frame_parity & 1) == 0) {
            released_now++;            /* stands in for enc_send_rel(sock, index) */
            continue;
        }
        pushed++;                      /* stands in for stream_enc_push(...) */
    }
    /* 10 frames at 60fps all pushed; 10 frames at 30fps -> 5 pushed + 5 dropped */
    bool pass_drop = (pushed == 15 && released_now == 5 &&
                      pushed + released_now == 20);
    std::printf("\nfps_div drop parity: pushed=%d released_now=%d -> %s\n",
                pushed, released_now, pass_drop ? "ok" : "BAD");

    /* snapshot the end-of-timeline state BEFORE parts 3/4 poke the globals */
    int final_bps = g_applied_bps, final_fps_div = g_applied_fps_div;
    int final_n_levels = g_n_levels;

    /* ==== part 3: fps_div bitrate compensation (mirrors aqc_apply_level).
     * comp OFF (default): a 30fps level's encoder target equals its label --
     * the SAFE failure direction if the caps-60/1 CBR halves the wire rate
     * (see the caveat block in capture_encoder.cpp). comp ON (YOLO_AQC_FPSDIV_BR_COMP
     * =1, set only after the board "30fps wire probe" reads ~label/2):
     * target = label * fps_div, and a 60fps level is UNCHANGED (fps_div=1). */
    g_fps_br_comp = 0; apply_level(4);                 /* L4 = {1500,2} */
    bool pass_comp = (g_applied_bps == 1500 * 1000 && g_applied_fps_div == 2);
    g_fps_br_comp = 1; apply_level(4);                 /* comp ON: 1500*2 */
    pass_comp = pass_comp && (g_applied_bps == 1500 * 1000 * 2);
    apply_level(0);                                    /* fps_div=1: comp is a no-op */
    pass_comp = pass_comp && (g_applied_bps == 8000 * 1000);
    g_fps_br_comp = 0;
    std::printf("fpsdiv bitrate comp: off=1500k on=3000k 60fps-noop=8000k -> %s\n",
                pass_comp ? "ok" : "BAD");

    /* ==== part 4: YOLO_AQC_LEVELS parser -- malformed-entry resync. The old
     * bug: garbage residue stalled the scan, so one bad entry silently
     * discarded EVERY later level (e.g. '8000,5000,3000@abc,2000,1500@30,
     * 1000@30' became a 3-level 60fps-only ladder and the 30fps levels
     * vanished). Now a bad entry warns + resyncs to the next ','. */
    bool pass_parse = true;
    { /* legacy pure-kbps form: unchanged */
      const int k[] = {8000,5000,3000,2000}, f[] = {1,1,1,1};
      pass_parse = expect_ladder("8000,5000,3000,2000", k, f, 4) && pass_parse; }
    { /* '@abc' garbage mid-ladder must NOT eat the 30fps levels */
      const int k[] = {8000,5000,3000,2000,1500,1000}, f[] = {1,1,1,1,2,2};
      pass_parse = expect_ladder("8000,5000,3000@abc,2000,1500@30,1000@30",
                                 k, f, 6) && pass_parse; }
    { /* trailing garbage after a valid @30 ('30x'): warn, keep, resync */
      const int k[] = {8000,1500,1000}, f[] = {1,2,2};
      pass_parse = expect_ladder("8000,1500@30x,1000@30", k, f, 3) && pass_parse; }
    { /* '@30.5': integer part parses as @30, '.5' is warned + resynced */
      const int k[] = {8000,1500,1000}, f[] = {1,2,2};
      pass_parse = expect_ladder("8000,1500@30.5,1000@30", k, f, 3) && pass_parse; }
    { /* pure-junk entry skipped, the rest survives */
      const int k[] = {8000,2000}, f[] = {1,1};
      pass_parse = expect_ladder("8000,junk,2000", k, f, 2) && pass_parse; }
    { /* <2 valid entries -> whole env ignored, defaults kept */
      const int k[] = {8000,5000,3000,2000,1500,1000}, f[] = {1,1,1,1,2,2};
      pass_parse = expect_ladder("8000", k, f, 6) && pass_parse; }
    std::printf("levels parser resync: %s\n", pass_parse ? "ok" : "BAD");

    std::printf("FINAL level=L%d applied=%dbps fps_div=%d apply_calls=%d min_level=L%d\n",
                lvl, final_bps, final_fps_div, g_apply_calls, min_lvl_reached);
    /* expected: DOWN all the way to L5 (1000kbps@30fps) during Phase A, UP back
     * to L0 (8000kbps@60fps) during Phase B, the single glitch at t=30 causes NO
     * extra DOWN, the frame-drop parity holds across a level switch, the comp
     * math matches, and the parser survives malformed entries. */
    bool pass = (lvl == 0 && final_bps == 8000 * 1000 && final_fps_div == 1 &&
                 min_lvl_reached == final_n_levels - 1 && g_seen_fps30 &&
                 pass_drop && pass_comp && pass_parse);
    std::printf("SELFTEST %s\n", pass
                ? "PASS (DOWN to L5@30fps + UP to L0@60fps, no thrash, drop parity ok, "
                  "comp math ok, parser resync ok)"
                : "FAIL");
    return pass ? 0 : 1;
}
