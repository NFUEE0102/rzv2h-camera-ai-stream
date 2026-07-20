/*
 * stream_enc.cpp -- in-process GStreamer 1.22 zero-copy H.265 (HEVC) encode.
 * Built ONLY into the standalone capture_encoder child (clean gst process).
 *
 * app_m5 H.265 SWITCH (long-distance WiFi optimisation): the encoder is now
 * omxh265enc (Renesas VCD hardware) instead of omxh264enc. HEVC achieves equal
 * subjective quality at ~40-50% of H.264's bitrate, so the same link budget
 * carries a much cleaner 1080p60 picture, or the same picture over a thinner
 * (long-range) link. The SEI box NAL is now an HEVC PREFIX_SEI_NUT (type 39).
 *
 * Hot path per frame: dup(fd) + dmabuf wrap + appsrc push. No memcpy, no
 * software colorspace (vspmfilter + omxh265enc are hardware).
 *
 * Adversarial-review fixes (workflow verify phase): appsrc leaky-type=downstream
 * (block=false+max-bytes does NOT drop in gst 1.22); release notify on the
 * GstMemory (precise dma-buf finalize); GstVideoMeta with real stride; teardown
 * drains in-flight before return; kernel-reported dma-buf size.
 */
#include "stream_enc.h"
#include "sei_box.h"          /* app_m5: SEI box NAL builder + UUID + layout */

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/video/video.h>

#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

namespace {

struct EncState {
    GstElement*    pipeline     = nullptr;
    GstElement*    appsrc       = nullptr;
    GstElement*    venc         = nullptr;   /* app_m4: omxh265enc (live bitrate) */
    GstAllocator*  dmabuf_alloc = nullptr;
    GstBus*        bus          = nullptr;
    std::thread    bus_thr;
    std::atomic<bool> running{false};
    std::atomic<long> inflight{0};
    int            width  = 0;
    int            height = 0;
    int            fps    = 60;
    int            stride = 0;
    /* app_m5 SEI box embedding */
    GstElement*    h265parse = nullptr;     /* SEI-inject probe lives on its SRC pad */
    std::atomic<bool> sei_enabled{false};
    std::mutex     sei_mtx;                  /* guards sei_payload + sei_epoch */
    unsigned char  sei_payload[16 + sizeof(SeiBoxHeader) + SEI_BOX_MAX * sizeof(SeiBoxRec)];
    size_t         sei_payload_len = 0;      /* 0 -> nothing to inject yet */
    /* L2 (CPU optimisation, 2026-06-27): the box/roll payload changes only at the
     * ~20 Hz detection rate, but sei_inject_probe() fires per AU at 60 fps. Bump
     * sei_epoch whenever the payload changes; the probe re-injects + full-copies
     * the AU only when it sees a NEW epoch (or on a keyframe AU, so a late joiner
     * gets boxes on the next IDR). On unchanged P-frame AUs it passes the buffer
     * through untouched -- the RX holds the last boxes, identical to today's
     * behaviour (today's per-frame re-inject carries the SAME bytes anyway). */
    std::atomic<uint32_t> sei_epoch{0};      /* payload version, bumped on change   */
    uint32_t       sei_probe_epoch = 0;      /* last epoch the probe injected (probe-only) */
};
EncState g_enc;

struct ReleaseCtx { void (*on_release)(void*); void* user; };

GQuark release_quark() {
    static GQuark q = g_quark_from_static_string("stream-release-ctx");
    return q;
}

/* Fires when the dma-buf GstMemory's last ref drops -> safe to recycle slot. */
void release_notify(gpointer d) {
    ReleaseCtx* c = static_cast<ReleaseCtx*>(d);
    if (c) {
        if (c->on_release) c->on_release(c->user);
        g_enc.inflight.fetch_sub(1, std::memory_order_acq_rel);
        delete c;
    }
}

void bus_drain_loop() {
    while (g_enc.running.load()) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            g_enc.bus, 200 * GST_MSECOND,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS));
        if (!msg) continue;
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError* err = nullptr; gchar* dbg = nullptr;
                gst_message_parse_error(msg, &err, &dbg);
                std::fprintf(stderr, "[stream_enc] ERROR from %s: %s (%s)\n",
                             GST_OBJECT_NAME(msg->src), err ? err->message : "?", dbg ? dbg : "");
                if (err) g_error_free(err); g_free(dbg);
                break;
            }
            case GST_MESSAGE_WARNING: {
                GError* err = nullptr; gchar* dbg = nullptr;
                gst_message_parse_warning(msg, &err, &dbg);
                std::fprintf(stderr, "[stream_enc] WARN from %s: %s\n",
                             GST_OBJECT_NAME(msg->src), err ? err->message : "?");
                if (err) g_error_free(err); g_free(dbg);
                break;
            }
            case GST_MESSAGE_EOS: std::fprintf(stderr, "[stream_enc] EOS\n"); break;
            default: break;
        }
        gst_message_unref(msg);
    }
}

/* app_m5: SEI-inject pad probe on the h265parse SRC pad. h265parse emits
 * byte-stream (Annex-B) access units (config-interval=1 => VPS/SPS/PPS prefixed
 * on IDR AUs). We build an HEVC PREFIX_SEI_NUT (nal_unit_type 39) from the
 * current box payload and PREPEND it to the AU, then replace the probe's buffer
 * with [SEI || original]. Putting the prefix-SEI first (before VPS/SPS/PPS/
 * slices) is valid AU ordering and survives rtph265pay (which re-packetizes
 * per-NAL by scanning start codes). Decoders ignore the unknown UUID SEI, so the
 * video still decodes for non-aware receivers. */
GstPadProbeReturn sei_inject_probe(GstPad* /*pad*/, GstPadProbeInfo* info, gpointer /*ud*/) {
    if (!g_enc.sei_enabled.load()) return GST_PAD_PROBE_OK;
    GstBuffer* inbuf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!inbuf) return GST_PAD_PROBE_OK;

    /* L2: decide whether THIS AU needs a (re)injected SEI before doing any work.
     * A keyframe AU (no DELTA_UNIT flag) always gets the SEI so a late RX joiner
     * picks up the current boxes at the next IDR (~0.5s). A P-frame AU only gets
     * a fresh SEI when the box/roll payload changed since the last injection
     * (epoch bump). On the ~40/60 unchanged P-frames we skip the full-AU alloc +
     * double memcpy entirely and pass the original buffer through -- the RX keeps
     * drawing the last boxes (it holds state across SEI-less AUs). */
    uint32_t epoch  = g_enc.sei_epoch.load(std::memory_order_acquire);
    bool is_keyframe = !GST_BUFFER_FLAG_IS_SET(inbuf, GST_BUFFER_FLAG_DELTA_UNIT);
    bool need_inject = is_keyframe || (epoch != g_enc.sei_probe_epoch);
    if (!need_inject) return GST_PAD_PROBE_OK;  /* unchanged P-frame -> pass through */

    /* snapshot the current payload under the lock */
    unsigned char payload[sizeof(g_enc.sei_payload)];
    size_t plen;
    {
        std::lock_guard<std::mutex> lk(g_enc.sei_mtx);
        plen = g_enc.sei_payload_len;
        if (plen) std::memcpy(payload, g_enc.sei_payload, plen);
    }
    if (plen == 0) return GST_PAD_PROBE_OK;     /* no boxes yet -> pass through */
    /* commit the epoch only once we know we have a payload to inject */
    g_enc.sei_probe_epoch = epoch;

    /* worst case escaped size: payloadType+size+UUID+payload each may gain a
     * 0x03, plus start code(4)+nal(1)+trailing(1). 2x + 32 is a safe bound. */
    unsigned char sei[ (16 + sizeof(SeiBoxHeader) + SEI_BOX_MAX*sizeof(SeiBoxRec)) * 2 + 64 ];
    size_t sei_len = sei_build_nal_h265(payload, plen, sei);   /* HEVC prefix-SEI (type 39) */

    /* Build ONE contiguous buffer = [SEI || original AU]. We deliberately avoid
     * gst_buffer_append() (which yields a multi-GstMemory buffer); a single
     * contiguous allocation is what rtph265pay's FU fragmentation and the
     * downstream udpsink consume reliably. Map the original AU read-only and
     * memcpy both parts into the fresh buffer. */
    GstMapInfo mi;
    if (!gst_buffer_map(inbuf, &mi, GST_MAP_READ))
        return GST_PAD_PROBE_OK;                /* can't read -> pass original through */
    gsize au_len = mi.size;
    GstBuffer* out = gst_buffer_new_allocate(nullptr, sei_len + au_len, nullptr);
    gst_buffer_fill(out, 0, sei, sei_len);
    gst_buffer_fill(out, sei_len, mi.data, au_len);
    gst_buffer_unmap(inbuf, &mi);

    /* carry the original timestamps + AU flags onto the replacement */
    GST_BUFFER_PTS(out)      = GST_BUFFER_PTS(inbuf);
    GST_BUFFER_DTS(out)      = GST_BUFFER_DTS(inbuf);
    GST_BUFFER_DURATION(out) = GST_BUFFER_DURATION(inbuf);
    GST_BUFFER_FLAGS(out)    = GST_BUFFER_FLAGS(inbuf);

    /* Replace the probe's buffer. info owned `inbuf`; we now hand it `out`
     * (fresh ref) and drop inbuf's ref. */
    GST_PAD_PROBE_INFO_DATA(info) = out;
    gst_buffer_unref(inbuf);
    return GST_PAD_PROBE_OK;
}

} /* anonymous namespace */

int stream_enc_init(const char* host, int port, int bitrate, int w, int h, int in_fps) {
    if (g_enc.running.load()) return 0;
    if (!gst_is_initialized()) {
        GError* gerr = nullptr;
        if (!gst_init_check(nullptr, nullptr, &gerr)) {
            std::fprintf(stderr, "[stream_enc] gst_init failed: %s\n", gerr ? gerr->message : "?");
            if (gerr) g_error_free(gerr);
            return -1;
        }
    }
    g_enc.width  = w;
    g_enc.height = h;
    g_enc.fps    = (in_fps > 0) ? in_fps : 60;
    g_enc.stride = w * 2;

    /* Phase1 (error resilience): shorter GOP so a lost-reference gray-out self-heals
     * faster. interval-intraframes was 30 (~0.5s @60fps); default is now 10 (~0.17s)
     * -> a dropped keyframe recovers in ~1/6 s instead of ~1/2 s, at a modest
     * bitrate cost (more I-frames). Overridable via YOLO_GOP. periodicity-idr is
     * still DELIBERATELY omitted (it zeroes this Allegro OMX's output). */
    int gop = 10;
    if (const char* e = getenv("YOLO_GOP")) {
        int g = atoi(e);
        if (g > 0) gop = g;
    }

    char desc[1024];
    std::snprintf(desc, sizeof(desc),
        /* --- H.265 low-bandwidth + low-latency + anti-macroblock tuning (2026-06-27) ---
         * omxh265enc (Renesas VCD): HEVC at ~40-50% the bitrate of H.264 for the
         *   same subjective quality -> ideal for a long-range / low-bandwidth WiFi
         *   link. All the proven H.264 knobs carry over 1:1 (same OMX backend).
         * control-rate=2 (CBR): VBR (cr=1) lets bitrate explode on hard/motion
         *   frames, which overflows the radio link budget -> packet loss ->
         *   macroblocking. CBR holds a steady ceiling (~ target, +IDR overhead).
         * interval-intraframes (Phase1: default 10, ~0.17s GOP @60fps, was 30):
         *   PERIODIC I-frames so any loss/macroblock self-heals fast instead of
         *   sticking forever. Shorter GOP = faster gray-out recovery. YOLO_GOP env.
         * b-frames=0: no B-frames -> no reorder latency and loss does not
         *   propagate along a B-pyramid.
         * NOTE: periodicity-idr is DELIBERATELY OMITTED (same as the H.264 build).
         *   Adding it on this Allegro OMX is the one combination empirically
         *   linked to a zero-frame / undecodable RX stream; interval-intraframes
         *   alone already yields periodic IDRs and decodes cleanly (nvh265dec).
         * mtu=1400: keep RTP payload under the path MTU (no IP fragmentation). */
        "appsrc name=src is-live=true format=time stream-type=0 "
        "do-timestamp=false block=false leaky-type=downstream max-bytes=%zu ! "
        "vspmfilter dmabuf-use=true ! "
        "video/x-raw,format=NV12,width=%d,height=%d,framerate=%d/1 ! "
        "omxh265enc name=venc use-dmabuf=true control-rate=2 target-bitrate=%d "
        "interval-intraframes=%d b-frames=0 ! "
        "h265parse name=h265parse config-interval=1 ! "
        "video/x-h265,stream-format=byte-stream,alignment=au ! "
        "rtph265pay pt=96 mtu=1400 config-interval=-1 ! "
        "udpsink host=%s port=%d sync=false async=false",
        (size_t)((size_t)w * h * 2 * 3), w, h, g_enc.fps, bitrate, gop, host, port);
    std::fprintf(stderr, "[stream_enc] GOP interval-intraframes=%d (YOLO_GOP)\n", gop);

    GError* perr = nullptr;
    g_enc.pipeline = gst_parse_launch(desc, &perr);
    if (!g_enc.pipeline || perr) {
        std::fprintf(stderr, "[stream_enc] gst_parse_launch failed: %s\n", perr ? perr->message : "(null)");
        if (perr) g_error_free(perr);
        if (g_enc.pipeline) { gst_object_unref(g_enc.pipeline); g_enc.pipeline = nullptr; }
        return -1;
    }
    g_enc.appsrc = gst_bin_get_by_name(GST_BIN(g_enc.pipeline), "src");
    if (!g_enc.appsrc) {
        std::fprintf(stderr, "[stream_enc] appsrc 'src' not found\n");
        gst_object_unref(g_enc.pipeline); g_enc.pipeline = nullptr; return -1;
    }
    /* app_m4: grab the encoder for runtime adaptive-bitrate control. */
    g_enc.venc = gst_bin_get_by_name(GST_BIN(g_enc.pipeline), "venc");
    if (!g_enc.venc)
        std::fprintf(stderr, "[stream_enc] WARN: omxh265enc 'venc' not found -- adaptive bitrate disabled\n");
    /* D3 P0-1d: on-demand IDR throttle. GstVideoEncoder exposes
     * min-force-key-unit-interval (guint64 ns) -- floor between two honored
     * force-key-unit requests. Set from YOLO_IDR_MIN_INTERVAL_MS (default
     * 1112ms, alink-borrowed assumption per D3 §4). Property may be absent on
     * this Allegro OMX (board-verify), so probe with g_object_class_find_property
     * before setting. This is the encoder-side half of the double throttle; the
     * TX-side half lives in capture_encoder.cpp's REQ_IDR handler.
     * GATED on YOLO_ONDEMAND_IDR (default OFF): origin never set this property, so
     * with on-demand IDR off we leave it untouched to stay bit-equivalent. */
    bool ondemand_idr = (getenv("YOLO_ONDEMAND_IDR") &&
                         atoi(getenv("YOLO_ONDEMAND_IDR")) == 1);
    if (ondemand_idr && g_enc.venc &&
        g_object_class_find_property(G_OBJECT_GET_CLASS(g_enc.venc),
                                     "min-force-key-unit-interval")) {
        guint idr_min_ms = 1112;
        if (const char* e = getenv("YOLO_IDR_MIN_INTERVAL_MS")) {
            int v = atoi(e); if (v >= 0) idr_min_ms = (guint)v;
        }
        g_object_set(g_enc.venc, "min-force-key-unit-interval",
                     (guint64)idr_min_ms * GST_MSECOND, NULL);
        std::fprintf(stderr, "[stream_enc] min-force-key-unit-interval=%ums (YOLO_IDR_MIN_INTERVAL_MS)\n",
                     idr_min_ms);
    } else if (ondemand_idr && g_enc.venc) {
        std::fprintf(stderr, "[stream_enc] min-force-key-unit-interval absent -- on-demand IDR relies on TX-side throttle only\n");
    }
    /* app_m5: keep a ref to h265parse for the SEI-inject probe (added on enable). */
    g_enc.h265parse = gst_bin_get_by_name(GST_BIN(g_enc.pipeline), "h265parse");
    if (!g_enc.h265parse)
        std::fprintf(stderr, "[stream_enc] WARN: h265parse not found -- SEI box embed disabled\n");
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "YUY2", "width", G_TYPE_INT, w,
        "height", G_TYPE_INT, h, "framerate", GST_TYPE_FRACTION, g_enc.fps, 1, nullptr);
    /* NOTE: plain caps (NO memory:DMABuf feature) -- vspmfilter accepts plain
     * video/x-raw and detects dma-buf at the buffer level (gst_is_dmabuf_memory)
     * then mmngr_import_start_in_user_ext()s the fd, which works cross-process.
     * The "could not copy metadata" warning (gstvspmfilter.c:841) is benign. */
    gst_app_src_set_caps(GST_APP_SRC(g_enc.appsrc), caps);
    gst_caps_unref(caps);
    g_object_set(g_enc.appsrc, "format", GST_FORMAT_TIME, "is-live", TRUE,
                 "block", FALSE, "leaky-type", 2,
                 "max-bytes", (guint64)((guint64)w * h * 2 * 3), nullptr);

    g_enc.dmabuf_alloc = gst_dmabuf_allocator_new();
    if (!g_enc.dmabuf_alloc) {
        std::fprintf(stderr, "[stream_enc] gst_dmabuf_allocator_new failed\n");
        gst_object_unref(g_enc.appsrc); g_enc.appsrc = nullptr;
        gst_object_unref(g_enc.pipeline); g_enc.pipeline = nullptr; return -1;
    }
    g_enc.bus = gst_element_get_bus(g_enc.pipeline);
    if (gst_element_set_state(g_enc.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        std::fprintf(stderr, "[stream_enc] set_state(PLAYING) failed\n");
        gst_object_unref(g_enc.bus); g_enc.bus = nullptr;
        gst_object_unref(g_enc.dmabuf_alloc); g_enc.dmabuf_alloc = nullptr;
        gst_object_unref(g_enc.appsrc); g_enc.appsrc = nullptr;
        gst_object_unref(g_enc.pipeline); g_enc.pipeline = nullptr; return -1;
    }
    g_enc.running.store(true);
    g_enc.bus_thr = std::thread(bus_drain_loop);
    std::fprintf(stderr, "[stream_enc] streaming H.265 -> %s:%d (%dx%d@%d, ceiling %d bps)\n",
                 host, port, w, h, g_enc.fps, bitrate);
    return 0;
}

int stream_enc_push(int dbuf_fd, size_t size, uint64_t pts_ns,
                    void (*on_release)(void* user), void* user) {
    if (!g_enc.running.load() || !g_enc.appsrc) {
        if (on_release) on_release(user); return -1;
    }
    int dfd = dup(dbuf_fd);
    if (dfd < 0) { std::perror("[stream_enc] dup"); if (on_release) on_release(user); return -1; }
    GstMemory* mem = gst_dmabuf_allocator_alloc(g_enc.dmabuf_alloc, dfd, (gsize)size);
    if (!mem) { close(dfd); if (on_release) on_release(user); return -1; }

    ReleaseCtx* ctx = new ReleaseCtx{on_release, user};
    g_enc.inflight.fetch_add(1, std::memory_order_acq_rel);
    gst_mini_object_set_qdata(GST_MINI_OBJECT(mem), release_quark(), ctx, release_notify);

    GstBuffer* buf = gst_buffer_new();
    gst_buffer_append_memory(buf, mem);
    GST_BUFFER_PTS(buf)      = (GstClockTime)pts_ns;
    GST_BUFFER_DTS(buf)      = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buf) = GST_SECOND / (guint)g_enc.fps;

    (void)gst_app_src_push_buffer(GST_APP_SRC(g_enc.appsrc), buf);
    return 0;
}

void stream_enc_set_bitrate(int bps) {
    if (!g_enc.running.load() || !g_enc.venc || bps <= 0) return;
    /* target-bitrate is a guint (bits/sec), live-settable in PLAYING. g_object_set
     * is GObject-thread-safe so the AIMD thread calls this directly. */
    g_object_set(g_enc.venc, "target-bitrate", (guint)bps, NULL);
}

void stream_enc_request_idr(void) {
    if (!g_enc.running.load() || !g_enc.venc) return;
    /* D3 P0-1d: push an upstream force-key-unit event into venc -> it emits an
     * IDR on the next AU. Mirrors stream_enc_set_bitrate(): acts on venc,
     * thread-safe. gst_element_send_event() routes an upstream event onto venc's
     * source pad, so it travels upstream into the encoder. The all-request=TRUE
     * variant asks for a full IDR (SPS/PPS + I). Throttling is handled by the
     * encoder's min-force-key-unit-interval + the caller's TX-side gate. */
    gst_element_send_event(g_enc.venc,
        gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0));
}

void stream_enc_enable_sei(void) {
    if (!g_enc.h265parse) {
        std::fprintf(stderr, "[stream_enc] SEI requested but h265parse missing -- ignored\n");
        return;
    }
    GstPad* src = gst_element_get_static_pad(g_enc.h265parse, "src");
    if (!src) { std::fprintf(stderr, "[stream_enc] h265parse src pad missing -- SEI off\n"); return; }
    gst_pad_add_probe(src, GST_PAD_PROBE_TYPE_BUFFER, sei_inject_probe, nullptr, nullptr);
    gst_object_unref(src);
    g_enc.sei_enabled.store(true);
    std::fprintf(stderr, "[stream_enc] SEI box embedding ENABLED (UUID=DETBOXSEI01_DRPI)\n");
}

void stream_enc_set_sei_payload(const unsigned char* payload, size_t len) {
    if (len > sizeof(g_enc.sei_payload)) len = sizeof(g_enc.sei_payload);
    std::lock_guard<std::mutex> lk(g_enc.sei_mtx);
    /* L2: only bump the epoch when the payload bytes actually change, so the
     * per-AU probe skips the full-AU re-copy on the ~40/60 frames between
     * detections. (Same det boxes/roll re-sent for an unchanged frame -> no-op.) */
    bool changed = (len != g_enc.sei_payload_len) ||
                   (len && std::memcmp(g_enc.sei_payload, payload, len) != 0);
    if (len) std::memcpy(g_enc.sei_payload, payload, len);
    g_enc.sei_payload_len = len;
    if (changed) g_enc.sei_epoch.fetch_add(1, std::memory_order_release);
}

void stream_enc_stop(void) {
    if (!g_enc.running.load()) return;
    g_enc.running.store(false);
    if (g_enc.appsrc) gst_app_src_end_of_stream(GST_APP_SRC(g_enc.appsrc));
    if (g_enc.bus_thr.joinable()) g_enc.bus_thr.join();
    if (g_enc.pipeline) {
        gst_element_set_state(g_enc.pipeline, GST_STATE_NULL);
        gst_element_get_state(g_enc.pipeline, nullptr, nullptr, GST_CLOCK_TIME_NONE);
        for (int i = 0; i < 500 && g_enc.inflight.load() > 0; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (g_enc.bus)          { gst_object_unref(g_enc.bus);          g_enc.bus = nullptr; }
    if (g_enc.h265parse)    { gst_object_unref(g_enc.h265parse);    g_enc.h265parse = nullptr; }
    if (g_enc.venc)         { gst_object_unref(g_enc.venc);         g_enc.venc = nullptr; }
    if (g_enc.appsrc)       { gst_object_unref(g_enc.appsrc);       g_enc.appsrc = nullptr; }
    if (g_enc.dmabuf_alloc) { gst_object_unref(g_enc.dmabuf_alloc); g_enc.dmabuf_alloc = nullptr; }
    if (g_enc.pipeline)     { gst_object_unref(g_enc.pipeline);     g_enc.pipeline = nullptr; }
}
