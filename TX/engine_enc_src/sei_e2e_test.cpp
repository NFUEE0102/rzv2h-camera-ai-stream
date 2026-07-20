/*
 * sei_e2e_test.cpp -- FAITHFUL end-to-end test of the ver==2 (roll-bearing) SEI
 * through the REAL board encode+packetize chain, WITHOUT the camera/dma-buf:
 *
 *   videotestsrc -> NV12 -> omxh265enc(CBR) -> h265parse(config-interval=1)
 *     -> [SEI-inject probe on h265parse SRC, identical logic to stream_enc.cpp]
 *     -> rtph265pay -> udpsink host:port
 *
 * The probe builds the SAME HEVC PREFIX_SEI as the child (sei_build_nal_h265 from
 * sei_box.h) carrying a ver==2 header with roll=+20.00 deg + one box, and prepends
 * it to every AU -- exactly as the deployed child does. A captor records the RTP;
 * the Python RX parser then confirms the roll/box decode from the REAL encoded,
 * packetized, depacketized bytes. This isolates "does the new payload survive
 * omxh265enc + rtph265pay" from the camera buffer plumbing.
 *
 * Build on board:
 *   g++ -std=c++17 -O2 -I. $(pkg-config --cflags gstreamer-1.0 gstreamer-video-1.0) \
 *       sei_e2e_test.cpp -o sei_e2e_test \
 *       $(pkg-config --libs gstreamer-1.0 gstreamer-video-1.0) -lpthread
 * Run:  ./sei_e2e_test <host> <port> <num_buffers>
 */
#include "sei_box.h"
#include <gst/gst.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static unsigned char g_payload[sizeof(SeiBoxHeader) + sizeof(SeiBoxRec)];
static size_t g_plen = 0;

static GstPadProbeReturn inject(GstPad*, GstPadProbeInfo* info, gpointer) {
    GstBuffer* inbuf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!inbuf || g_plen == 0) return GST_PAD_PROBE_OK;
    unsigned char sei[(16 + sizeof(SeiBoxHeader) + sizeof(SeiBoxRec)) * 2 + 64];
    size_t sei_len = sei_build_nal_h265(g_payload, g_plen, sei);
    GstMapInfo mi;
    if (!gst_buffer_map(inbuf, &mi, GST_MAP_READ)) return GST_PAD_PROBE_OK;
    GstBuffer* out = gst_buffer_new_allocate(nullptr, sei_len + mi.size, nullptr);
    gst_buffer_fill(out, 0, sei, sei_len);
    gst_buffer_fill(out, sei_len, mi.data, mi.size);
    gst_buffer_unmap(inbuf, &mi);
    GST_BUFFER_PTS(out) = GST_BUFFER_PTS(inbuf);
    GST_BUFFER_DTS(out) = GST_BUFFER_DTS(inbuf);
    GST_BUFFER_DURATION(out) = GST_BUFFER_DURATION(inbuf);
    GST_BUFFER_FLAGS(out) = GST_BUFFER_FLAGS(inbuf);
    GST_PAD_PROBE_INFO_DATA(info) = out;
    gst_buffer_unref(inbuf);
    return GST_PAD_PROBE_OK;
}

int main(int argc, char** argv) {
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : 50020;
    int nbuf = (argc > 3) ? atoi(argv[3]) : 180;
    gst_init(&argc, &argv);

    /* ver==2 payload: 1 box + roll=+20.00 */
    SeiBoxRec rec; sei_fill_rec(&rec, 14, 0.9f, 960, 540, 100, 120);
    SeiBoxHeader h; h.frame_seq = 1; h.n = 1;
    h.ver = SEI_BOX_VER; h.flags = SEI_FLAG_ROLL_VALID; h.roll_cdeg = 2000;
    std::memcpy(g_payload, &h, sizeof(h));
    std::memcpy(g_payload + sizeof(h), &rec, sizeof(rec));
    g_plen = sizeof(g_payload);

    char desc[1024];
    std::snprintf(desc, sizeof(desc),
        "videotestsrc num-buffers=%d ! video/x-raw,format=NV12,width=1920,height=1080,framerate=60/1 ! "
        "omxh265enc control-rate=2 target-bitrate=4000000 interval-intraframes=30 b-frames=0 ! "
        "h265parse name=hp config-interval=1 ! video/x-h265,stream-format=byte-stream,alignment=au ! "
        "rtph265pay pt=96 mtu=1400 config-interval=-1 ! udpsink host=%s port=%d sync=false",
        nbuf, host, port);
    GError* e = nullptr;
    GstElement* pipe = gst_parse_launch(desc, &e);
    if (!pipe || e) { std::fprintf(stderr, "parse_launch: %s\n", e ? e->message : "?"); return 1; }
    GstElement* hp = gst_bin_get_by_name(GST_BIN(pipe), "hp");
    GstPad* src = gst_element_get_static_pad(hp, "src");
    gst_pad_add_probe(src, GST_PAD_PROBE_TYPE_BUFFER, inject, nullptr, nullptr);
    gst_object_unref(src); gst_object_unref(hp);

    std::fprintf(stderr, "[e2e] streaming %d frames -> %s:%d with ver==2 roll=+20.00 SEI\n",
                 nbuf, host, port);
    gst_element_set_state(pipe, GST_STATE_PLAYING);
    GstBus* bus = gst_element_get_bus(pipe);
    GstMessage* msg = gst_bus_timed_pop_filtered(bus, 30 * GST_SECOND,
        (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    if (msg) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError* err = nullptr; gchar* dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            std::fprintf(stderr, "[e2e] ERROR %s (%s)\n", err ? err->message : "?", dbg ? dbg : "");
        } else std::fprintf(stderr, "[e2e] EOS (done)\n");
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(pipe);
    return 0;
}
