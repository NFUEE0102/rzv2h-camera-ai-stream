/*
 * sei_roll_harness.cpp -- BOARD self-test of the child SEI roll path on the REAL
 * omxh265enc, with NO app and NO camera. It drives the same stream_enc.cpp the
 * child uses: init the omxh265enc->h265parse->rtph265pay->udpsink pipeline, enable
 * SEI, set a ver==2 payload carrying a known roll (+20.00 deg) + one box, push a
 * handful of dummy YUY2 dma-buf frames, and stream to 127.0.0.1:<port>. A captor
 * (gst filesink) records the elementary stream; we then grep for the UUID and the
 * Python parser confirms roll==+20.00 from the REAL encoded bytes.
 *
 * Build on board:
 *   g++ -std=c++17 -O2 -I. $(pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 \
 *       gstreamer-allocators-1.0 gstreamer-video-1.0) \
 *       sei_roll_harness.cpp stream_enc.cpp -o sei_roll_harness \
 *       $(pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-allocators-1.0 \
 *       gstreamer-video-1.0) -lmmngr -lmmngrbuf -lpthread
 * Run:
 *   ./sei_roll_harness <host> <port> <nframes>
 */
#include "stream_enc.h"
#include "sei_box.h"
extern "C" {
#include "mmngr_user_public.h"
#include "mmngr_buf_user_public.h"
}
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>

#define W 1920
#define H 1080
#define FRAME_SIZE (W * H * 2)   /* YUY2 */

static std::atomic<int> g_released{0};
static void on_rel(void*) { g_released.fetch_add(1); }

int main(int argc, char** argv) {
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : 50020;
    int nframes = (argc > 3) ? atoi(argv[3]) : 120;

    /* one mmngr YUY2 buffer, filled mid-gray, reused for every push */
    MMNGR_ID id; unsigned int phys = 0; void* virt = nullptr; int dfd = -1;
    if (mmngr_alloc_in_user_ext(&id, FRAME_SIZE, &phys, &virt, MMNGR_VA_SUPPORT, NULL) != R_MM_OK) {
        std::fprintf(stderr, "mmngr_alloc failed\n"); return 1;
    }
    uint8_t* p = (uint8_t*)virt;
    for (size_t k = 0; k + 3 < FRAME_SIZE; k += 4) { p[k]=128; p[k+1]=128; p[k+2]=128; p[k+3]=128; }
    if (mmngr_export_start_in_user_ext(&id, FRAME_SIZE, phys, &dfd, NULL) != R_MM_OK || dfd < 0) {
        std::fprintf(stderr, "mmngr_export failed\n"); return 1;
    }

    if (stream_enc_init(host, port, 4000000, W, H, 60) != 0) {
        std::fprintf(stderr, "stream_enc_init failed\n"); return 1;
    }
    stream_enc_enable_sei();

    /* ver==2 payload: 1 person box + roll = +20.00 deg */
    SeiBoxRec rec; sei_fill_rec(&rec, 14, 0.9f, 960, 540, 100, 120);
    unsigned char payload[sizeof(SeiBoxHeader) + sizeof(SeiBoxRec)];
    SeiBoxHeader h; h.frame_seq = 1; h.n = 1;
    h.ver = SEI_BOX_VER; h.flags = SEI_FLAG_ROLL_VALID; h.roll_cdeg = 2000;
    std::memcpy(payload, &h, sizeof(h));
    std::memcpy(payload + sizeof(h), &rec, sizeof(rec));
    stream_enc_set_sei_payload(payload, sizeof(payload));

    std::fprintf(stderr, "[harness] streaming %d frames -> %s:%d, roll=+20.00 in SEI\n",
                 nframes, host, port);
    uint64_t pts = 0;
    for (int i = 0; i < nframes; i++) {
        /* bump frame_seq each frame so the captured SEIs aren't all identical */
        h.frame_seq = (uint32_t)i;
        std::memcpy(payload, &h, sizeof(h));
        stream_enc_set_sei_payload(payload, sizeof(payload));
        stream_enc_push(dfd, FRAME_SIZE, pts, on_rel, nullptr);
        pts += 1000000000ULL / 60;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    stream_enc_stop();
    std::fprintf(stderr, "[harness] done (released=%d)\n", g_released.load());
    close(dfd);
    return 0;
}
