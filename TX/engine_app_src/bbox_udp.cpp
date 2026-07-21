/*
 * bbox_udp.cpp -- compact-JSON detection metadata over UDP (gst-free).
 * Linked into the APP (no gstreamer) so it never triggers the in-process
 * gst_init vs TVM/LLVM crash. The H.264 stream goes out the separate uav_enc.
 */
#include "bbox_udp.h"
#include "box.h"             /* real `struct detection` (common_files/box.h) */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <cstring>
#include <mutex>

namespace {
int                g_bbox_fd = -1;
struct sockaddr_in g_bbox_addr;
std::mutex         g_bbox_mtx;
}

int bbox_udp_init(const char* host, int port) {
    std::lock_guard<std::mutex> lk(g_bbox_mtx);
    g_bbox_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_bbox_fd < 0) { std::perror("[bbox] socket"); return -1; }
    int fl = fcntl(g_bbox_fd, F_GETFL, 0);
    fcntl(g_bbox_fd, F_SETFL, fl | O_NONBLOCK);    /* never block inference */
    std::memset(&g_bbox_addr, 0, sizeof(g_bbox_addr));
    g_bbox_addr.sin_family = AF_INET;
    g_bbox_addr.sin_port   = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &g_bbox_addr.sin_addr) != 1) {
        std::fprintf(stderr, "[bbox] bad host '%s'\n", host);
        close(g_bbox_fd); g_bbox_fd = -1; return -1;
    }
    std::fprintf(stderr, "[bbox] sending detections -> %s:%d\n", host, port);
    return 0;
}

void bbox_udp_send(const std::vector<detection>& det, int seq,
                   uint64_t ts_ms, int w, int h,
                   int roll_cdeg, bool roll_valid) {
    std::lock_guard<std::mutex> lk(g_bbox_mtx);
    if (g_bbox_fd < 0) return;

    char json[8192];
    /* app_m5 "hz": emit roll (degrees) when the IMU reading is valid, so a
     * SEI-less / JSON-only viewer can still horizon-lock. */
    char rollbuf[48]; rollbuf[0] = '\0';
    if (roll_valid)
        std::snprintf(rollbuf, sizeof(rollbuf), "\"roll\":%.2f,", roll_cdeg / 100.0);
    int n = std::snprintf(json, sizeof(json),
        "{\"seq\":%d,\"ts_ms\":%llu,\"w\":%d,\"h\":%d,%s\"det\":[",
        seq, (unsigned long long)ts_ms, w, h, rollbuf);

    const size_t MAX_DET = 64;
    size_t count = det.size() < MAX_DET ? det.size() : MAX_DET;
    bool first = true;
    for (size_t i = 0; i < count; ++i) {
        if (n < 0 || (size_t)n >= sizeof(json) - 96) break;
        const detection& d = det[i];
        /* Skip NMS-suppressed boxes (prob zeroed by filter_boxes_nms) so the PC
         * overlay only draws the kept detections. */
        if (d.prob <= 0.0f) continue;
        n += std::snprintf(json + n, sizeof(json) - n,
            "%s{\"x\":%.1f,\"y\":%.1f,\"w\":%.1f,\"h\":%.1f,\"p\":%.3f,\"c\":%d}",
            (first ? "" : ","),
            d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h, d.prob, d.c);
        first = false;
    }
    if (n < 0 || (size_t)n >= sizeof(json) - 3) n = (int)sizeof(json) - 3;
    n += std::snprintf(json + n, sizeof(json) - n, "]}");

    (void)sendto(g_bbox_fd, json, (size_t)n, MSG_DONTWAIT,
                 (struct sockaddr*)&g_bbox_addr, sizeof(g_bbox_addr));
}

void bbox_udp_close(void) {
    std::lock_guard<std::mutex> lk(g_bbox_mtx);
    if (g_bbox_fd >= 0) { close(g_bbox_fd); g_bbox_fd = -1; }
}
