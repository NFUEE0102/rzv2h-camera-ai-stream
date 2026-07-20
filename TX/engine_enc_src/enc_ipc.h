/*
 * enc_ipc.h -- IPC between the app (capture + DRP-AI) and the gst encoder child
 * (capture_encoder). Header-only, plain C/POSIX (no glib/gst) so the APP never links
 * gstreamer (gst_init conflicts with the TVM/LLVM runtime).
 *
 * Design B (inverted dma-buf ownership) -- the cross-process zero-copy works
 * because the CHILD owns the capture buffers:
 *   1. startup handshake, child -> app: N x BufMsg{index,size,phys_addr} + the
 *      mmngr dma-buf fd (SCM_RIGHTS). The app QBUFs these fds to the camera
 *      (V4L2_MEMORY_DMABUF, kernel-level import) and uses phys_addr for DRP-AI.
 *   2. per frame, app -> child: FrameMsg{index,pts_ns} (NO fd -- the child
 *      already holds every buffer). The child encodes its OWN native buffer, so
 *      vspmfilter's mmngr_import resolves correctly (same process).
 *   3. release, child -> app: RelMsg{index} -> app re-QBUFs that slot.
 */
#ifndef ENC_IPC_H
#define ENC_IPC_H

#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#pragma pack(push, 1)
struct BufMsg   { uint32_t index; uint32_t size; uint64_t phys_addr; };
struct FrameMsg { uint32_t index; uint64_t pts_ns; };
struct RelMsg   { uint32_t index; };
/* ---- app_m5 SEI path: latest detection box set, app -> child ----
 * Sent (when YOLO_SEI=1) right after R_Post_Proc whenever DRP-AI produces a
 * new det[]. The child caches it and injects it into every encoded AU as an
 * SEI NAL. Variable length: BoxSetMsg header on the wire is followed by
 * `n` * 11-byte SeiBoxRec (see sei_box.h). A distinct 4-byte magic lets the
 * child demux box-sets from FrameMsg on the same SOCK_STREAM socketpair. */
#define BOXSET_MAGIC 0x55425853u   /* 'UBXS' little-endian */
/* app_m5 "hz" horizon-lock: BoxSetMsg now also carries the latest IMU roll
 * (centi-degrees, signed) + a version/flags byte so the child can copy it
 * straight into the SEI header (ver==2). roll_valid in flags (bit0). Older
 * peers are not in play (we control both ends), but the ver byte keeps the
 * wire self-describing. */
struct BoxSetMsg { uint32_t magic; uint32_t frame_seq; uint16_t n;
                   uint8_t ver; uint8_t flags; int16_t roll_cdeg; };
#pragma pack(pop)

/* ---- SCM_RIGHTS fd transfer (used only for the startup buffer handshake) ---- */
static inline int enc_send_buf(int sock, const BufMsg& m, int fd) {
    struct iovec iov; iov.iov_base = (void*)&m; iov.iov_len = sizeof(m);
    char cbuf[CMSG_SPACE(sizeof(int))]; std::memset(cbuf, 0, sizeof(cbuf));
    struct msghdr msg; std::memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov; msg.msg_iovlen = 1;
    msg.msg_control = cbuf; msg.msg_controllen = sizeof(cbuf);
    struct cmsghdr* c = CMSG_FIRSTHDR(&msg);
    c->cmsg_level = SOL_SOCKET; c->cmsg_type = SCM_RIGHTS; c->cmsg_len = CMSG_LEN(sizeof(int));
    std::memcpy(CMSG_DATA(c), &fd, sizeof(int));
    ssize_t n = sendmsg(sock, &msg, MSG_NOSIGNAL);
    return (n == (ssize_t)sizeof(m)) ? 0 : -1;
}
static inline int enc_recv_buf(int sock, BufMsg* m, int* fd) {
    struct iovec iov; iov.iov_base = (void*)m; iov.iov_len = sizeof(*m);
    char cbuf[CMSG_SPACE(sizeof(int))]; std::memset(cbuf, 0, sizeof(cbuf));
    struct msghdr msg; std::memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov; msg.msg_iovlen = 1;
    msg.msg_control = cbuf; msg.msg_controllen = sizeof(cbuf);
    ssize_t n = recvmsg(sock, &msg, MSG_WAITALL);
    if (n == 0) return 0;
    if (n != (ssize_t)sizeof(*m)) return -1;
    *fd = -1;
    for (struct cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c))
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS) {
            std::memcpy(fd, CMSG_DATA(c), sizeof(int)); break;
        }
    return (*fd >= 0) ? 1 : -1;
}

/* ---- per-frame index (no fd): app -> child ---- */
static inline int enc_send_frame(int sock, const FrameMsg& m) {
    /* DONTWAIT: never block the capture thread; caller re-QBUFs on failure. */
    return (send(sock, &m, sizeof(m), MSG_NOSIGNAL | MSG_DONTWAIT) == (ssize_t)sizeof(m)) ? 0 : -1;
}
static inline int enc_recv_frame(int sock, FrameMsg* m) {
    ssize_t n = recv(sock, m, sizeof(*m), MSG_WAITALL);
    if (n == 0) return 0;
    return (n == (ssize_t)sizeof(*m)) ? 1 : -1;
}

/* ---- app_m5 SEI box set: app -> child (variable length) ----
 * Wire = BoxSetMsg{magic,frame_seq,n} immediately followed by n*rec_bytes box
 * records (rec_bytes = sizeof(SeiBoxRec) = 11; passed in so this header stays
 * gst/sei_box-free). The app sends from the inference thread; non-blocking so a
 * full socket just drops the update (next det overwrites it anyway). */
static inline int enc_send_boxset(int sock, uint32_t frame_seq,
                                  const void* recs, uint16_t n, size_t rec_bytes,
                                  uint8_t ver, uint8_t flags, int16_t roll_cdeg) {
    BoxSetMsg h; h.magic = BOXSET_MAGIC; h.frame_seq = frame_seq; h.n = n;
    h.ver = ver; h.flags = flags; h.roll_cdeg = roll_cdeg;
    struct iovec iov[2];
    iov[0].iov_base = (void*)&h;          iov[0].iov_len = sizeof(h);
    iov[1].iov_base = (void*)recs;        iov[1].iov_len = (size_t)n * rec_bytes;
    struct msghdr msg; std::memset(&msg, 0, sizeof(msg));
    msg.msg_iov = iov; msg.msg_iovlen = (n > 0) ? 2 : 1;
    ssize_t want = (ssize_t)(sizeof(h) + (size_t)n * rec_bytes);
    return (sendmsg(sock, &msg, MSG_NOSIGNAL | MSG_DONTWAIT) == want) ? 0 : -1;
}

/* Unified child receive: demuxes FrameMsg vs BoxSetMsg on the shared stream by
 * peeking the leading 4 bytes (BoxSetMsg.magic == BOXSET_MAGIC; FrameMsg.index
 * is a small ring index 0..7, never the magic).
 *   return  1 -> *is_box=0, FrameMsg in *fm
 *   return  1 -> *is_box=1, BoxSetMsg in *bm, up to *n_out recs in recs[]
 *   return  0 -> peer closed; -1 -> error
 * rec_bytes = sizeof(SeiBoxRec). On a box set with more recs than cap, the
 * extra recs are still drained from the socket (so framing stays in sync) but
 * not stored. */
static inline int enc_recv_msg(int sock, int* is_box, FrameMsg* fm,
                               BoxSetMsg* bm, void* recs, uint16_t cap,
                               size_t rec_bytes) {
    uint32_t magic = 0;
    ssize_t p = recv(sock, &magic, sizeof(magic), MSG_PEEK | MSG_WAITALL);
    if (p == 0) return 0;
    if (p != (ssize_t)sizeof(magic)) return -1;
    if (magic == BOXSET_MAGIC) {
        *is_box = 1;
        ssize_t n = recv(sock, bm, sizeof(*bm), MSG_WAITALL);
        if (n == 0) return 0;
        if (n != (ssize_t)sizeof(*bm)) return -1;
        size_t total = (size_t)bm->n * rec_bytes;
        size_t keep  = ((size_t)cap < (size_t)bm->n ? (size_t)cap : (size_t)bm->n) * rec_bytes;
        size_t got = 0;
        while (got < total) {
            unsigned char tmp[256];
            size_t want = total - got;
            if (got < keep) {
                size_t k = keep - got; if (k > want) k = want;
                ssize_t r = recv(sock, (char*)recs + got, k, MSG_WAITALL);
                if (r <= 0) return (r == 0) ? 0 : -1;
                got += (size_t)r;
            } else {
                size_t k = want > sizeof(tmp) ? sizeof(tmp) : want;
                ssize_t r = recv(sock, tmp, k, MSG_WAITALL);
                if (r <= 0) return (r == 0) ? 0 : -1;
                got += (size_t)r;
            }
        }
        return 1;
    }
    *is_box = 0;
    ssize_t n = recv(sock, fm, sizeof(*fm), MSG_WAITALL);
    if (n == 0) return 0;
    return (n == (ssize_t)sizeof(*fm)) ? 1 : -1;
}

/* ---- release ack: child -> app ---- */
static inline int enc_send_rel(int sock, uint32_t index) {
    RelMsg r{index};
    return (send(sock, &r, sizeof(r), MSG_NOSIGNAL) == (ssize_t)sizeof(r)) ? 0 : -1;
}
static inline int enc_recv_rel(int sock, uint32_t* index) {
    RelMsg r; ssize_t n = recv(sock, &r, sizeof(r), MSG_WAITALL);
    if (n == 0) return 0;
    if (n != (ssize_t)sizeof(r)) return -1;
    *index = r.index; return 1;
}

#endif /* ENC_IPC_H */
