/*
 * stream_enc.h -- in-process GStreamer zero-copy H.265 (HEVC) encode pipeline.
 *
 * Linked ONLY into the standalone capture_encoder child process (which links no
 * DRP-AI/TVM/OpenCV), so gst_init runs in a clean process.
 *
 * Pipeline: appsrc(YUY2 dma-buf) -> vspmfilter(dmabuf-use,NV12)
 *           -> omxh265enc(use-dmabuf, CBR) -> h265parse -> rtph265pay -> udpsink
 *
 * stream_enc_push() ownership contract:
 *   on_release(user) fires EXACTLY ONCE per call (success: later from the gst
 *   thread once the encoder released the dma-buf; early failure: synchronously
 *   before return). dbuf_fd is dup()'d, never closed by this call.
 */
#ifndef STREAM_ENC_H
#define STREAM_ENC_H

#include <cstddef>
#include <cstdint>

int  stream_enc_init(const char* host, int port, int bitrate,
                     int w, int h, int in_fps);

int  stream_enc_push(int dbuf_fd, size_t size, uint64_t pts_ns,
                     void (*on_release)(void* user), void* user);

/* app_m4 adaptive bitrate: live-set omxh264enc target-bitrate (bits/sec).
 * Thread-safe (GObject property locking); called from the feedback/AIMD thread.
 * target-bitrate is "changeable in PLAYING" per gst-inspect, so it takes effect
 * within ~one GOP (sub-second at config-interval=1). No-op if encoder absent. */
void stream_enc_set_bitrate(int bps);

/* D3 P0-1d on-demand IDR: request a single forced keyframe NOW (mirrors
 * stream_enc_set_bitrate() -- acts directly on venc, GObject/element-thread
 * safe, called from the feedback thread). Sends an upstream force-key-unit
 * event to omxh265enc. Throttled by the encoder's min-force-key-unit-interval
 * (set in stream_enc_init) PLUS a TX-side throttle in the caller (belt+braces).
 * No-op if encoder absent. */
void stream_enc_request_idr(void);

/* app_m5 SEI box embedding: enable an SEI-inject pad probe on the h264parse SRC
 * pad. When enabled, every outgoing AU buffer gets a user_data_unregistered SEI
 * NAL (sei_box.h layout) PREPENDED carrying the latest box payload. Call once
 * after stream_enc_init(); no-op (and stays off) if never called. */
void stream_enc_enable_sei(void);

/* Replace the box payload injected from now on. `payload` is the raw box
 * payload [SeiBoxHeader || n*SeiBoxRec] (NOT the UUID, NOT the NAL framing --
 * sei_build_nal() adds those). Copied internally; thread-safe (called from the
 * app->child IPC reader thread). len==0 clears -> no SEI emitted until set. */
void stream_enc_set_sei_payload(const unsigned char* payload, size_t len);

void stream_enc_stop(void);

#endif /* STREAM_ENC_H */
