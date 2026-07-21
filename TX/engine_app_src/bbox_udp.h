/*
 * bbox_udp.h -- compact-JSON detection metadata over UDP. gst-free, so the APP
 * (which links the DRP-AI/TVM runtime) can use it WITHOUT linking gstreamer.
 */
#ifndef BBOX_UDP_H
#define BBOX_UDP_H

#include <cstdint>
#include <vector>

struct detection;   /* real def in common_files/box.h */

int  bbox_udp_init(const char* host, int port);   /* 0 ok, -1 fail */
/* app_m5 "hz": roll_cdeg = IMU roll in centi-degrees (signed); roll_valid gates
 * whether the "roll" JSON field is emitted (so a stale/absent IMU is explicit). */
void bbox_udp_send(const std::vector<detection>& det, int seq,
                   uint64_t ts_ms, int w, int h,
                   int roll_cdeg, bool roll_valid);
void bbox_udp_close(void);

#endif /* BBOX_UDP_H */
