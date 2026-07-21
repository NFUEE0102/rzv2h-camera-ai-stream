/*
 * sei_box.h -- H.264 SEI user_data_unregistered detection-box payload (Design B,
 * SEI path "app_m5"). Header-only, plain C/POSIX (no glib/gst), so it can be
 * shared by BOTH the app (sends boxes over the existing socketpair) and the
 * gst encoder child (builds + injects the SEI NAL). The PC viewer re-implements
 * the SAME byte layout in Python (uav_view.py).
 *
 * ---------------------------------------------------------------------------
 * WHY: instead of (or in addition to) the separate UDP-JSON bbox sidecar on
 * :50012, embed the YOLOX boxes for a frame INTO that frame's H.264 access
 * unit as an SEI NAL. Result: the boxes ride the SAME RTP stream, perfectly
 * frame-synced, single transport. A standard decoder ignores unknown SEI, so
 * the video still decodes everywhere; only our viewer parses the SEI.
 *
 * ===========================================================================
 *  ON-WIRE FORMAT (documented + reproducible)
 * ===========================================================================
 *  The SEI NAL is byte-stream (Annex-B) form, PREPENDED to the AU buffer:
 *
 *    00 00 00 01            4-byte start code
 *    06                     NAL header: forbidden_zero=0, nal_ref_idc=0,
 *                           nal_unit_type=6 (SEI)
 *    05                     payloadType = 5 (user_data_unregistered)
 *    <payloadSize>          ff ff ... xx  (one or more bytes, each <=255,
 *                           last <255; payloadSize = 16 UUID + box payload)
 *    <16-byte UUID>         SEI_BOX_UUID below (ASCII "UAVBOXSEI01_DRPI")
 *    <box payload>          SeiBoxHeader + N * SeiBoxRec  (see structs below)
 *    80                     rbsp_trailing_bits (stop-bit byte)
 *
 *  EMULATION PREVENTION: after the NAL header byte (0x06), the whole remaining
 *  byte stream (payloadType .. 0x80 trailing) is RBSP and MUST be escaped:
 *  every 00 00 00 / 00 00 01 / 00 00 02 / 00 00 03 has a 0x03 inserted ->
 *  00 00 03 00 / 00 00 03 01 / .... The receiver removes 0x03 in 00 00 03.
 *  build_sei_nal()/parse helpers below do this for you.
 *
 *  BOX PAYLOAD (all little-endian):
 *    struct SeiBoxHeader { uint32 frame_seq; uint16 n;        (frame_seq, #boxes)
 *                          uint8  ver; uint8 flags;           (SEI_BOX_VER, flags)
 *                          int16  roll_cdeg; }                (10 bytes)
 *    struct SeiBoxRec    { uint8 cls;                          (class id 0..255)
 *                          uint16 conf_q15;                    (prob*32767, clamp)
 *                          int16 x, y, w, h; }                 (11 bytes each)
 *  Coordinates are CENTER-based pixels in the 1920x1080 frame space (same as
 *  the UDP-JSON path: det.bbox is center x/y + w/h). conf_q15 = round(prob*
 *  32767) clamped to [0,32767]; decode prob = conf_q15 / 32767.0.
 *
 *  HORIZON LOCK (app_m5 "hz"): the header now carries a 1-byte version, a 1-byte
 *  flags field and an int16 roll in CENTI-DEGREES (roll*100, signed). ver==2 ->
 *  roll fields present and meaningful when (flags & SEI_FLAG_ROLL_VALID). The RX
 *  rotates the decoded frame + boxes by -roll about image center so the horizon
 *  stays level as the airframe banks. A receiver that only knows ver==1 (legacy)
 *  must read frame_seq+n then STOP at offset 6 -- but since we control both ends,
 *  both parsers read the full 10-byte header; the ver byte keeps it self-
 *  describing and future-proof. roll = roll_cdeg / 100.0 degrees.
 *
 *  Max boxes capped at SEI_BOX_MAX (64); excess are dropped (matches JSON path).
 * ===========================================================================
 */
#ifndef SEI_BOX_H
#define SEI_BOX_H

#include <cstdint>
#include <cstddef>
#include <cmath>

/* 16-byte UUID for our user_data_unregistered SEI. ASCII "UAVBOXSEI01_DRPI"
 * (greppable in a raw capture). DO NOT CHANGE without bumping the version in
 * the payload header consumers. */
static const unsigned char SEI_BOX_UUID[16] = {
    'U','A','V','B','O','X','S','E','I','0','1','_','D','R','P','I'
};

#define SEI_BOX_MAX 64

/* Payload header version + flags (horizon-lock). ver==2 adds the roll fields. */
#define SEI_BOX_VER          2
#define SEI_FLAG_ROLL_VALID  0x01   /* roll_cdeg is a live IMU reading */

#pragma pack(push, 1)
struct SeiBoxHeader { uint32_t frame_seq; uint16_t n;             /* 6 bytes */
                      uint8_t ver; uint8_t flags;                 /* +2 = 8  */
                      int16_t roll_cdeg; };                       /* +2 = 10 bytes */
struct SeiBoxRec    { uint8_t cls; uint16_t conf_q15;
                      int16_t x, y, w, h; };                       /* 11 bytes */
#pragma pack(pop)

/* Boxes the app forwards to the encoder child over the socketpair (BoxSetMsg
 * in enc_ipc.h). Already quantized to the wire format so the child only has to
 * concatenate UUID + header + recs and escape. Kept separate from SeiBoxRec
 * only for clarity; layout is identical. */

/* ---- inject side (encoder child): build the full Annex-B SEI NAL ---- */

/* Append v to out[] applying H.264 emulation-prevention against the running
 * RBSP tail (last two emitted RBSP bytes tracked by z2). Used for every RBSP
 * byte AFTER the 1-byte NAL header. */
static inline void sei_emit_rbsp_byte(unsigned char* out, size_t* olen,
                                      int* zero_run, unsigned char v) {
    if (*zero_run >= 2 && v <= 3) {
        out[(*olen)++] = 0x03;     /* emulation_prevention_three_byte */
        *zero_run = 0;
    }
    out[(*olen)++] = v;
    if (v == 0) (*zero_run)++; else *zero_run = 0;
}

/* Build a complete Annex-B SEI NAL (00 00 00 01 06 ... 80, escaped) carrying
 * the box payload [hdr || recs]. payload = SeiBoxHeader followed by hdr.n
 * SeiBoxRec, all little-endian, length = payload_len.
 * Returns total bytes written to out (out must be >= payload_len*2 + 32). */
static inline size_t sei_build_nal(const unsigned char* payload,
                                   size_t payload_len,
                                   unsigned char* out) {
    size_t olen = 0;
    /* start code + NAL header (not RBSP, never escaped) */
    out[olen++] = 0x00; out[olen++] = 0x00; out[olen++] = 0x00; out[olen++] = 0x01;
    out[olen++] = 0x06;                         /* nal_unit_type = 6 (SEI) */

    int zr = 0;                                 /* RBSP zero-run for escaping */
    sei_emit_rbsp_byte(out, &olen, &zr, 0x05);  /* payloadType = 5 */

    /* payloadSize = 16 (UUID) + payload_len, coded as ff..xx */
    size_t psize = 16 + payload_len;
    while (psize >= 255) { sei_emit_rbsp_byte(out, &olen, &zr, 0xff); psize -= 255; }
    sei_emit_rbsp_byte(out, &olen, &zr, (unsigned char)psize);

    for (int i = 0; i < 16; i++)
        sei_emit_rbsp_byte(out, &olen, &zr, SEI_BOX_UUID[i]);
    for (size_t i = 0; i < payload_len; i++)
        sei_emit_rbsp_byte(out, &olen, &zr, payload[i]);

    sei_emit_rbsp_byte(out, &olen, &zr, 0x80);  /* rbsp_trailing_bits */
    return olen;
}

/* ---- H.265 / HEVC variant (app_m5 codec switch) ----------------------------
 * Same user_data_unregistered payload (payloadType 5, 16-byte UUID + box bytes),
 * but framed as an HEVC PREFIX_SEI_NUT (nal_unit_type = 39) with a 2-BYTE NAL
 * header instead of H.264's 1-byte header:
 *
 *   00 00 00 01            4-byte start code (not RBSP)
 *   4E 01                  NAL header: forbidden_zero=0, nal_unit_type=39
 *                          (PREFIX_SEI_NUT), nuh_layer_id=0, nuh_temporal_id_plus1=1
 *                          byte0 = (0<<7)|(39<<1)|((0>>5)&1) = 0x4E
 *                          byte1 = ((0<<5)&0xE0)|1            = 0x01
 *   05                     payloadType = 5 (user_data_unregistered)
 *   <payloadSize>          ff..xx  (= 16 + box_payload_len)
 *   <16-byte UUID>
 *   <box payload>          SeiBoxHeader + N * SeiBoxRec
 *   80                     rbsp_trailing_bits
 *
 * RBSP emulation-prevention is byte-identical to H.264 (the 00 00 03 rule is the
 * same in HEVC) and is applied to everything AFTER the 2-byte NAL header. A
 * conformant HEVC decoder ignores the unknown-UUID SEI, so the video still
 * decodes for non-aware receivers. out must be >= payload_len*2 + 32. */
static inline size_t sei_build_nal_h265(const unsigned char* payload,
                                        size_t payload_len,
                                        unsigned char* out) {
    size_t olen = 0;
    /* start code + 2-byte HEVC NAL header (not RBSP, never escaped) */
    out[olen++] = 0x00; out[olen++] = 0x00; out[olen++] = 0x00; out[olen++] = 0x01;
    out[olen++] = 0x4E;                         /* nal_unit_type=39 PREFIX_SEI_NUT */
    out[olen++] = 0x01;                         /* layer_id=0, temporal_id_plus1=1 */

    int zr = 0;                                 /* RBSP zero-run for escaping */
    sei_emit_rbsp_byte(out, &olen, &zr, 0x05);  /* payloadType = 5 */

    size_t psize = 16 + payload_len;
    while (psize >= 255) { sei_emit_rbsp_byte(out, &olen, &zr, 0xff); psize -= 255; }
    sei_emit_rbsp_byte(out, &olen, &zr, (unsigned char)psize);

    for (int i = 0; i < 16; i++)
        sei_emit_rbsp_byte(out, &olen, &zr, SEI_BOX_UUID[i]);
    for (size_t i = 0; i < payload_len; i++)
        sei_emit_rbsp_byte(out, &olen, &zr, payload[i]);

    sei_emit_rbsp_byte(out, &olen, &zr, 0x80);  /* rbsp_trailing_bits */
    return olen;
}

/* Quantize one detection (center px in 1920x1080, prob 0..1) into a SeiBoxRec. */
static inline void sei_fill_rec(SeiBoxRec* r, int cls, float prob,
                                float cx, float cy, float w, float h) {
    int q = (int)lroundf(prob * 32767.0f);
    if (q < 0) q = 0;
    if (q > 32767) q = 32767;
    r->cls = (uint8_t)(cls & 0xff);
    r->conf_q15 = (uint16_t)q;
    r->x = (int16_t)lroundf(cx);
    r->y = (int16_t)lroundf(cy);
    r->w = (int16_t)lroundf(w);
    r->h = (int16_t)lroundf(h);
}

#endif /* SEI_BOX_H */
