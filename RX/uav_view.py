#!/usr/bin/env python3
"""
uav_view.py -- RZ/V2H decoupled stream viewer + ADAPTIVE-BITRATE feedback (app_m5).

Decodes the board's 1080p H.265/HEVC (RTP :50010) on the NVIDIA GPU (nvh265dec)
and overlays the detection boxes the board streams either as an in-stream HEVC
prefix-SEI NAL (perfect frame sync) or, as a fallback, as UDP JSON (:50012),
using an in-pipeline cairooverlay (no frame copies). Boxes are CENTER-based
pixels in the 1920x1080 frame, Pascal VOC 20-class.

CODEC: the board encoder is now omxh265enc (HEVC ~40-50% the bitrate of H.264 for
equal quality -> long-range / low-bandwidth WiFi). RX pipeline is rtph265depay ->
h265parse -> nvh265dec. The 4 MiB udpsrc buffer + leaky queues + jb-latency are
codec-agnostic and carried over verbatim from the H.264 build.

ROTATION: --rotate cw|ccw|none inserts a videoflip (after the decoder, before
cairooverlay) AND applies the SAME 90-deg rotation to the box coordinates in the
overlay draw, so boxes still land on their objects after the frame turns. This is
the fixed correction for a side-mounted (90-deg) camera. DEFAULT none: the present
test rig's sensor was VERIFIED to already deliver an upright 1920x1080 landscape
frame (no sensor/pipeline rotation), so no flip is applied. Set cw/ccw if the
physical mount on a given airframe is actually rotated 90 deg.

HORIZON-LOCK (--horizon imu|off, default imu): the board reads the camera-module
IMU (LSM6DSO16IS) roll and ships it per-frame in the SEI. The RX rotates the
decoded frame by -roll about center on the GPU (gltransformation rotation-z, live)
with a crop-to-fill zoom so no black corners appear, and rotates the box coords by
the same -roll, so the horizon stays LEVEL while the airframe banks. Composes with
--rotate (90-deg mount) + the dynamic roll. --horizon-sign 1|-1 flips the rotation
direction (the IMU-axis->screen cw/ccw mapping can't be resolved headless; flip
after the first physical tilt test). --no-gl drops the GPU frame rotation (boxes
still rotate) if the GL elements are missing on the host.

app_m5 ADAPTIVE BITRATE: a 1 Hz sampler reads the rtpjitterbuffer "stats"
(num-pushed / num-lost) for loss%% and a udpsrc src-pad byte-probe for received
kbps, then sends a 16-byte FbMsg to the board (UDP --fb-port, default 50013).
The board's uav_enc runs the AIMD controller and live-sets omxh265enc bitrate.

NOTE: the board stream has no periodic IDR, so START THIS BEFORE the board app --
a late joiner stays black until the next (never) IDR.

  DISPLAY=:0 python3 uav_view.py --fb-host 192.168.51.180 \
      [--sink autovideosink|fakesink|fpsdisplaysink] [--decoder nvh265dec|avdec_h265] \
      [--rotate cw|ccw|none]
"""
import gi, json, socket, threading, argparse, colorsys, struct, time, signal, os
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

VOC = ["aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
       "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person",
       "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
COLORS = [colorsys.hsv_to_rgb(i / len(VOC), 0.9, 1.0) for i in range(len(VOC))]

state = {"w": 1920, "h": 1080, "det": [], "seq": -1, "n": 0, "src": "json",
         "box_ts": 0.0}                # box_ts = RX 收到最後一框的單調鐘(秒), D3 P0-1a
vdim = {"w": 1920, "h": 1080}          # actual decoded frame size (from caps)
lock = threading.Lock()

# ---- app_m5 SEI box extraction (see src/sei_box.h for the byte layout) ----
# HEVC prefix-SEI NAL: nal_unit_type 39 (2-byte NAL header), payloadType 5
# (user_data_unregistered), 16-byte UUID, then the box payload.
#
# HEADER (app_m5 "hz", ver==2, all little-endian, 10 bytes):
#   uint32 frame_seq, uint16 n, uint8 ver, uint8 flags, int16 roll_cdeg
# followed by n * {uint8 cls, uint16 conf_q15, int16 x,y,w,h}.
# Coords are CENTER px in 1920x1080. conf = conf_q15/32767. roll = roll_cdeg/100
# DEGREES (signed) when (flags & SEI_FLAG_ROLL_VALID). ver==1 = legacy 6-byte
# header (no roll); we still parse it (roll stays None).
# (The emulation-prevention + payloadType/size parsing are identical to H.264;
# only the NAL header is 2 bytes and the type lives in bits (byte0>>1)&0x3f.)
SEI_UUID = b"UAVBOXSEI01_DRPI"          # 16 bytes, must match sei_box.h
HEVC_SEI_PREFIX = 39                    # nal_unit_type for PREFIX_SEI_NUT
SEI_FLAG_ROLL_VALID = 0x01              # flags bit0 -> roll_cdeg is a live reading
sei_state = {"have": False, "ts": 0.0}  # have=曾解到SEI; ts=最近一次SEI單調鐘(秒), D3 P0-1a

# ---- D3 P0-1a/P0-1c staleness / ReconGate 門檻(皆為校準旋鈕, D3 §④)----------
SEI_FRESH_S = 0.5     # SEI 新鮮窗;逾此(未更新)回退 sidecar 框 (D3 P0-1a〔A3〕)
BOX_FADE_S  = 2.0     # sidecar 最後已知框 age 逾此褪色 (D3 P0-1a sidecar staleness)
GATE_HOLD_S = 0.5     # 未標齡 hold 上限;video_age 逾此上 HELD 徽章 (D3 P0-1c S8)
GATE_RED_S  = 3.0     # HELD 逾此紅警 "SIGNAL LOST — HELD Xs" (D3 P0-1c S8)
GATE_LOSS_DEGRADED = 2.0  # jb loss% 逾此 → DEGRADED (校準旋鈕)
recon = {"gate": None}    # ReconGate 實例;僅 --freeze on 時建立, None=閘門關(零行為變更)

# ---- app_m5 "hz" horizon-lock (dynamic IMU roll) ----
# The board ships the per-frame roll (degrees, about the camera optical axis) in
# the SEI. We rotate the decoded frame by -roll on the GPU (gltransformation
# rotation-z, live-settable) with a slight zoom so the rotated corners don't show
# black, and rotate the box coords by the SAME -roll about image center so boxes
# stay on objects. mode "imu" = dynamic; "off" = ignore the roll. sign flips the
# rotation direction (in case the IMU axis convention is mirrored vs the screen).
horizon = {"mode": "off", "roll": 0.0, "have": False,
           "elem": None,        # the gltransformation element (or None)
           "sign": 1.0,         # +1 / -1 ; one-flag flip after the first tilt test
           "zoom": 1.0}         # crop-to-fill scale applied on the GPU


def _rbsp_unescape(b):
    """Remove emulation_prevention_three_byte (00 00 03 -> 00 00).
    Identical rule in H.264 and HEVC."""
    out = bytearray()
    i, n = 0, len(b)
    while i < n:
        if i + 2 < n and b[i] == 0 and b[i + 1] == 0 and b[i + 2] == 3:
            out.append(0); out.append(0); i += 3   # drop the 0x03
        else:
            out.append(b[i]); i += 1
    return bytes(out)


def _iter_nals(au):
    """Yield raw NAL unit payloads (after the start code, incl. NAL header byte)
    from an Annex-B byte-stream access unit."""
    n = len(au)
    i = 0
    starts = []
    while i + 2 < n:
        if au[i] == 0 and au[i + 1] == 0 and au[i + 2] == 1:
            starts.append(i + 3); i += 3
        else:
            i += 1
    for k, s in enumerate(starts):
        e = (starts[k + 1] - 3) if k + 1 < len(starts) else n
        # trim a trailing 00 that belongs to the next start code's 00 00 01
        while e > s and au[e - 1] == 0:
            e -= 1
        yield au[s:e]


def _parse_sei_boxes(au):
    """Scan an AU for our UUID HEVC prefix-SEI; return (frame_seq, det_list) or None."""
    for nal in _iter_nals(au):
        # HEVC NAL header is 2 bytes; nal_unit_type = (byte0 >> 1) & 0x3f.
        if len(nal) < 2 or ((nal[0] >> 1) & 0x3f) != HEVC_SEI_PREFIX:
            continue
        rbsp = _rbsp_unescape(nal[2:])             # strip the 2-byte NAL header
        p = 0
        L = len(rbsp)
        while p < L:
            # payloadType (ff-extended)
            pt = 0
            while p < L and rbsp[p] == 0xff:
                pt += 255; p += 1
            if p >= L:
                break
            pt += rbsp[p]; p += 1
            # payloadSize (ff-extended)
            ps = 0
            while p < L and rbsp[p] == 0xff:
                ps += 255; p += 1
            if p >= L:
                break
            ps += rbsp[p]; p += 1
            body = rbsp[p:p + ps]; p += ps
            if pt == 5 and len(body) >= 16 and body[:16] == SEI_UUID:
                payload = body[16:]
                if len(payload) < 6:
                    return None
                frame_seq, nbox = struct.unpack_from("<IH", payload, 0)
                # version-aware header: ver==2 carries ver/flags/roll_cdeg (10 B),
                # ver==1 (or a too-short header) is the legacy 6-byte form.
                roll = None
                off = 6
                if len(payload) >= 10:
                    ver, flags, roll_cdeg = struct.unpack_from("<BBh", payload, 6)
                    if ver >= 2:
                        off = 10
                        if flags & SEI_FLAG_ROLL_VALID:
                            roll = roll_cdeg / 100.0
                det = []
                for _ in range(nbox):
                    if off + 11 > len(payload):
                        break
                    cls, cq, x, y, w, h = struct.unpack_from("<BHhhhh", payload, off)
                    off += 11
                    det.append({"c": cls, "p": cq / 32767.0,
                                "x": float(x), "y": float(y),
                                "w": float(w), "h": float(h)})
                return frame_seq, det, roll
    return None


def on_sei_au(pad, info):
    """Pad probe on h265parse src: extract HEVC prefix-SEI boxes, update overlay state."""
    buf = info.get_buffer()
    if buf is None:
        return Gst.PadProbeReturn.OK
    ok, mi = buf.map(Gst.MapFlags.READ)
    if not ok:
        return Gst.PadProbeReturn.OK
    try:
        res = _parse_sei_boxes(bytes(mi.data))
    finally:
        buf.unmap(mi)
    if res is not None:
        frame_seq, det, roll = res
        now = time.monotonic()
        with lock:
            state["det"] = det; state["seq"] = frame_seq
            state["w"] = 1920; state["h"] = 1080
            state["n"] += 1; state["src"] = "sei"
            state["box_ts"] = now              # D3 P0-1a: 框新鮮度(單一 staleness 用)
            if roll is not None:
                horizon["roll"] = roll; horizon["have"] = True
        sei_state["have"] = True
        sei_state["ts"] = now                  # D3 P0-1a: SEI 新鮮窗判據(bbox_rx 回退用)
        if roll is not None:
            apply_horizon(roll)        # live-set the GL frame rotation
    return Gst.PadProbeReturn.OK

# ---- app_m4 adaptive-bitrate feedback state ----
FB_MAGIC = 0x55415646   # 'UAVF' little-endian
FB_VER = 1
fb = {
    "rx_bytes": 0,          # cumulative bytes seen on udpsrc src pad
    "jb": None,             # rtpjitterbuffer element
    "sock": None,           # UDP socket to the board
    "dst": None,            # (host, port)
    "prev_bytes": 0, "prev_lost": 0, "prev_pushed": 0, "prev_t": None,
    "seq": 0, "cap_kbps": 0, "prev_frames": 0, "prev_disp": 0,
}


def bbox_rx(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    while True:
        try:
            data, _ = s.recvfrom(65536)
            m = json.loads(data.decode("utf-8", "replace"))
            # D3 P0-1a〔紅隊 A3〕: SEI 新鮮(≤SEI_FRESH_S)才優先(完美幀同步);
            # SEI 逾期(>0.5s 未更新,如板端只在關鍵幀注入 SEI 而視訊斷流)則回退
            # sidecar 框,避免整段無框。原 bug: 一旦見過 SEI 就永久 continue、把所有
            # sidecar 都丟掉(即使 SEI 早已停更)——這正是現場「框不見了」的根因。
            if sei_state["have"] and (time.monotonic() - sei_state["ts"]) <= SEI_FRESH_S:
                continue
            roll = m.get("roll", None)
            with lock:
                state["w"] = m.get("w", 1920); state["h"] = m.get("h", 1080)
                state["det"] = m.get("det", []); state["seq"] = m.get("seq", -1)
                state["n"] += 1; state["src"] = "json"
                state["box_ts"] = time.monotonic()   # D3 P0-1a: 框新鮮度
                if roll is not None:
                    horizon["roll"] = roll; horizon["have"] = True
            if roll is not None:
                apply_horizon(roll)     # JSON-fallback horizon-lock
        except Exception:
            continue


def on_udp_buffer(pad, info):
    buf = info.get_buffer()
    if buf:
        fb["rx_bytes"] += buf.get_size()
    return Gst.PadProbeReturn.OK


# ---- Phase1 REAL display-fps counter ----------------------------------------
# The old dec_fps used state["n"], which only increments on an AU that carried an
# SEI box. Because the board injects the SEI only on keyframes + detection changes
# (a CPU optimisation), state["n"] climbs at the ~20 Hz DETECTION/SEI rate, NOT the
# real rendered-frame rate -- so the operator saw "~20 fps" and thought the link
# was slow. This probe counts EVERY decoded frame (one buffer per frame on the
# decoder src pad) so we can report the TRUE ~60 fps display rate separately.
disp = {"frames": 0, "last_ts": 0.0,    # frames=總解碼幀數; last_ts=最近解幀單調鐘, D3 P0-1c
        "first_ts": 0.0}                # 首幀解出單調鐘; ever_decoded ≡ frames>0, D3 P0-1c cold-start


def on_decoded_frame(pad, info):
    """Buffer probe on the decoder SRC pad: one buffer == one rendered frame.
    Counts the real display-frame rate independent of the SEI/detection rate.
    D3 P0-1c: 兼作 freeze 探針的 decode-health 取樣——記錄最近解幀時戳, ReconGate
    用以算 video_age(視訊凍結偵測)。〔ponytail〕本函式只取樣時戳; 完整「快取最後
    良性 buffer + LOST 態放行快取幀」的重新入鏈需 appsrc/imagefreeze 重打時鐘, 依
    D3 §③ P0-1c 閘門(先量 nvh265dec 對污染 AU 之輸出型態)再決定是否建元件——未量
    前不建, 避免 pad probe 持有/重注 pipeline buffer 的風險。"""
    if info.get_buffer() is not None:
        if disp["frames"] == 0:
            disp["first_ts"] = time.monotonic()   # D3 P0-1c cold-start: 首幀時戳(等待時長回報)
        disp["frames"] += 1
        disp["last_ts"] = time.monotonic()
    return Gst.PadProbeReturn.OK


def _struct_int(st, field):
    """Read an integer field from a GstStructure regardless of uint64/int type."""
    try:
        ok, v = st.get_uint64(field)
        if ok:
            return int(v)
    except Exception:
        pass
    try:
        ok, v = st.get_int(field)
        if ok:
            return int(v)
    except Exception:
        pass
    try:
        ok, v = st.get_uint(field)
        if ok:
            return int(v)
    except Exception:
        pass
    return None


def jb_stats():
    """Return (num_pushed, num_lost) from the rtpjitterbuffer 'stats' struct."""
    jb = fb["jb"]
    if jb is None:
        return None
    try:
        st = jb.get_property("stats")
        if st is None:
            return None
        pushed = _struct_int(st, "num-pushed")
        lost = _struct_int(st, "num-lost")
        if pushed is None and lost is None:
            return None
        return (pushed or 0, lost or 0)
    except Exception:
        return None


def jb_jitter_ms():
    """Best-effort current jitter (ms) from the rtpjitterbuffer 'stats' struct.
    The 'jitter' field is in clock units (clock-rate 90000 -> /90 = ms). Returns
    None if the field is absent on this GStreamer build."""
    jb = fb["jb"]
    if jb is None:
        return None
    try:
        st = jb.get_property("stats")
        if st is None:
            return None
        j = _struct_int(st, "jitter")
        if j is None:
            return None
        return round(j / 90.0, 2)        # 90 kHz RTP clock -> milliseconds
    except Exception:
        return None


def fb_tick():
    """1 Hz: compute loss% + recv kbps, send FbMsg to the board."""
    now = time.monotonic()
    if fb["prev_t"] is None:
        fb["prev_t"] = now
        fb["prev_bytes"] = fb["rx_bytes"]
        s = jb_stats()
        if s:
            fb["prev_pushed"], fb["prev_lost"] = s
        return True

    dt = now - fb["prev_t"]
    if dt <= 0:
        return True
    d_bytes = fb["rx_bytes"] - fb["prev_bytes"]
    recv_kbps = int(8.0 * d_bytes / 1000.0 / dt)   # includes RTP/UDP/IP overhead

    loss_milli = 0
    s = jb_stats()
    if s:
        pushed, lost = s
        d_pushed = max(0, pushed - fb["prev_pushed"])
        d_lost = max(0, lost - fb["prev_lost"])
        denom = d_pushed + d_lost
        if denom > 0:
            loss_milli = int(100000.0 * d_lost / denom)   # loss% * 1000
        fb["prev_pushed"], fb["prev_lost"] = pushed, lost
    else:
        # coarse fallback: no bytes for >2 ticks ~ link stalled
        if d_bytes == 0:
            loss_milli = 100000   # 100%

    loss_milli = max(0, min(loss_milli, 65535))

    # Phase1: TWO distinct rates.
    #  disp_fps  = the REAL rendered-frame rate: disp["frames"] counts every buffer
    #              on the decoder src pad (one per decoded frame) -> the true ~60.
    #  detect_fps= the SEI/detection rate: state["n"] increments only on an AU that
    #              carried an SEI box (~20 Hz). Kept as a separate, honest number so
    #              the operator can still see how often boxes update.
    with lock:
        frames_now = state["n"]
    detect_fps = round((frames_now - fb["prev_frames"]) / dt, 1) if dt > 0 else 0.0
    fb["prev_frames"] = frames_now
    disp_now = disp["frames"]
    disp_fps = round((disp_now - fb["prev_disp"]) / dt, 1) if dt > 0 else 0.0
    fb["prev_disp"] = disp_now
    jitter_ms = jb_jitter_ms()

    fb["prev_t"] = now
    fb["prev_bytes"] = fb["rx_bytes"]
    fb["seq"] += 1

    if fb["sock"] is not None and fb["dst"] is not None:
        pkt = struct.pack("<IHHII", FB_MAGIC, FB_VER, loss_milli,
                          max(0, recv_kbps), fb["seq"])
        try:
            fb["sock"].sendto(pkt, fb["dst"])
        except Exception:
            pass
    # the [fb] line is parsed by the RX supervisor (uav_rx) for its telemetry CSV.
    # Phase1: emit BOTH the true rendered-frame rate (disp_fps ~60) and the
    # SEI/detection rate (detect_fps ~20) as distinct fields. The legacy dec_fps=
    # is kept (== disp_fps now, the REAL rate) so any older parser still reads the
    # true fps. Keep the "[fb]" prefix + field order stable for the supervisor RE.
    print("[fb] seq=%d loss=%.2f%% recv=%d kbps disp_fps=%.1f detect_fps=%.1f "
          "dec_fps=%.1f jitter=%s ms -> %s" %
          (fb["seq"], loss_milli / 1000.0, recv_kbps, disp_fps, detect_fps,
           disp_fps, ("%.2f" % jitter_ms) if jitter_ms is not None else "na",
           fb["dst"]),
          flush=True)
    return True


# ---- D3 P0-1c ReconGate 單一真相源(移植 arch-rx, 收攏 D2 S7/S8/S9)-------------
class ReconGate:
    """融合輸入(優先序見 D3 §⑤: jb do-lost/stats 最硬 > decode-health > SEI 連續性
    > FbMsg 殘容量最軟)→ 輸出 HEALTHY/DEGRADED/LOST, 並發布唯一權威
    displayed-frame-age = max(video_age, bbox_age)〔紅隊 A3〕。判定採「最嚴重者勝、
    最硬證據(jb)可再升級」: 凍幀(decode-health)不被較軟的 jb loss% 降級。純狀態機、
    無 GStreamer 相依 → 可離線 _gate_demo() 自檢。閾值全為校準旋鈕。
    NOTE(D3 P0-1c 範圍): 第 4 輸入「FbMsg 殘容量」為最軟佐證, 本版 update() 未接線
    ——三硬輸入(jb loss / decode-health / SEI)已足判主狀態; 殘容量佐證留待後續。"""
    def __init__(self):
        self.state = "HEALTHY"
        self.video_age = 0.0
        self.bbox_age = 0.0
        self.frame_age = 0.0      # 對外唯一權威 = max(video, bbox)

    def update(self, now, video_ts, box_ts, loss_pct, sei_have, sei_ts):
        self.video_age = (now - video_ts) if video_ts > 0.0 else 0.0
        self.bbox_age = (now - box_ts) if box_ts > 0.0 else 0.0
        self.frame_age = max(self.video_age, self.bbox_age)
        st = "HEALTHY"
        # (3) SEI 連續性: SEI 長斷 → 至少 DEGRADED
        if sei_have and sei_ts > 0.0 and (now - sei_ts) > GATE_RED_S:
            st = "DEGRADED"
        # (2) decode-health: 視訊幀齡是凍幀最直接證據
        if video_ts > 0.0 and self.video_age > GATE_HOLD_S:
            st = "LOST"
        # (2b) 冷啟動: 從未解出任何幀(video_ts==0)視同 LOST——REQ_IDR hint 迴圈
        #      不得依賴「曾有幀」狀態, 冷啟動(無封包/無可解 IDR)即持續催 IDR。
        if video_ts <= 0.0:                      # D3 P0-1c cold-start
            st = "LOST"
        # (1) jb do-lost/stats: 最硬證據, 只升級不降級(不遮蓋凍幀 LOST)
        if loss_pct is not None:
            if loss_pct >= 99.0:
                st = "LOST"
            elif loss_pct >= GATE_LOSS_DEGRADED and st != "LOST":
                st = "DEGRADED"
        self.state = st
        return st


# ---- D3 P0-1c REQ_IDR 上行 hint(預設關; env 開)-------------------------------
# ReconGate 進 LOST 時, 若 env UAV_RX_REQIDR=1, 向 UAV_RX_FB_HINT(host:port)送
# 1-byte UDP 提示。地面 fb sender(bpir3_fb.py)收到後於下一封 FbMsg 標 REQ_IDR、板端
# 補發 IDR。UAV_RX_FB_HINT 空(預設)=關。best-effort, LOST 持續時每秒重送一次。
_reqidr = {"sock": None, "dst": None, "last_send": 0.0}


def send_reqidr_hint():
    if _reqidr["dst"] is None:
        return
    try:
        _reqidr["sock"].sendto(b"\x01", _reqidr["dst"])
    except Exception:
        pass


def gate_tick():
    """5 Hz: 驅動 ReconGate 狀態機 + LOST 時發 REQ_IDR hint。僅 --freeze on 時排程
    (recon['gate'] is None → 立即停排, 零行為變更)。"""
    g = recon["gate"]
    if g is None:
        return False                          # 未啟用 → 不再排程
    now = time.monotonic()
    with lock:
        box_ts = state["box_ts"]
    # jb do-lost: 本地 prev 計數(與 fb_tick 互不干擾), 粗略 loss% 即可
    loss_pct = None
    s = jb_stats()
    if s:
        pushed, lost = s
        dp = pushed - recon["p_pushed"]; dl = lost - recon["p_lost"]
        if dp >= 0 and dl >= 0 and (dp + dl) > 0:
            loss_pct = 100.0 * dl / (dp + dl)
        recon["p_pushed"], recon["p_lost"] = pushed, lost
    st = g.update(now, disp["last_ts"], box_ts, loss_pct,
                  sei_state["have"], sei_state["ts"])
    if st == "LOST" and _reqidr["dst"] is not None and (now - _reqidr["last_send"]) >= 1.0:
        send_reqidr_hint(); _reqidr["last_send"] = now
    return True                               # 續排程


# ---- D3 P0-1c cold-start: 「等待影像」狀態列(無條件; 純 log, 非行為開關)--------
# 冷啟動若從未解出首幀(無封包、或有封包但無可解 IDR), autovideosink 無 preroll →
# 無視窗、操作員只見黑。本 1s timer 由 GLib main loop 驅動、不依賴 buffer 流(同
# fb_tick 模式: 斷流時照跳), 每秒印一行穩定格式 [wait] 供人眼與 uav_rx.py status
# 解析; 首幀解出後印一行 first-frame 延遲並自停。no-packets/packets-no-idr 以
# udpsrc byte probe(fb["rx_bytes"], 無條件掛載)的每秒增量判定。
wait = {"t0": 0.0, "prev_bytes": 0, "prev_t": 0.0}   # D3 P0-1c cold-start


def wait_tick():
    """1 Hz, 首幀前有效: 印 [wait] 等待狀態; ever_decoded(disp['frames']>0)後自停。"""
    now = time.monotonic()
    if disp["frames"] > 0:                       # ever_decoded → 印首幀延遲, 停排程
        print("[wait] first frame after %.1fs" % (disp["first_ts"] - wait["t0"]),
              flush=True)
        return False
    d_bytes = fb["rx_bytes"] - wait["prev_bytes"]
    dt = now - wait["prev_t"]
    wait["prev_bytes"] = fb["rx_bytes"]
    wait["prev_t"] = now
    rx_kbps = int(8.0 * d_bytes / 1000.0 / dt) if dt > 0 else 0
    st = "no-packets" if d_bytes == 0 else "packets-no-idr"
    print("[wait] WAITING FOR SIGNAL %ds state=%s rx_kbps=%d"
          % (int(round(now - wait["t0"])), st, rx_kbps), flush=True)
    return True


def _gate_demo():
    """離線自檢(免 GStreamer): python3 -c 'import uav_view; uav_view._gate_demo()'。"""
    g = ReconGate()
    t = 100.0
    assert g.update(t, t, t, 0.0, True, t) == "HEALTHY"
    assert g.update(t, t - 1.0, t, 0.0, True, t) == "LOST"       # 視訊幀齡 1s>0.5 → 凍
    assert g.update(t, t, t, 100.0, True, t) == "LOST"           # jb 全丟 → LOST
    assert g.update(t, t, t, 5.0, True, t) == "DEGRADED"         # jb 5%>2% → DEGRADED
    assert g.update(t, t - 1.0, t, 5.0, True, t) == "LOST"       # 凍幀不被 jb 降級
    assert g.update(t, t, t, 0.0, True, t - 5.0) == "DEGRADED"   # SEI 斷 5s>3.0 → DEGRADED
    assert g.update(t, 0.0, 0.0, None, False, 0.0) == "LOST"     # 冷啟動未曾解幀 → LOST, D3 P0-1c cold-start
    g.update(t, t - 0.2, t - 0.1, 0.0, True, t)
    assert abs(g.frame_age - 0.2) < 1e-9                         # 權威=max(video,bbox)
    print("[uav_view] _gate_demo OK")


def on_caps(overlay, caps):
    st = caps.get_structure(0)
    ok_w, w = st.get_int("width"); ok_h, h = st.get_int("height")
    if ok_w and ok_h:
        vdim["w"], vdim["h"] = w, h


def _crop_fill_zoom(roll_deg, aspect):
    """Scale factor so a W:H (aspect=W/H) frame rotated by roll_deg about its
    center still fully covers the viewport (no black corners). Derived from how
    far the rotated rectangle's edges pull in."""
    import math
    c = abs(math.cos(math.radians(roll_deg)))
    s = abs(math.sin(math.radians(roll_deg)))
    sw = c + s / aspect          # cover the width
    sh = c + s * aspect          # cover the height
    return max(sw, sh)


def apply_horizon(roll_deg):
    """Live-set the GL frame rotation to counter the airframe roll: rotate the
    decoded frame by -roll about center (GPU), with a crop-to-fill zoom so the
    corners stay covered. Called from the SEI parse callback (~20 Hz)."""
    el = horizon["elem"]
    if el is None or horizon["mode"] != "imu":
        return
    rot = horizon["sign"] * (-roll_deg)            # counter-rotate by -roll
    aspect = (vdim["w"] / float(vdim["h"])) if vdim["h"] else (16.0 / 9.0)
    zoom = _crop_fill_zoom(roll_deg, aspect)
    horizon["zoom"] = zoom
    try:
        # gltransformation: rotation-z in DEGREES about the screen normal (aspect-
        # corrected internally); scale-x/y > 1 zooms to crop-to-fill. Both are
        # live-settable GObject props, so we just push the new values per update.
        el.set_property("rotation-z", float(rot))
        el.set_property("scale-x", float(zoom))
        el.set_property("scale-y", float(zoom))
    except Exception as e:
        print("[uav_view] gltransformation set failed:", e, flush=True)


# ---- 90-deg rotation of detection boxes to match the videoflip on the frame ----
# Detection coords are CENTER x/y + w/h in the UNROTATED rw x rh space (1920x1080).
# videoflip rotates the *frame* 90 deg; we rotate each box the SAME way so it still
# lands on its object, then scale into the actual (rotated) frame pixels (vdim).
#   none : new=(cx,cy,w,h), scaled by vdim/(rw,rh)
#   cw   : 90 deg clockwise        -> new_cx = rh-cy, new_cy = cx,    w/h swap
#   ccw  : 90 deg counterclockwise -> new_cx = cy,    new_cy = rw-cx, w/h swap
# After a 90 deg turn the rotated detection space is rh wide x rw tall, so the
# frame-space scale uses (vdim_w/rh, vdim_h/rw).
ROTATE = {"mode": "none"}                # set from --rotate in main()


def _rot_box(d, rw, rh):
    """Return (cx, cy, w, h, angle_deg) in FRAME pixels for one detection dict.
    Two rotations compose:
      1) the FIXED 90-deg side-mount correction (--rotate cw/ccw/none), mapping
         the unrotated rw x rh detection space into the (fixed-)rotated frame;
      2) the DYNAMIC horizon roll: the GL stage rotated the frame by sign*(-roll)
         about its center with a crop-to-fill zoom, so each box center is rotated
         the SAME way about the frame center (and scaled by the zoom) to stay on
         its object. angle_deg is returned so the drawn rectangle can be tilted to
         match the horizon."""
    import math
    cx = d["x"]; cy = d["y"]; bw = d["w"]; bh = d["h"]
    m = ROTATE["mode"]
    if m == "cw":
        ncx, ncy, nw, nh = (rh - cy), cx, bh, bw
        sx = vdim["w"] / float(rh); sy = vdim["h"] / float(rw)
    elif m == "ccw":
        ncx, ncy, nw, nh = cy, (rw - cx), bh, bw
        sx = vdim["w"] / float(rh); sy = vdim["h"] / float(rw)
    else:  # none
        ncx, ncy, nw, nh = cx, cy, bw, bh
        sx = vdim["w"] / float(rw); sy = vdim["h"] / float(rh)
    fx, fy, fw, fh = ncx * sx, ncy * sy, nw * sx, nh * sy

    # dynamic horizon roll about the FRAME center (matches the GL gltransformation)
    angle = 0.0
    if horizon["mode"] == "imu" and horizon["have"]:
        roll = horizon["roll"]
        angle = horizon["sign"] * (-roll)        # same sense as the frame rotation
        z = horizon.get("zoom", 1.0)
        a = math.radians(angle)
        ca, sa = math.cos(a), math.sin(a)
        ccx, ccy = vdim["w"] / 2.0, vdim["h"] / 2.0
        dx, dy = (fx - ccx) * z, (fy - ccy) * z
        fx = ccx + dx * ca - dy * sa
        fy = ccy + dx * sa + dy * ca
        fw *= z; fh *= z
    return fx, fy, fw, fh, angle


def on_draw(overlay, cr, timestamp, duration):
    now = time.monotonic()
    with lock:
        det = state["det"]; rw = state["w"] or 1920; rh = state["h"] or 1080
        seq = state["seq"]; n = state["n"]; src = state["src"]
        box_ts = state["box_ts"]
    import math
    # D3 P0-1a: 框 staleness。box_age = RX 收到最後一框至今(單調鐘, 免跨鐘校時)。
    # 健康時 age~0 → 不標齡不褪色, 畫面與現行位元級等同; 逾 2s 才褪色誠實標陳舊。
    box_age = (now - box_ts) if box_ts > 0.0 else 0.0
    fade = 0.4 if box_age > BOX_FADE_S else 1.0
    g = recon["gate"]
    # D3 P0-1c〔紅隊 A3〕: 視訊凍結時框釘凍幀——比凍幀更新的框不得單獨渲染在舊幀上
    # (否則新鮮框替陳舊幀背書為 live)。--freeze off 時 g is None → pin_boxes 恆 False。
    frozen = (g is not None) and (g.video_age > GATE_HOLD_S)
    pin_boxes = frozen and (box_ts > disp["last_ts"])
    cr.select_font_face("Sans"); cr.set_font_size(20); cr.set_line_width(3.0)
    for d in (() if pin_boxes else det):
        try:
            x, y, w, h, angle = _rot_box(d, rw, rh)
            c = int(d["c"]); p = float(d["p"])
        except Exception:
            continue
        r, g_, b = COLORS[c % len(COLORS)]
        label = "%s %d%%" % (VOC[c] if 0 <= c < len(VOC) else str(c), int(p * 100))
        cr.save()
        if abs(angle) > 0.05:
            # tilt the box (and its label) by the horizon angle about the box
            # center so the rectangle stays aligned to the rotated object.
            cr.translate(x, y); cr.rotate(math.radians(angle)); cr.translate(-x, -y)
        x0, y0 = x - w / 2.0, y - h / 2.0
        cr.set_source_rgba(r, g_, b, fade); cr.rectangle(x0, y0, w, h); cr.stroke()
        ext = cr.text_extents(label)
        cr.set_source_rgba(0, 0, 0, 0.6 * fade); cr.rectangle(x0, y0 - 24, ext.width + 8, 22); cr.fill()
        cr.set_source_rgba(r, g_, b, fade); cr.move_to(x0 + 4, y0 - 6); cr.show_text(label)
        cr.restore()
    # D3 P0-1c: 凍幀顯示狀態機——未標齡 hold≤0.5s → HELD 徽章隨齡由黃升紅(消
    # 0.5–3.0s 靜默窗) → >3.0s 紅警 SIGNAL LOST。僅 --freeze on(g 存在)時繪; 絕不外推。
    if g is not None and g.video_age > GATE_HOLD_S:
        age = g.video_age
        if age > GATE_RED_S:
            bcol = (1.0, 0.0, 0.0); btxt = "SIGNAL LOST — HELD %.1fs" % age
        else:
            f = min(1.0, (age - GATE_HOLD_S) / (GATE_RED_S - GATE_HOLD_S))
            bcol = (1.0, 1.0 - f, 0.0); btxt = "HELD %.1fs" % age
        cr.set_font_size(28); bext = cr.text_extents(btxt)
        bx = (vdim["w"] - bext.width) / 2.0
        cr.set_source_rgba(0, 0, 0, 0.6); cr.rectangle(bx - 10, 40, bext.width + 20, 40); cr.fill()
        cr.set_source_rgb(*bcol); cr.move_to(bx, 68); cr.show_text(btxt)
        cr.set_font_size(20)
    # D3 P0-1a/P0-1c: 狀態列尾綴 box 齡(逾新鮮窗才標)與 gate 狀態(僅 --freeze on)。
    extra = ""
    if box_age > SEI_FRESH_S:
        extra += "  box+%.1fs" % box_age
    if g is not None:
        extra += "  gate=%s age=%.1fs" % (g.state, g.frame_age)
    cr.set_source_rgba(0, 0, 0, 0.5); cr.rectangle(0, 0, 680 + (240 if extra else 0), 30); cr.fill()
    cr.set_source_rgb(0, 1, 0); cr.move_to(8, 21)
    hz = ("%+.1f deg" % horizon["roll"]) if (horizon["mode"] == "imu" and horizon["have"]) \
         else (horizon["mode"])
    cr.show_text("RZ/V2H H265 1080p  boxes=%d  seq=%d  upd=%d  src=%s  rot=%s  horizon=%s%s" %
                 (len(det), seq, n, src, ROTATE["mode"], hz, extra))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtp-port", type=int, default=50010)
    ap.add_argument("--bbox-port", type=int, default=50012)
    ap.add_argument("--decoder", default="nvh265dec",
                    help="HEVC decoder (nvh265dec GPU, avdec_h265 CPU fallback)")
    ap.add_argument("--sink", default="autovideosink")
    ap.add_argument("--rotate", default="none", choices=["cw", "ccw", "none"],
                    help="fixed 90-deg frame+box rotation for a side-mounted camera "
                         "(cw / ccw rotate both the frame AND the boxes to match). "
                         "DEFAULT none: this test rig's sensor already delivers an "
                         "upright 1920x1080 landscape frame (verified), so no flip is "
                         "needed. Set cw/ccw if your physical mount is rotated 90 deg.")
    ap.add_argument("--horizon", default="imu", choices=["imu", "off"],
                    help="DYNAMIC horizon-lock from the camera IMU roll carried in "
                         "the SEI (default imu). The decoded frame is rotated by "
                         "-roll on the GPU (gltransformation) with a crop-to-fill "
                         "zoom, and the boxes are rotated the same way, so the "
                         "horizon stays level as the airframe banks. 'off' ignores "
                         "the roll. Independent of (and composes with) --rotate.")
    ap.add_argument("--horizon-sign", type=int, default=1, choices=[1, -1],
                    help="flip the horizon rotation direction (1 or -1). The IMU "
                         "axis convention's mapping to on-screen cw/ccw can't be "
                         "resolved headless; if the first physical tilt test rolls "
                         "the picture the WRONG way, pass --horizon-sign -1.")
    ap.add_argument("--no-gl", action="store_true",
                    help="disable the GL frame-rotation stage (boxes still rotate "
                         "in the overlay, but the video frame won't be GPU-rotated). "
                         "Use if the GL elements are unavailable on this host.")
    ap.add_argument("--drop-on-latency", default="false", choices=["true", "false"],
                    help="rtpjitterbuffer drop-on-latency: true=遲到封包丟棄(低延遲但抖動→"
                         "整段破圖);false(預設,同 view.sh)=遲到仍交解碼器(只短暫花屏). "
                         "wfb 無重傳,丟不如糊 → 預設 false 減少破圖.")
    ap.add_argument("--jb-latency", type=int, default=60,
                    help="rtpjitterbuffer latency ms (low=less e2e delay, more loss "
                         "sensitivity). 60 is a good wired default; raise for lossy WiFi.")
    ap.add_argument("--fb-host", default=None, help="board wlx IP for adaptive feedback")
    ap.add_argument("--fb-port", type=int, default=50013)
    ap.add_argument("--no-sei", action="store_true",
                    help="disable SEI box extraction (use UDP-JSON sidecar only)")
    ap.add_argument("--freeze", default="off", choices=["off", "on"],
                    help="D3 P0-1c: 凍幀+ReconGate 顯示狀態機(預設 off=零行為變更, "
                         "與現行位元級等同)。on: 依 jb-loss/解碼健康/SEI 連續性判 "
                         "HEALTHY/DEGRADED/LOST, 凍結時顯示 HELD/SIGNAL-LOST 徽章"
                         "(隨齡升色), 框釘凍幀不獨立渲染, 絕不做運動外推。REQ_IDR 上行"
                         "另需 env UAV_RX_REQIDR=1 + UAV_RX_FB_HINT=host:port。")
    ap.add_argument("--record", dest="record", action="store_true", default=True,
                    help="record the received H.265 to a fixed-name .mkv (default ON). "
                         "A tee after h265parse muxes the ALREADY-encoded stream to "
                         "disk -- no re-encode, no extra decode -- so the live display "
                         "is untouched. The fixed filename means each new START "
                         "overwrites it, keeping only the most recent run.")
    ap.add_argument("--no-record", dest="record", action="store_false",
                    help="disable the .mkv recording branch.")
    ap.add_argument("--record-file", default=None,
                    help="path for the recording (default ~/uav_release/RX/last_run.mkv)")
    a = ap.parse_args()
    ROTATE["mode"] = a.rotate
    horizon["mode"] = a.horizon
    horizon["sign"] = float(a.horizon_sign)
    Gst.init(None)
    caps = ("application/x-rtp,media=(string)video,clock-rate=(int)90000,"
            "encoding-name=(string)H265,payload=(int)96")
    # videoflip: clockwise / counterclockwise puts the side-mounted camera upright.
    # method=identity (none) is a passthrough. Inserted AFTER the decoder, BEFORE
    # cairooverlay, so the overlay draws onto the already-upright frame (and the
    # box coords are rotated the same way in _rot_box).
    flip = {"cw": "videoflip method=clockwise ! ",
            "ccw": "videoflip method=counterclockwise ! ",
            "none": ""}[a.rotate]
    # --- low-latency RX (2026-06-27) ---
    # latency=60 drop-on-latency=true: small jitter window; frames later than the
    #   window are DROPPED, not queued (queuing is what made e2e latency grow >10s).
    # leaky=downstream queues after the depay and before the sink: whenever the
    #   CPU-heavy BGRA cairooverlay path falls behind 60fps the OLD frames are
    #   dropped so we always render the freshest one (live view, not a backlog).
    # sink sync=false already renders ASAP (no clock buffering).
    jb_latency = a.jb_latency
    # --- app_m5 "hz" GPU horizon rotation -----------------------------------
    # gltransformation rotates the DECODED frame about the screen Z (normal) by a
    # live rotation-z (degrees), aspect-corrected, with scale-x/y for a crop-to-
    # fill zoom so the rotated corners don't show black. It sits AFTER the decoder
    # and BEFORE the BGRA/cairooverlay, so cairo draws boxes onto the already-
    # rotated frame (and _rot_box rotates the box coords the same way). The GL up/
    # download bridges system<->GL memory. Disabled by --horizon off or --no-gl.
    use_gl = (a.horizon == "imu") and (not a.no_gl)
    gl_stage = ("glupload ! gltransformation name=gltf ! "
                "gldownload ! videoconvert ! ") if use_gl else ""
    # --- LAST-RUN recording (additive, no re-encode) -------------------------
    # A tee right after h265parse feeds a second branch: queue ! matroskamux !
    # filesink. It muxes the ALREADY-encoded H.265 to disk -- no decode, no
    # re-encode, so the live display branch is untouched. The filename is FIXED,
    # so each new viewer START overwrites it (only the most recent run is kept).
    # The record branch uses its own h265parse config-interval=-1 to re-insert
    # VPS/SPS/PPS so the .mkv is self-contained and seekable/playable.
    import os as _os
    rec_path = a.record_file or _os.path.join(
        _os.path.expanduser("~"), "uav_release", "RX", "last_run.mkv")
    if a.record:
        try:
            _os.makedirs(_os.path.dirname(rec_path), exist_ok=True)
        except Exception:
            pass
    # buffer-size: enlarge the kernel UDP receive buffer (SO_RCVBUF). The default
    # is small (~200KB) and overflows on bursty 1080p60 IDR frames (one IDR is
    # ~70+ MTU-sized packets back-to-back), which showed up as 3-7% RTP loss ->
    # macroblocking even on a wired link (RcvbufErrors climbing in /proc/net/snmp).
    # 4 MiB absorbs the bursts.
    def build_desc(with_gl):
        gl = ("glupload ! gltransformation name=gltf ! gldownload ! videoconvert ! "
              if with_gl else "")
        # record branch: tee off the parsed H.265 -> mux to a fixed-name .mkv. The
        # record queue is leaky=downstream so a slow disk can NEVER stall the live
        # display branch (dropping recorded frames is acceptable; a frozen viewer
        # is not). Empty string when --no-record so the pipeline is byte-identical
        # to the proven one.
        rec = ""
        tee = ""
        if a.record:
            # gst-launch tee syntax: `tee name=rtee ! <branch1> rtee. ! <branch2>`.
            # The `!` right after the tee links branch1 (the record branch); the
            # `rtee. !` then re-references the tee for branch2 (the live display).
            tee = "tee name=rtee ! "
            rec = (f'queue leaky=downstream max-size-buffers=120 '
                   f'max-size-time=0 max-size-bytes=0 ! '
                   f'h265parse config-interval=-1 ! matroskamux ! '
                   f'filesink location="{rec_path}" sync=false async=false '
                   f'rtee. ! ')
        return (f'udpsrc name=usrc port={a.rtp_port} buffer-size=4194304 caps="{caps}" ! '
                f'rtpjitterbuffer name=jb latency={jb_latency} drop-on-latency={a.drop_on_latency} ! '
                f'rtph265depay ! h265parse name=hp ! '
                f'{tee}{rec}'
                f'queue leaky=downstream max-size-buffers=16 max-size-time=0 max-size-bytes=0 ! '
                f'{a.decoder} name=dec ! '
                f'queue leaky=downstream max-size-buffers=16 max-size-time=0 max-size-bytes=0 ! '
                f'{gl}videoconvert ! video/x-raw,format=BGRA ! '
                f'{flip}cairooverlay name=ov ! '
                f'videoconvert ! '
                f'queue leaky=downstream max-size-buffers=16 max-size-time=0 max-size-bytes=0 ! '
                f'{a.sink} sync=false')
    desc = build_desc(use_gl)
    print("[uav_view] pipeline:", desc, flush=True)
    try:
        pipe = Gst.parse_launch(desc)
    except Exception as e:
        # GL elements missing? Fall back to the no-GL pipeline (boxes still
        # horizon-rotate in the overlay; the video frame just won't be GPU-rotated).
        if use_gl:
            print("[uav_view] GL pipeline failed (%s) -- falling back to --no-gl" % e, flush=True)
            use_gl = False
            desc = build_desc(False)
            print("[uav_view] pipeline:", desc, flush=True)
            pipe = Gst.parse_launch(desc)
        else:
            raise

    # If the sink is fpsdisplaysink, print fps measurements to stdout.
    fdsink = pipe.get_by_name("fpsdisplaysink0")
    if fdsink is not None:
        try:
            fdsink.set_property("signal-fps-measurements", True)
            fdsink.connect("fps-measurements",
                           lambda s, fps, drop, avg: print(
                               "[fps] current=%.2f avg=%.2f dropped=%d" % (fps, avg, drop),
                               flush=True))
        except Exception as e:
            print("[uav_view] fpsdisplaysink signal not available:", e, flush=True)

    ov = pipe.get_by_name("ov")
    ov.connect("draw", on_draw)
    ov.connect("caps-changed", on_caps)

    # ---- app_m5 "hz": grab the GL transform element (live rotation-z target) ----
    if use_gl:
        gltf = pipe.get_by_name("gltf")
        if gltf is not None:
            horizon["elem"] = gltf
            # seed an identity transform (roll updates arrive via apply_horizon)
            try:
                gltf.set_property("rotation-z", 0.0)
                gltf.set_property("scale-x", 1.0)
                gltf.set_property("scale-y", 1.0)
            except Exception as e:
                print("[uav_view] gltransformation seed failed:", e, flush=True)
            print("[uav_view] horizon-lock GL rotate ON (sign=%+d)" % int(horizon["sign"]),
                  flush=True)
        else:
            print("[uav_view] WARN: gltransformation 'gltf' not found -- GL rotate off", flush=True)
    elif a.horizon == "imu":
        print("[uav_view] horizon-lock: GL rotate disabled (--no-gl); boxes still "
              "rotate, frame won't", flush=True)

    # ---- app_m5 SEI box extraction: probe the h265parse SRC pad (Annex-B AUs
    # with VPS/SPS/PPS/SEI/slices), parse our UUID HEVC prefix-SEI, drive the
    # overlay. Works alongside the JSON path; SEI wins once seen (see bbox_rx). ----
    if not a.no_sei:
        hp = pipe.get_by_name("hp")
        if hp is not None:
            hp_src = hp.get_static_pad("src")
            if hp_src is not None:
                hp_src.add_probe(Gst.PadProbeType.BUFFER, on_sei_au)
                print("[uav_view] SEI box extraction ON (h265parse src probe)", flush=True)

    # ---- Phase1 REAL display-fps: count every decoded frame on the decoder src
    # pad (one buffer == one rendered frame), independent of the SEI/detection
    # rate. This is what fb_tick reports as disp_fps (~60), fixing the misleading
    # ~20 that the SEI-counter dec_fps used to show. ----
    dec = pipe.get_by_name("dec")
    if dec is not None:
        dec_src = dec.get_static_pad("src")
        if dec_src is not None:
            dec_src.add_probe(Gst.PadProbeType.BUFFER, on_decoded_frame)
            print("[uav_view] real display-fps counter ON (decoder src probe)",
                  flush=True)
        else:
            print("[uav_view] WARN: decoder src pad missing -- disp_fps unavailable",
                  flush=True)
    else:
        print("[uav_view] WARN: decoder element 'dec' not found -- disp_fps unavailable",
              flush=True)

    # ---- adaptive-bitrate feedback wiring ----
    fb["jb"] = pipe.get_by_name("jb")
    usrc = pipe.get_by_name("usrc")
    if usrc is not None:
        srcpad = usrc.get_static_pad("src")
        if srcpad is not None:
            srcpad.add_probe(Gst.PadProbeType.BUFFER, on_udp_buffer)
    if a.fb_host:
        fb["sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fb["dst"] = (a.fb_host, a.fb_port)
        print("[uav_view] adaptive feedback -> %s:%d (1 Hz)" % (a.fb_host, a.fb_port), flush=True)
        GLib.timeout_add_seconds(1, fb_tick)

    # ---- D3 P0-1c freeze + ReconGate(--freeze on 才啟用; off=零行為變更)----
    if a.freeze == "on":
        recon["gate"] = ReconGate()
        recon["p_pushed"] = 0; recon["p_lost"] = 0
        # REQ_IDR 上行 hint: 額外需 env UAV_RX_REQIDR=1 且 UAV_RX_FB_HINT=host:port
        if os.environ.get("UAV_RX_REQIDR") == "1":
            hint = os.environ.get("UAV_RX_FB_HINT", "").strip()
            if hint and ":" in hint:
                h, _, p = hint.rpartition(":")
                try:
                    _reqidr["dst"] = (h, int(p))
                    _reqidr["sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    print("[uav_view] REQ_IDR hint -> %s:%s (on ReconGate LOST)" % (h, p), flush=True)
                except Exception as e:
                    print("[uav_view] REQ_IDR hint disabled (bad UAV_RX_FB_HINT):", e, flush=True)
        GLib.timeout_add(200, gate_tick)   # 5 Hz 狀態機驅動
        print("[uav_view] freeze/ReconGate ON (hold<=%.1fs, red>%.1fs)"
              % (GATE_HOLD_S, GATE_RED_S), flush=True)

    threading.Thread(target=bbox_rx, args=(a.bbox_port,), daemon=True).start()

    loop = GLib.MainLoop()
    bus = pipe.get_bus(); bus.add_signal_watch()

    def on_msg(_b, m):
        if m.type == Gst.MessageType.ERROR:
            err, dbg = m.parse_error(); print("[uav_view] ERROR", err, dbg, flush=True); loop.quit()
        elif m.type == Gst.MessageType.EOS:
            print("[uav_view] EOS", flush=True); loop.quit()
    bus.connect("message", on_msg)

    if a.record:
        print("[uav_view] RECORDING last-run H.265 -> %s (overwrites each START)"
              % rec_path, flush=True)

    # Clean shutdown for the recording: on SIGTERM/SIGINT (the RX supervisor's
    # stop_viewer() sends SIGTERM) inject an EOS so matroskamux finalizes the .mkv
    # header/index -> a VALID, seekable, playable file. Without this the file is
    # left truncated/unfinalized. We arm a hard-kill fallback so a stuck EOS can
    # never hang the viewer past ~3 s. When not recording this is a plain quit.
    shutdown = {"done": False}

    def _graceful_quit():
        if shutdown["done"]:
            return
        shutdown["done"] = True
        if a.record:
            print("[uav_view] shutdown: sending EOS to finalize the recording...",
                  flush=True)
            try:
                pipe.send_event(Gst.Event.new_eos())
            except Exception:
                loop.quit()
            # fallback: if EOS doesn't drain in time, quit anyway (file flushed
            # best-effort) so stop_viewer's terminate->kill stays bounded.
            GLib.timeout_add(2500, lambda: (loop.quit(), False)[1])
        else:
            loop.quit()

    def _sig(_signum, _frame):
        # marshal into the GLib loop thread (signal handlers run async)
        GLib.idle_add(lambda: (_graceful_quit(), False)[1])
    try:
        signal.signal(signal.SIGTERM, _sig)
        signal.signal(signal.SIGINT, _sig)
    except Exception:
        pass

    pipe.set_state(Gst.State.PLAYING)
    # 「等待影像」狀態列: 無條件排程(純 log, 非行為開關), 首幀後自停。D3 P0-1c cold-start
    wait["t0"] = wait["prev_t"] = time.monotonic()
    GLib.timeout_add_seconds(1, wait_tick)
    print("[uav_view] PLAYING (rtp:%d bbox:%d dec:%s sink:%s rotate:%s horizon:%s gl:%s rec:%s)" %
          (a.rtp_port, a.bbox_port, a.decoder, a.sink, a.rotate, a.horizon,
           "on" if use_gl else "off", "on" if a.record else "off"), flush=True)
    try:
        loop.run()
    except KeyboardInterrupt:
        _graceful_quit()
    pipe.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
