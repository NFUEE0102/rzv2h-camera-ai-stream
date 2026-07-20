#!/usr/bin/env python3
"""
link_proto.py -- shared discovery + control wire protocol for the RZ/V2H camera/AI stream
video-link supervisor (TX = board, RX = ground-station PC).

This is the NEW supervisor layer. It is intentionally self-contained (no GStreamer
import) so it runs identically on the board and on the PC. The heavy engines
(app_m5 on the board, stream_viewer.py on the PC) are spawned as subprocesses by the
supervisors -- this module only carries the control plane.

ROLE DIRECTION (per the requirement "the RX finds the TX"):
  * RX broadcasts a DISCOVER datagram on UDP DISCOVERY_PORT.
  * TX listens on DISCOVERY_PORT (bound to the wlx NIC) and replies with an OFFER
    datagram straight back to the RX (unicast).
  * RX then opens a TCP control connection to the TX on CONTROL_PORT.
  * TX is the TCP *server* -- it just sits there, autonomous, waiting. The RX is
    the one a human drives.

Control messages are 4-byte big-endian length-prefixed UTF-8 JSON, identical in
framing to the proven test0528/test0417 stack (send_msg/recv_msg below).
"""
import json
import socket
import struct

# ---- ports (already allocated by this system; do not move) ----
DISCOVERY_PORT = 50000      # UDP: RX broadcast DISCOVER  <->  TX unicast OFFER
CONTROL_PORT   = 50001      # TCP: control + feedback (TX is the server)
RTP_PORT       = 50010      # RTP H.264 video  (board -> PC)
BBOX_PORT      = 50012      # UDP JSON bbox sidecar (board -> PC)  [SEI is primary]
FB_PORT        = 50013      # UDP adaptive-bitrate feedback (PC -> board)

# ---- discovery datagram magic ----
DISCOVER_MAGIC = "LINK_DISCOVER_V1"   # RX -> broadcast
OFFER_MAGIC    = "LINK_OFFER_V1"      # TX -> unicast reply

# ---- control message types (JSON "type" field) ----
# RX -> TX
MSG_START   = "start"     # begin 60fps detect + stream to me
MSG_STOP    = "stop"      # stop streaming, stay alive in READY
MSG_PING    = "ping"      # liveness probe
MSG_BYE     = "bye"       # RX is disconnecting cleanly
MSG_TIME    = "time"      # Phase1 time-sync: RX is the CLOCK MASTER. Right after
                          # the control link is established the RX sends this once
                          # with its wall-clock epoch under "epoch" (float, UTC
                          # seconds). The TX supervisor runs as root, so it can set
                          # the board clock (`date -s @<epoch>`) if the board drifted
                          # (e.g. after a reboot that reset it) so ALL board logs
                          # share the RX/ground timeline. It is a no-op when already
                          # aligned (|offset| <= a threshold). Purely additive: an
                          # old TX that doesn't know this type simply ignores it.
# TX -> RX
MSG_HELLO   = "hello"     # sent right after the TCP accept: TX identity + state
MSG_STATE   = "state"     # state transition notification (READY/RUNNING/ERROR/...)
MSG_STAT    = "stat"      # periodic fps / bitrate / engine stats
MSG_ERROR   = "error"     # an engine-side problem (camera/encoder/child crash)
MSG_PONG    = "pong"      # reply to ping
MSG_LOG     = "log"       # forwarded engine stderr line (optional, throttled)
MSG_TELEM   = "telem"     # ~1 Hz compact telemetry summary (TX -> RX): the board's
                          # WiFi / camera / engine / encoder values, so the RX can
                          # log the complete both-sides picture in its own CSV. The
                          # payload is a flat {key: value} dict under "d".

# ---- TX state names (string, shared so both sides log the same words) ----
ST_WAIT_NIC     = "WAIT_NIC"
ST_DISCOVERABLE = "DISCOVERABLE"
ST_READY        = "READY"
ST_RUNNING      = "RUNNING"
ST_ERROR        = "ERROR"


# ===========================================================================
# TCP length-prefixed JSON framing (same as common.py in the reference stack)
# ===========================================================================
def send_msg(sock, msg_dict):
    """Send one 4-byte-length-prefixed JSON message. Raises on broken socket."""
    data = json.dumps(msg_dict).encode("utf-8")
    sock.sendall(struct.pack(">I", len(data)) + data)


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (socket.timeout, ConnectionResetError, OSError):
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def recv_msg(sock, timeout=None):
    """Receive one message. Returns dict, or None on timeout / closed / bad frame."""
    if timeout is not None:
        sock.settimeout(timeout)
    try:
        header = _recv_exact(sock, 4)
        if header is None:
            return None
        (length,) = struct.unpack(">I", header)
        if length > 10 * 1024 * 1024:
            return None
        payload = _recv_exact(sock, length)
        if payload is None:
            return None
        return json.loads(payload.decode("utf-8"))
    except (socket.timeout, ConnectionResetError, OSError, ValueError):
        return None
