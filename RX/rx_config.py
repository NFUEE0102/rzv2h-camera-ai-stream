#!/usr/bin/env python3
"""
rx_config.py -- RX-side (ground-station PC) supervisor configuration.
"""
import os

HOME = os.path.expanduser("~")

# ---- the NIC that reaches the board's wlx (the clean .51 OpenWrt subnet) ----
# Used to pick the broadcast address and the adaptive-feedback source. The PC's
# enp2s0 is 192.168.51.243; the board wlx is 192.168.51.180.
RX_IFACE_IP = "192.168.51.243"           # this PC's IP on the .51 subnet
BROADCAST_ADDR = "192.168.51.255"        # directed broadcast for DISCOVER
# (255.255.255.255 also tried automatically as a fallback)

# ---- the viewer engine (decode + SEI overlay + display + adaptive feedback) ----
VIEWER_PY = os.path.join(HOME, "stream_viewer.py")
DECODER   = "nvh265dec"                   # GPU HEVC decode; avdec_h265 = CPU fallback
SINK      = "autovideosink"

# ---- ports (must match the board) ----
DISCOVERY_PORT = 50000
CONTROL_PORT   = 50001
RTP_PORT       = 50010
BBOX_PORT      = 50012
FB_PORT        = 50013

# adaptive bitrate: send loss/recv reports to the board's wlx so it can lower
# the encoder bitrate live. Set to "" to disable adaptive feedback.
# NOTE: since 2026-07-02 the supervisor prefers the LIVE discovered TX IP over
# this value (a stale static IP here silently killed the AQC feedback loop when
# the AP renumbered 51.x -> 50.x). This is a manual-run fallback ONLY.
FB_HOST = "192.168.50.180"               # board wlx IP (fallback; discovery wins)

# ---- OpenWrt AP telemetry (the RX is on the AP's wired LAN, so it can SSH the
# AP directly). The RX telemetry thread logs the board's association data
# (signal, tx/rx rate, retries) under ap_* columns. Best-effort: never blocks
# the link.
# 2026-07-02: AP swapped OpenWrt One (51.1, root:root) -> BPI-R3 (50.1, root
# with NO password: sshd offers only auth "none"; AP_PASS="" makes the client
# drive the paramiko Transport with auth_none). The old 51.1 answered through
# the morning transition window, then went dark -> ap_* columns silently went
# empty; hence this update. ----
AP_HOST     = "192.168.50.1"             # BPI-R3 AP on the .50 subnet
AP_USER     = "root"
AP_PASS     = ""                          # "" = passwordless auth_none (BPI-R3)
AP_BOARD_MAC = "00:c0:ca:bb:63:9c"       # the board wlx MAC -> pick its station row
AP_IFACE    = ""                          # "" = auto-detect via `iwinfo` (phy*-ap*)

# ---- logging ----
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

# ---- timing ----
DISCOVER_INTERVAL = 1.0                   # seconds between DISCOVER broadcasts
PING_INTERVAL     = 2.0                   # liveness ping to TX
TELEM_INTERVAL    = 1.0                   # seconds between telemetry CSV rows

# ===========================================================================
# RESILIENCE SUITE (all ADDITIVE; each behaviour gated by a flag, default ON).
# A single "stream supervisor" thread reconciles actual state -> want_stream:
# it ensures the viewer is running and the board is streaming whenever the
# operator wants video and the control link is up, with shared backoff so the
# watchdogs / reconnect-resume / error-retry never crash-loop or double-fire.
# Set any flag False to fully disable that behaviour and fall back to manual 's'.
# ===========================================================================
AUTO_RESUME         = True    # after a reconnect, auto re-START if want_stream
AUTO_RETRY_ON_ERROR = True    # on board engine MSG_ERROR, auto re-START (backoff)
VIEWER_WATCHDOG     = True    # restart a dead viewer subprocess + re-START
STALL_WATCHDOG      = True    # restart a silently-stalled stream (no frames)

STALL_TIMEOUT       = 6.0     # s of genuinely-zero receive before declaring a stall
SUPERVISOR_INTERVAL = 1.0     # s between stream-supervisor reconcile passes

# shared exponential backoff + crash-loop cap for every auto (re)start above
RETRY_BACKOFF_BASE  = 1.0     # s -- first auto-retry waits this long
RETRY_BACKOFF_MAX   = 16.0    # s -- backoff is capped here
MAX_AUTO_RETRY      = 5       # consecutive failed auto-(re)starts before giving up
RETRY_RESET_AFTER   = 30.0    # s of frames flowing -> reset the failure counter

# ---- Phase 1.5: recognise the board's "camera wedged" MSG_ERROR as NON-retryable.
# When the board's cam-prep robustness declares the camera wedged (a rapid-cycle
# CRU wedge that only a board reboot clears), it reports MSG_ERROR whose detail
# contains this marker. Auto-retry MUST stop (pressing 's' or backing off cannot
# help until the board reboots), so the RX matches this substring, sets the
# quiescent/exhausted state, and prints a clear operator message. The board's
# board_tx.py emits the SAME marker -- keep the two in sync if you change it.
CAM_WEDGED_MARKER   = "camera wedged"
