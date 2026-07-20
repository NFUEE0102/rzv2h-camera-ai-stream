#!/usr/bin/env python3
"""
tx_config.py -- TX-side (RZ/V2H board) supervisor configuration.

Edit this file to point at the right NIC / engine paths. The supervisor
(board_tx.py) reads it once at startup.
"""
import os

# The engine (app_m5), camera scripts, and CSI device all belong to the 'ubuntu'
# user on the board. The supervisor runs as root (systemd, for SO_BINDTODEVICE),
# so DO NOT use os.path.expanduser("~") here -- under root that resolves to /root
# and the engine would not be found. Pin the owning user's home explicitly.
ENGINE_USER_HOME = os.environ.get("ENGINE_HOME", "/home/ubuntu")
HOME = ENGINE_USER_HOME

# ---- the wlx NIC the link runs over (board side of the clean .51 path) ----
# Discovery + control are bound to this device (SO_BINDTODEVICE) so they never
# leak onto the wired end0 management LAN. `ip link` to confirm the name.
TX_IFACE = "wlx00c0cabb639c"

# ---- engine: app_m5 (CSI 1080p -> DRP-AI YOLOX -> H.264 60fps + SEI -> RTP) ----
ENGINE_DIR  = os.path.join(HOME, "sample_yolox_cam")
ENGINE_BIN  = "./app_m5"
ENGINE_ARGS = ["2", "5"]          # app_m5 <cam_index> <?> -- proven launch args
ENGINE_CHILD = "capture_encoder_m5"       # the RTP-streaming child app_m5 exec()s

# camera 60fps prep script (manual exposure -> true 60fps); run before each START
CAM_60FPS_SH = os.path.join(HOME, "cam-60fps.sh")

# ---- Phase 1.5: cam-prep robustness (prevent rapid START/STOP wedging the CRU) ----
# The wedge mechanism (observed twice, 2026-07-02): rapid START/STOP piles multiple
# cam-60fps.sh (v4l2-ctl --set-ctrl) calls onto a rzg2l-cru that is mid retry-init ->
# a stuck D-state v4l2-ctl -> /dev/video0 kernel-wedged -> only a reboot recovers.
# These three guards make that impossible from the supervisor side (all tunable):
#   * CAM_PREP_TIMEOUT_S  -- cam-60fps.sh normally finishes in <2s (v4l2-ctl is fast);
#     a hang past this = the CRU is wedging. On timeout we back off, do NOT spawn the
#     engine, do NOT pile a second cam-prep. Was subprocess.run(timeout=30).
#   * MIN_ENGINE_RESTART_S -- hard debounce: at least this long between engine starts
#     regardless of trigger (operator rapid s/x/s OR the auto-retry). A START arriving
#     sooner waits out the remainder so the camera is never hammered.
#   * CAM_WEDGE_FAIL_THRESHOLD -- this many consecutive cam-prep timeouts (or a dmesg
#     CRU-retry/tevs-stop match) => declare camera_wedged: report MSG_ERROR "camera
#     wedged -- reboot the board" and REFUSE further START until the process restarts
#     (a reboot restarts board-tx fresh and clears the flag).
CAM_PREP_TIMEOUT_S       = float(os.environ.get("CAM_PREP_TIMEOUT", "8"))
MIN_ENGINE_RESTART_S     = float(os.environ.get("MIN_ENGINE_RESTART", "4"))
CAM_WEDGE_FAIL_THRESHOLD = int(os.environ.get("CAM_WEDGE_FAIL_THRESHOLD", "2"))

# ---- stream parameters handed to the engine via env ----
STREAM_RTP_PORT  = 50010
STREAM_BBOX_PORT = 50012
# Phase1: start at the healthy L0 = 8 Mbps ceiling. capture_encoder's AQC ladder picks the
# level nearest this on startup (=L0 8000), then adapts DOWN the 8000/5000/3000/2000
# ladder on WiFi retries / weak signal / RX loss and back UP when sustained-healthy.
# Headroom for motion, no quantization macroblocking (the prior 1km "blocks" were
# loss gray-outs, not quantization). L0 is easily retuned (here + YOLO_AQC_LEVELS).
STREAM_BITRATE   = 8000000        # 8 Mbps healthy ceiling (AQC L0)

# ---- logging ----
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

# ---- timing ----
STAT_INTERVAL      = 2.0          # seconds between STAT messages to the RX
PING_DEAD_TIMEOUT  = 8.0          # no RX traffic for this long -> drop RX, re-listen
ENGINE_SPAWN_GRACE = 4.0          # seconds to wait for engine to come up before
                                  # treating an immediate exit as a hard error

# ---- Phase1 time-sync (RX = clock master) ----
# When the RX sends its wall-clock epoch (MSG_TIME) on connect, the root TX
# supervisor sets the board clock IFF it is off by more than this many seconds.
# 2s tolerates ordinary NTP-less skew + link latency while still catching the
# gross drift a reboot (clock reset to the epoch / build date) leaves behind.
TIME_SYNC_THRESHOLD_S = float(os.environ.get("TIME_SYNC_THRESHOLD", "2.0"))
