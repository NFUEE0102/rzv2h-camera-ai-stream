#!/usr/bin/env python3
"""
ground_rx.py -- ground-station RX supervisor + operator console for the RZ/V2H
camera+AI video/detection link.

A human runs THIS. It:
  1. broadcasts a DISCOVER on UDP 50000 until a TX answers (OFFER), then
  2. opens a TCP control connection to the TX (port 50001) and shows READY,
  3. lets the operator press keys to START / STOP the board's stream,
  4. on START launches the local viewer (stream_viewer.py: GPU decode + SEI box
     overlay + display + adaptive-bitrate feedback to the board),
  5. on STOP tears the viewer down but stays connected (TX stays alive too),
  6. displays + logs any TX-side error / state / stat messages,
  7. logs every event on this side with timestamps.

OPERATOR KEYS (type the letter + Enter):
    s   START   -- tell the TX to begin 60fps detect+stream, open the viewer
    x   STOP    -- tell the TX to stop streaming, close the viewer (link stays up)
    p   PING     -- liveness probe to the TX
    i   INFO     -- print current link state
    q   QUIT     -- stop, disconnect, exit the RX (TX stays autonomous)

Run:  DISPLAY=:0 python3 ground_rx.py
"""
import sys
import os
import re
import json
import socket
import threading
import subprocess
import time
import csv
from datetime import datetime

import rx_config as cfg
import link_proto as P

try:
    import paramiko                       # for the direct OpenWrt AP SSH pull
except Exception:
    paramiko = None


def now_iso():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def enable_keepalive(sock, idle=3, intvl=2, cnt=3):
    """Turn on TCP keepalive so a hard drop (no FIN, e.g. the remote board moving out of
    WiFi range) is detected in ~idle+intvl*cnt seconds instead of the long TCP
    retransmit timeout. Every option is best-effort (absent on some platforms)."""
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except OSError:
        return
    for opt, val in (("TCP_KEEPIDLE", idle), ("TCP_KEEPINTVL", intvl),
                     ("TCP_KEEPCNT", cnt)):
        o = getattr(socket, opt, None)
        if o is not None:
            try:
                sock.setsockopt(socket.IPPROTO_TCP, o, val)
            except OSError:
                pass


class EventLog:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fh = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.fh)
        self.w.writerow(["ts", "event", "detail"])
        self.fh.flush()
        self.lock = threading.Lock()

    def log(self, event, detail="", echo=True):
        if echo:
            print("%s  [%s] %s" % (now_iso(), event, detail), flush=True)
        with self.lock:
            self.w.writerow([now_iso(), event, str(detail)])
            self.fh.flush()

    def close(self):
        with self.lock:
            try:
                self.fh.flush(); self.fh.close()
            except Exception:
                pass


# ===========================================================================
# telemetry: a SEPARATE clean per-second CSV (distinct from the event log). It
# folds together (a) the local receive quality parsed from the viewer's [fb]
# line, (b) the board's forwarded summary (MSG_TELEM over the control link),
# and (c) the OpenWrt AP association data SSH-pulled from 192.168.51.1.
# ===========================================================================
RX_TELEM_COLUMNS = [
    "ts", "recv_kbps", "loss_pct", "decode_fps", "detect_fps", "jitter_ms",
    "tx_fps_sent", "tx_ae_exposure", "tx_ae_gain", "tx_imu_roll",
    "tx_wifi_signal_dbm", "tx_wifi_tx_mbps", "tx_enc_target_kbps", "tx_aqc_level",
    "tx_cpu_pct", "tx_temp_c", "tx_time_offset_s", "tx_time_applied_offset_s",
    "ap_station_signal_dbm", "ap_signal_avg_dbm", "ap_last_ack_dbm",
    "ap_noise_dbm", "ap_snr_db",
    "ap_tx_mbps", "ap_tx_mcs", "ap_tx_nss", "ap_rx_mbps",
    "ap_tx_retries", "ap_tx_failed", "ap_tx_retry_ps",
    "ap_airtime_tx_us", "ap_airtime_rx_us",
    "ap_channel_busy_pct", "ap_expected_mbps", "ap_n_stations",
]


class TelemetryLog:
    """Thread-safe clean telemetry CSV with a fixed column schema."""
    def __init__(self, path, columns):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.columns = columns
        self.fh = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.fh, fieldnames=columns, extrasaction="ignore")
        self.w.writeheader()
        self.fh.flush()
        self.lock = threading.Lock()

    def write(self, row):
        with self.lock:
            self.w.writerow(row)
            self.fh.flush()

    def close(self):
        with self.lock:
            try:
                self.fh.flush(); self.fh.close()
            except Exception:
                pass


_NUM = r"(-?\d+(?:\.\d+)?)"


class ApPoller:
    """Best-effort OpenWrt AP association puller (direct SSH on the .51 subnet).
    Holds a persistent paramiko connection; reconnects on demand. EVERY public
    call is wrapped so an unreachable/rebooting AP just yields empty ap_* fields
    and is logged once -- it can never crash or stall the RX telemetry loop."""
    def __init__(self, evt):
        self.evt = evt
        self.cli = None
        self.iface = cfg.AP_IFACE or None
        self.warned = False
        self.ok_once = False
        self.prev = {}            # for per-second deltas (retries, channel busy)

    def _connect(self):
        if paramiko is None:
            return False
        if self.cli is not None:
            return True
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            if cfg.AP_PASS:
                c.connect(cfg.AP_HOST, username=cfg.AP_USER,
                          password=cfg.AP_PASS,
                          timeout=4, banner_timeout=4, auth_timeout=4,
                          look_for_keys=False, allow_agent=False)
            else:
                # Passwordless AP (BPI-R3): sshd offers only auth "none", which
                # SSHClient.connect() cannot negotiate -- drive the Transport
                # directly and graft it onto the client so exec_command works.
                t = paramiko.Transport((cfg.AP_HOST, 22))
                t.start_client(timeout=4)
                t.auth_none(cfg.AP_USER)
                c._transport = t
            self.cli = c
            return True
        except Exception as e:
            if not self.warned:
                self.evt.log("ap_unreachable", "%s: %s" % (cfg.AP_HOST, e), echo=False)
                self.warned = True
            self.cli = None
            return False

    def _run(self, cmd):
        if not self._connect():
            return None
        try:
            _i, o, _e = self.cli.exec_command(cmd, timeout=6)
            return o.read().decode(errors="replace")
        except Exception as e:
            self.evt.log("ap_cmd_fail", str(e), echo=False)
            try:
                self.cli.close()
            except Exception:
                pass
            self.cli = None
            return None

    def _detect_iface(self):
        if self.iface:
            return self.iface
        out = self._run("iwinfo 2>/dev/null")
        if not out:
            return None
        # first column of an AP line, e.g. "phy1-ap0  ESSID: ..."
        m = re.search(r"^(\S+)\s+ESSID", out, re.MULTILINE)
        if m:
            self.iface = m.group(1)
            self.evt.log("ap_iface", self.iface, echo=False)
        return self.iface

    def sample(self):
        """Return {ap_*} for the board's station (best-effort). {} if unreachable."""
        iface = self._detect_iface()
        if not iface:
            return {}
        out = self._run("iw dev %s station dump 2>/dev/null" % iface)
        if not out:
            # fall back to iwinfo assoclist (covers builds without `iw`)
            return self._sample_iwinfo(iface)
        # split into per-station blocks; pick the board's MAC (or the strongest)
        blocks = re.split(r"(?m)^Station\s+", out)
        want = cfg.AP_BOARD_MAC.lower()
        n_stations = sum(1 for b in blocks if b.strip())
        chosen = None
        for b in blocks:
            if not b.strip():
                continue
            mac = b.split()[0].lower() if b.split() else ""
            if mac == want:
                chosen = b
                break
            if chosen is None:
                chosen = b
        res = {"ap_n_stations": n_stations}
        if chosen:
            def _g(pat, cast=float):
                mm = re.search(pat, chosen)
                return cast(mm.group(1)) if mm else None
            v = _g(r"signal:\s*" + _NUM)
            if v is not None: res["ap_station_signal_dbm"] = v
            v = _g(r"signal avg:\s*" + _NUM)
            if v is not None: res["ap_signal_avg_dbm"] = v
            v = _g(r"last ack signal:\s*" + _NUM)
            if v is not None: res["ap_last_ack_dbm"] = v
            v = _g(r"tx bitrate:\s*" + _NUM + r"\s*MBit/s")
            if v is not None: res["ap_tx_mbps"] = v
            v = _g(r"rx bitrate:\s*" + _NUM + r"\s*MBit/s")
            if v is not None: res["ap_rx_mbps"] = v
            v = _g(r"tx bitrate:[^\n]*?MCS\s*(\d+)", int)
            if v is not None: res["ap_tx_mcs"] = v
            v = _g(r"tx bitrate:[^\n]*?NSS\s*(\d+)", int)
            if v is not None: res["ap_tx_nss"] = v
            v = _g(r"tx retries:\s*(\d+)", int)
            if v is not None:
                res["ap_tx_retries"] = v
                pv = self.prev.get("retr")
                if pv is not None:
                    res["ap_tx_retry_ps"] = max(0, v - pv)
                self.prev["retr"] = v
            v = _g(r"tx failed:\s*(\d+)", int)
            if v is not None: res["ap_tx_failed"] = v
            v = _g(r"tx duration:\s*(\d+)", int)
            if v is not None: res["ap_airtime_tx_us"] = v
            v = _g(r"rx duration:\s*(\d+)", int)
            if v is not None: res["ap_airtime_rx_us"] = v
            v = _g(r"expected throughput:\s*" + _NUM + r"\s*Mbps")
            if v is not None: res["ap_expected_mbps"] = v
        # channel survey: noise floor + busy% (+ derived SNR vs the board's signal)
        sv = self._run("iw dev %s survey dump 2>/dev/null" % iface)
        if sv:
            inuse = next((b for b in re.split(r"Survey data from", sv)
                          if "[in use]" in b), "")
            mn = re.search(r"noise:\s*" + _NUM, inuse)
            if mn:
                noise = float(mn.group(1))
                res["ap_noise_dbm"] = noise
                if "ap_station_signal_dbm" in res:
                    res["ap_snr_db"] = round(res["ap_station_signal_dbm"] - noise, 1)
            ma = re.search(r"channel active time:\s*(\d+)", inuse)
            mb = re.search(r"channel busy time:\s*(\d+)", inuse)
            if ma and mb and int(ma.group(1)) > 0:
                res["ap_channel_busy_pct"] = round(
                    100.0 * int(mb.group(1)) / int(ma.group(1)), 1)
        if not self.ok_once:
            self.evt.log("ap_ok", "iface=%s stations=%d" % (iface, n_stations),
                         echo=False)
            self.ok_once = True
        return res

    def _sample_iwinfo(self, iface):
        out = self._run("iwinfo %s assoclist 2>/dev/null" % iface)
        if not out:
            return {}
        want = cfg.AP_BOARD_MAC.upper()
        blocks = re.split(r"(?m)^(?=[0-9A-F]{2}:)", out)
        n_stations = sum(1 for b in blocks if re.match(r"[0-9A-F]{2}:", b.strip()))
        chosen = None
        for b in blocks:
            if b.strip().upper().startswith(want):
                chosen = b
                break
            if chosen is None and re.match(r"[0-9A-F]{2}:", b.strip()):
                chosen = b
        res = {"ap_n_stations": n_stations}
        if chosen:
            m = re.search(_NUM + r"\s*dBm", chosen)
            if m:
                res["ap_station_signal_dbm"] = float(m.group(1))
            m = re.search(r"TX:\s*" + _NUM + r"\s*MBit/s", chosen)
            if m:
                res["ap_tx_mbps"] = float(m.group(1))
            m = re.search(r"RX:\s*" + _NUM + r"\s*MBit/s", chosen)
            if m:
                res["ap_rx_mbps"] = float(m.group(1))
            m = re.search(r"expected throughput:\s*" + _NUM, chosen)
            if m:
                res["ap_expected_mbps"] = float(m.group(1))
        return res

    def close(self):
        try:
            if self.cli:
                self.cli.close()
        except Exception:
            pass


class RxSupervisor:
    def __init__(self):
        self.running = True
        self.conn = None                 # TCP control socket to the TX
        self.tx_ip = None
        self.tx_host = None
        self.tx_state = "?"
        self.viewer = None               # subprocess.Popen of stream_viewer.py
        self.viewer_lock = threading.Lock()
        # ---- resilience: single source of truth + stream-supervisor state ----
        # want_stream is what the OPERATOR wants ("video should be on"). The
        # stream-supervisor thread reconciles reality -> want_stream: ensure the
        # viewer runs and the board streams, with the shared backoff/cap below.
        self.want_stream = False
        self.sup_lock = threading.RLock()   # serialises every (re)START / restart
                                            # action so the watchdogs, the
                                            # reconnect-resume and the error-retry
                                            # can never double-fire concurrently
        self.fail_count = 0              # consecutive failed auto-(re)starts
        self.exhausted = False          # cap hit -> stop auto-retry, stay quiescent
        self.pending = False            # an auto-(re)START is on probation (grace)
        self.error_retry = False        # next auto-start is an engine-error retry
        self.next_retry_at = 0.0        # monotonic time the next auto-start is due
        self.last_frame_t = 0.0         # monotonic: last time recv_kbps>0 was seen
        self.last_start_t = 0.0         # monotonic: last time we issued a START
        self.stream_up_since = 0.0      # monotonic: frames first started flowing
        self.want_stream_changed = 0.0  # monotonic: last want_stream toggle (logging)
        os.makedirs(cfg.LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.evt = EventLog(os.path.join(cfg.LOG_DIR, "rx_events_%s.csv" % ts))
        # ---- clean per-second telemetry CSV (separate from the event log) ----
        self.telem = TelemetryLog(
            os.path.join(cfg.LOG_DIR, "rx_telemetry_%s.csv" % ts), RX_TELEM_COLUMNS)
        self.telem_lock = threading.Lock()
        self.rx_quality = {}      # recv_kbps/loss_pct/decode_fps/jitter_ms from viewer
        self.tx_summary = {}      # latest board MSG_TELEM payload (d)
        self.ap = ApPoller(self.evt)

    # =======================================================================
    # discovery: broadcast DISCOVER, wait for an OFFER, return (ip, port, host)
    # =======================================================================
    def discover_tx(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            s.bind((cfg.RX_IFACE_IP, 0))
        except OSError:
            pass
        s.settimeout(cfg.DISCOVER_INTERVAL)
        payload = (P.DISCOVER_MAGIC + " rx=" + cfg.RX_IFACE_IP).encode("utf-8")
        targets = [(cfg.BROADCAST_ADDR, P.DISCOVERY_PORT),
                   ("255.255.255.255", P.DISCOVERY_PORT)]
        self.evt.log("discover_start",
                     "broadcasting on %s:%d" % (cfg.BROADCAST_ADDR, P.DISCOVERY_PORT))
        while self.running:
            for t in targets:
                try:
                    s.sendto(payload, t)
                except OSError:
                    pass
            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                m = json.loads(data.decode("utf-8", "replace"))
            except Exception:
                continue
            if m.get("magic") == P.OFFER_MAGIC:
                tx_ip = m.get("tx_ip") or addr[0]
                port = int(m.get("control_port", P.CONTROL_PORT))
                host = m.get("hostname", "?")
                self.evt.log("discover_offer",
                             "TX %s (%s) state=%s" % (tx_ip, host, m.get("state")))
                s.close()
                return tx_ip, port, host
        s.close()
        return None, None, None

    # =======================================================================
    # control connection + reader thread
    # =======================================================================
    def connect_tx(self, ip, port):
        c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        c.settimeout(5.0)
        try:
            c.connect((ip, port))
        except OSError as e:
            self.evt.log("connect_fail", "%s:%d %s" % (ip, port, e))
            return False
        enable_keepalive(c)              # detect a hard TX drop (no FIN) in seconds
        self.conn = c
        self.tx_ip = ip
        self.evt.log("connected", "control link to TX %s:%d" % (ip, port))
        # ---- Phase1 time-sync: the RX is the CLOCK MASTER. Push our wall-clock
        # epoch to the TX right after connecting so the board (root supervisor)
        # can snap its clock onto the ground timeline if it drifted (e.g. a reboot
        # reset it). No-op on the board when already aligned. Best-effort: a send
        # failure here never blocks the link (the reader thread handles teardown).
        try:
            P.send_msg(c, {"type": P.MSG_TIME, "epoch": time.time(),
                           "ts": now_iso()})
            self.evt.log("time_master", "sent RX epoch %.3f to TX" % time.time())
        except Exception as e:
            self.evt.log("time_send_fail", str(e), echo=False)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._pinger, daemon=True).start()
        return True

    def _reader(self):
        """Receive TX -> RX messages: HELLO/STATE/STAT/ERROR/PONG/LOG."""
        c = self.conn
        while self.running and c is not None:
            msg = P.recv_msg(c, timeout=2.0)
            if msg is None:
                # could be idle; check the socket is still alive via a 0-byte peek
                if self._sock_dead(c):
                    self.evt.log("tx_link_lost", "control socket closed")
                    self._on_link_lost()
                    return
                continue
            mt = msg.get("type")
            if mt == P.MSG_HELLO:
                self.tx_host = msg.get("hostname")
                self.tx_state = msg.get("state", "?")
                self.evt.log("tx_hello",
                             "host=%s ip=%s state=%s"
                             % (self.tx_host, msg.get("tx_ip"), self.tx_state))
            elif mt == P.MSG_STATE:
                self.tx_state = msg.get("state", self.tx_state)
                self.evt.log("tx_state",
                             "%s %s" % (self.tx_state, msg.get("detail", "")))
            elif mt == P.MSG_STAT:
                # quiet-log stats (no console spam unless engine line present)
                detail = msg.get("engine") or ("state=%s streaming=%s"
                                               % (msg.get("state"), msg.get("streaming")))
                self.evt.log("tx_stat", detail, echo=bool(msg.get("engine")))
            elif mt == P.MSG_ERROR:
                self.tx_state = P.ST_ERROR
                detail = msg.get("detail", "")
                self.evt.log("TX_ERROR", detail)
                # ---- Phase 1.5: "camera wedged" is NON-retryable ----
                # The board declared the CRU wedged (a rapid-cycle wedge that only a
                # board REBOOT clears). Auto-retry, backoff, and pressing 's' all
                # CANNOT help -- the board refuses every START until it reboots. So
                # treat this as terminal: stop the auto-retry (quiescent/exhausted),
                # tear the dead viewer down, and tell the operator plainly.
                marker = getattr(cfg, "CAM_WEDGED_MARKER", "camera wedged")
                if marker and marker.lower() in detail.lower():
                    with self.sup_lock:
                        # want_stream stays as the operator's intent, but exhausted
                        # halts every auto-(re)start path (supervisor early-returns on
                        # exhausted) and clears any in-flight pending/backoff so the
                        # stream-supervisor goes quiescent instead of crash-looping.
                        self.exhausted = True
                        self.pending = False
                        self.error_retry = False
                        self.next_retry_at = 0.0
                    self.stop_viewer("camera wedged -- board reboot required")
                    self.evt.log("camera_wedged",
                                 "board reports camera WEDGED -> auto-retry STOPPED")
                    print("\n*** board camera WEDGED -- reboot the board "
                          "(press 's' won't help until reboot) ***\n", flush=True)
                    continue
                # the board engine (app_m5) crashed and the TX fell back to READY.
                # Tear the now-dead viewer down and arm a backoff so the stream-
                # supervisor's "viewer died" branch issues a paced auto re-START
                # (NOT instantly -- an engine that re-crashes must not busy-loop).
                # The retry's success/failure is judged by the supervisor's PENDING
                # evaluator, which feeds the shared crash-loop cap.
                if getattr(cfg, "AUTO_RETRY_ON_ERROR", True) and self.want_stream \
                        and not self.exhausted:
                    # Set the intent + backoff FIRST (under the lock) so a
                    # concurrent supervisor pass sees error_retry and attributes
                    # the recovery correctly; only then tear the dead viewer down
                    # OUTSIDE the lock (stop_viewer blocks up to 4s on the
                    # recording finalize -- holding sup_lock across it would make
                    # an operator STOP wait that long). The supervisor's "viewer
                    # died" branch then issues exactly one paced auto re-START.
                    do_stop = False
                    with self.sup_lock:
                        if not self.pending:   # don't disturb an in-flight attempt
                            self.error_retry = True
                            self.next_retry_at = time.monotonic() + \
                                self._backoff_for(self.fail_count + 1)
                        do_stop = True
                    if do_stop:
                        self.stop_viewer("tx engine error -> auto-retry")
                    print("\n*** TX ERROR: %s ***\n(auto-retry scheduled)\n"
                          % detail, flush=True)
                else:
                    print("\n*** TX ERROR: %s ***\n(press 's' to retry)\n"
                          % detail, flush=True)
            elif mt == P.MSG_LOG:
                self.evt.log("tx_engine_log", msg.get("line", ""), echo=False)
            elif mt == P.MSG_TELEM:
                # the board's 1 Hz WiFi/camera/engine summary -> stash for the RX
                # telemetry CSV so it carries the complete both-sides picture.
                d = msg.get("d", {})
                if isinstance(d, dict):
                    with self.telem_lock:
                        self.tx_summary = d
            elif mt == P.MSG_PONG:
                self.evt.log("pong", msg.get("state", ""), echo=False)

    def _sock_dead(self, c):
        try:
            c.setblocking(False)
            d = c.recv(1, socket.MSG_PEEK)
            c.setblocking(True)
            return d == b""
        except BlockingIOError:
            c.setblocking(True)
            return False
        except OSError:
            return True

    def _pinger(self):
        while self.running and self.conn is not None:
            time.sleep(cfg.PING_INTERVAL)
            self._send({"type": P.MSG_PING, "ts": now_iso()})

    def _send(self, msg):
        c = self.conn
        if c is None:
            return False
        try:
            P.send_msg(c, msg)
            return True
        except Exception as e:
            self.evt.log("send_fail", str(e))
            self._on_link_lost()
            return False

    def _on_link_lost(self):
        c = self.conn
        self.conn = None
        try:
            if c:
                c.close()
        except Exception:
            pass
        self.stop_viewer("link lost")

    # =======================================================================
    # viewer engine (stream_viewer.py) control
    # =======================================================================
    def start_viewer(self):
        with self.viewer_lock:
            if self.viewer is not None and self.viewer.poll() is None:
                self.evt.log("viewer", "already running")
                return
            cmd = [sys.executable, cfg.VIEWER_PY,
                   "--rtp-port", str(cfg.RTP_PORT),
                   "--bbox-port", str(cfg.BBOX_PORT),
                   "--decoder", cfg.DECODER,
                   "--sink", cfg.SINK,
                   "--no-gl",   # the GL rotate path (glupload!gltransformation!gldownload) fails
                                # to negotiate caps on this GPU/gst and kills the whole pipeline
                                # (not-negotiated -4); --no-gl keeps the stream + horizon-lock via
                                # the non-GL rotate path. Flip sign below if the horizon tilts wrong.
                   "--horizon-sign", str(getattr(cfg, "HORIZON_SIGN", 1)),
                   "--fb-port", str(cfg.FB_PORT)]
            # Adaptive-feedback target: prefer the LIVE discovered TX address.
            # The static rx_config.py FB_HOST went stale when the AP renumbered
            # 51.x -> 50.x, silently sending every FbMsg to a dead address -- the
            # board never saw feedback and the AQC ladder sat at L0 8 Mbps
            # (found in the 2026-07-02 07:31 telemetry). The discovered tx_ip is
            # authoritative; FB_HOST remains only as a manual-run fallback.
            fb_host = self.tx_ip or cfg.FB_HOST
            if fb_host:
                cmd += ["--fb-host", fb_host]
            env = dict(os.environ)
            env.setdefault("DISPLAY", ":0")
            try:
                # capture stdout so we can parse the viewer's [fb] receive-quality
                # line for the telemetry CSV; a reader thread re-prints every line
                # so the operator console output is UNCHANGED.
                self.viewer = subprocess.Popen(
                    cmd, env=env, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, bufsize=1, universal_newlines=True)
                self.evt.log("viewer_start",
                             "pid=%d dec=%s fb=%s" % (self.viewer.pid,
                                                      cfg.DECODER, fb_host))
                threading.Thread(target=self._viewer_reader,
                                 args=(self.viewer,), daemon=True).start()
            except Exception as e:
                self.evt.log("viewer_fail", str(e))

    # Phase1: the viewer now emits BOTH the true rendered-frame rate (disp_fps ~60)
    # and the SEI/detection rate (detect_fps ~20), plus the legacy dec_fps (== the
    # real rate now). Named groups so field order/presence is tolerant; decode_fps
    # in the CSV carries the TRUE display fps (disp_fps, or dec_fps as a fallback
    # for an older viewer). detect_fps is logged in its own column.
    _FB_RE = re.compile(
        r"\[fb\].*?loss=(?P<loss>-?\d+(?:\.\d+)?)%\s+"
        r"recv=(?P<recv>-?\d+(?:\.\d+)?)\s*kbps"
        r"(?:.*?\bdisp_fps=(?P<disp>-?\d+(?:\.\d+)?))?"
        r"(?:.*?\bdetect_fps=(?P<detect>-?\d+(?:\.\d+)?))?"
        r"(?:.*?\bdec_fps=(?P<dec>-?\d+(?:\.\d+)?))?"
        r"(?:.*?\bjitter=(?P<jitter>\S+))?")

    def _viewer_reader(self, v):
        """Re-print the viewer's stdout (so the console is unchanged) and parse
        its [fb] line into the receive-quality telemetry snapshot."""
        try:
            for line in iter(v.stdout.readline, ""):
                if not line:
                    if v.poll() is not None:
                        break
                    continue
                sys.stdout.write(line)
                sys.stdout.flush()
                if "[fb]" in line:
                    m = self._FB_RE.search(line)
                    if m:
                        q = {"loss_pct": float(m.group("loss")),
                             "recv_kbps": float(m.group("recv"))}
                        # frames are genuinely flowing only when bytes arrive; a
                        # [fb] line with recv=0 kbps prints every second even when
                        # the stream is silently stalled, so gate on recv_kbps>0.
                        if q["recv_kbps"] > 0:
                            self.last_frame_t = time.monotonic()
                        # decode_fps carries the TRUE display rate: disp_fps if the
                        # viewer emits it, else the legacy dec_fps (== real now).
                        real = m.group("disp") or m.group("dec")
                        if real is not None:
                            q["decode_fps"] = float(real)
                        if m.group("detect") is not None:
                            q["detect_fps"] = float(m.group("detect"))
                        jit = m.group("jitter")
                        if jit is not None and jit != "na":
                            try:
                                q["jitter_ms"] = float(jit)
                            except ValueError:
                                pass
                        with self.telem_lock:
                            self.rx_quality.update(q)
        except Exception:
            pass

    def stop_viewer(self, reason="stop"):
        with self.viewer_lock:
            v = self.viewer
            self.viewer = None
        if v is None:
            return
        try:
            v.terminate()
            v.wait(timeout=4)
        except Exception:
            try:
                v.kill()
            except Exception:
                pass
        self.evt.log("viewer_stop", reason)

    # =======================================================================
    # operator commands
    # =======================================================================
    def _issue_start(self, why):
        """Ensure the viewer is up, then send MSG_START. Called WITHOUT sup_lock:
        it blocks for ~0.8s (the pre-START settle) and must never hold the lock
        across that. Serialisation against the operator / reconnect-resume /
        error-retry / watchdogs is provided by the CALLER, which ARMS the PENDING
        window (pending=True) under the lock before releasing it and calling us;
        only the single supervisor thread issues auto-(re)STARTs, and it will not
        fire again while pending+since_start<grace. Returns True if START sent.
        viewer must be up BEFORE the board sends frames (no periodic IDR -> a
        late-joining viewer stays black until the next, never, keyframe)."""
        if self.conn is None:
            return False
        self.start_viewer()
        time.sleep(0.8)
        ok = self._send({"type": P.MSG_START, "ts": now_iso()})
        if ok:
            self.last_start_t = time.monotonic()
            self.evt.log("op_start" if why == "operator" else "auto_start",
                         "sent START to TX (%s)" % why)
        return ok

    def cmd_start(self):
        if self.conn is None:
            print("not connected to a TX yet", flush=True)
            return
        # a manual START is an explicit operator request: (re)arm the supervisor.
        # Reset the crash-loop cap so 's' always works even after auto-retry gave
        # up, and let the supervisor own the lifecycle from here on. We ARM the
        # PENDING window under the lock (so the supervisor SUPERVISES this very
        # first START -- it judges it after the grace and auto-retries if it never
        # yields frames), then do the blocking _issue_start OUTSIDE the lock so the
        # operator console returns at once. The supervisor will NOT double-start:
        # with pending=True it sees since_start<grace and just waits out the grace.
        now = time.monotonic()
        with self.sup_lock:
            self.want_stream = True
            self.want_stream_changed = now
            self.fail_count = 0
            self.exhausted = False
            self.pending = True
            self.error_retry = False
            self.next_retry_at = 0.0
            self.stream_up_since = 0.0
            self.last_frame_t = 0.0
            self.last_start_t = now
        self._issue_start("operator")

    def cmd_stop(self):
        # the operator no longer wants video -> the supervisor must stop trying.
        # Hold sup_lock ONLY to flip the flags (fast); do the blocking MSG_STOP
        # send + viewer teardown OUTSIDE the lock so this returns immediately and
        # the supervisor (which only ARMS under the lock, never blocks under it)
        # cannot be racing a START against us. want_stream=False is now visible,
        # so any auto-START in flight will re-check it after its I/O and abort.
        with self.sup_lock:
            self.want_stream = False
            self.want_stream_changed = time.monotonic()
            self.exhausted = False
            self.fail_count = 0
            self.pending = False
            self.error_retry = False
            self.next_retry_at = 0.0
        if self._send({"type": P.MSG_STOP, "ts": now_iso()}):
            self.evt.log("op_stop", "sent STOP to TX")
        self.stop_viewer("operator STOP")

    def cmd_ping(self):
        self._send({"type": P.MSG_PING, "ts": now_iso()})
        self.evt.log("op_ping", "")

    def cmd_info(self):
        v = self.viewer is not None and self.viewer.poll() is None
        print("link: TX=%s (%s) state=%s  viewer=%s"
              % (self.tx_ip, self.tx_host, self.tx_state,
                 "running" if v else "stopped"), flush=True)

    # =======================================================================
    # resilience: shared backoff/cap + the single stream-supervisor thread
    # =======================================================================
    # --- the auto-(re)start lifecycle is a tiny 3-state machine, all under
    # sup_lock: IDLE (nothing pending) -> after we issue an auto-START we are
    # PENDING until grace elapses -> then one evaluation decides success (frames
    # flowing -> back to healthy/IDLE) or failure (increment counter, arm backoff,
    # maybe exhaust). The healthy steady-state ages the counter toward a reset. ---

    def _backoff_for(self, n):
        base = getattr(cfg, "RETRY_BACKOFF_BASE", 1.0)
        mx = getattr(cfg, "RETRY_BACKOFF_MAX", 16.0)
        return min(mx, base * (2 ** max(0, n - 1)))

    def _note_failure(self, why):
        """An auto-(re)start did NOT bring frames back. Increment the consecutive-
        failure counter, arm the next backoff, and -- at the cap -- give up auto-
        retry (stay quiescent, want_stream still True). Caller holds sup_lock."""
        self.fail_count += 1
        self.stream_up_since = 0.0
        cap = getattr(cfg, "MAX_AUTO_RETRY", 5)
        if self.fail_count >= cap:
            self.exhausted = True
            self.evt.log("auto_retry_exhausted",
                         "%s: gave up after %d attempts" % (why, self.fail_count))
            print("\n*** auto-retry exhausted after %d attempts -- "
                  "press 's' to retry manually ***\n" % self.fail_count, flush=True)
            return
        backoff = self._backoff_for(self.fail_count)
        self.next_retry_at = time.monotonic() + backoff
        self.evt.log("auto_retry_scheduled",
                     "%s: attempt %d/%d in %.1fs"
                     % (why, self.fail_count + 1, cap, backoff))

    def _note_success(self):
        """Frames are flowing. Once they've flowed for RETRY_RESET_AFTER seconds,
        clear the consecutive-failure counter so a later glitch starts fresh."""
        now = time.monotonic()
        if self.stream_up_since == 0.0:
            self.stream_up_since = now
            return
        if self.fail_count and (now - self.stream_up_since) >= \
                getattr(cfg, "RETRY_RESET_AFTER", 30.0):
            self.evt.log("auto_retry_reset",
                         "stream good %.0fs -> reset failure counter (was %d)"
                         % (now - self.stream_up_since, self.fail_count))
            self.fail_count = 0

    def _viewer_alive(self):
        v = self.viewer
        return v is not None and v.poll() is None

    def _frames_ok(self, now):
        return (self.last_frame_t > 0 and
                (now - self.last_frame_t) <= getattr(cfg, "STALL_TIMEOUT", 6.0))

    def _arm_pending(self, now):
        """Arm the PENDING-evaluation window for an auto-(re)START that is about to
        be dispatched. MUST be called under sup_lock, immediately BEFORE releasing
        the lock and calling _auto_start (which does the blocking I/O lock-free).
        Setting last_start_t=now starts the grace clock so the supervisor does not
        re-fire while _auto_start is still doing its (slow) teardown+spawn."""
        self.pending = True
        self.error_retry = False         # intent consumed by this (re)start
        self.last_frame_t = 0.0          # measure freshness from THIS start
        self.last_start_t = now
        self.next_retry_at = 0.0

    def _auto_start(self, why):
        """Issue an auto-(re)START. Called WITHOUT sup_lock so the blocking viewer
        teardown (stop_viewer can wait up to 4s for the SIGTERM-EOS recording
        finalize) and the 0.8s pre-START settle do NOT hold the lock -- an operator
        pressing 'x' must win in ~1s, not block ~5s behind an auto-START.

        Contract: the CALLER has already ARMED the PENDING window under the lock
        (pending=True; error_retry=False; last_frame_t=0.0; last_start_t=now;
        next_retry_at=0.0) and released the lock before calling us. We do the
        blocking teardown + spawn here with NO lock held, then RE-ACQUIRE the lock
        and verify the operator did not STOP during the I/O. If they did, operator
        STOP WINS: we tear the just-started viewer down and clear pending.

        A clean viewer teardown first guarantees a fresh START -> fresh keyframe
        (no periodic IDR), curing a black/stalled viewer."""
        self.evt.log({"auto-resume": "auto_resume",
                      "engine-error": "auto_retry_error",
                      "viewer-watchdog": "viewer_watchdog",
                      "stall-watchdog": "stall_watchdog"}.get(why, "auto_start"), why)
        # ---- blocking I/O, NO lock held ----
        self.stop_viewer("auto (re)start: %s" % why)
        ok = self._issue_start(why)      # start_viewer + 0.8s settle + send START
        # ---- re-acquire the lock; let the operator's STOP win if it raced us ----
        with self.sup_lock:
            if not self.want_stream or self.conn is None:
                # operator pressed STOP (or the link dropped) while we were doing
                # the blocking teardown/spawn -> operator STOP WINS. Abandon this
                # attempt and tear the just-started viewer down so no orphan runs.
                self.pending = False
                self.next_retry_at = 0.0
                aborted = (not self.want_stream and ok)
                if aborted:
                    self.stop_viewer("aborted: operator stopped")
                    # our MSG_START may have reached the board AFTER the operator's
                    # MSG_STOP (cmd_stop sent STOP while we were mid-spawn) -> re-
                    # send STOP so the BOARD also ends stopped, not left streaming.
                    if self.conn is not None:
                        self._send({"type": P.MSG_STOP, "ts": now_iso()})
                        self.evt.log("auto_start_aborted",
                                     "operator stopped during %s -> STOP re-sent" % why)
                return
            if not ok:
                # couldn't even send START (link just dropped) -> count + back off
                self.pending = False
                self._note_failure("%s send" % why)

    def stream_supervisor(self):
        """The SINGLE reconciler thread (~1 Hz). It drives reality -> want_stream
        and owns every auto-(re)START so the watchdogs, the reconnect-resume and
        the error-retry can never double-fire. Cases:
          * !want_stream / exhausted / link down -> idle (manual or clean stop).
          * PENDING (we just auto-STARTed) -> after grace, judge success/failure.
          * viewer died while wanted -> VIEWER_WATCHDOG restart + re-START.
          * frames silent > STALL_TIMEOUT while viewer up -> STALL_WATCHDOG.
          * healthy (viewer up, frames flowing) -> age the failure counter."""
        interval = getattr(cfg, "SUPERVISOR_INTERVAL", 1.0)
        while self.running:
            time.sleep(interval)
            try:
                self._supervise_once()
            except Exception as e:
                self.evt.log("supervisor_exception", str(e), echo=False)

    def _supervise_once(self):
        if not self.want_stream or self.exhausted or self.conn is None:
            return
        now = time.monotonic()
        grace = getattr(cfg, "STALL_TIMEOUT", 6.0) + 2.0
        # All evaluation + arming happens under the lock; the chosen action's
        # blocking I/O (stop_viewer + _issue_start) is dispatched via _auto_start
        # AFTER the lock is released, so an operator STOP is never blocked behind
        # an auto-START. We only capture WHICH action to take here.
        why = None
        with self.sup_lock:
            if not self.want_stream or self.exhausted or self.conn is None:
                return
            viewer_ok = self._viewer_alive()
            frames_ok = self._frames_ok(now)
            since_start = (now - self.last_start_t) if self.last_start_t else 1e9

            # ---- PENDING: an auto-START is on probation; judge ONLY after the
            # FULL grace window. A brief recv_kbps>0 blip INSIDE the window must
            # not score a false "recovered" -- if it did, a board engine that
            # emits a few frames then re-crashes would never increment fail_count
            # and would defeat the crash-loop cap (it would busy-loop forever at
            # the 1s backoff). By waiting out the grace first, a re-crash (which
            # tears the viewer down via the MSG_ERROR handler -> viewer_ok False,
            # or simply stops frames -> frames_ok False) is correctly scored as a
            # FAILURE, so fail_count climbs and MAX_AUTO_RETRY actually trips. ----
            if getattr(self, "pending", False):
                if since_start < grace:
                    return                         # still settling -- wait out grace
                if frames_ok and viewer_ok:
                    self.pending = False           # survived the grace WITH frames
                    self.stream_up_since = now
                    self.evt.log("auto_recovered",
                                 "frames flowing again after auto (re)start")
                    return
                # grace elapsed without a live, frame-producing stream -> failed
                self.pending = False
                self._note_failure("auto (re)start")
                return

            # ---- healthy: viewer up AND frames flowing -> age the counter ----
            if viewer_ok and frames_ok:
                self._note_success()
                return

            # ---- waiting out a backoff before the next attempt? ----
            if now < self.next_retry_at:
                return

            # ---- viewer DIED -> engine-error retry OR viewer watchdog ----
            if not viewer_ok:
                # the viewer is torn down both when the BOARD engine crashes (an
                # engine-error retry, gated by AUTO_RETRY_ON_ERROR) and when the
                # viewer subprocess itself dies (the viewer watchdog). The
                # error_retry flag set by the MSG_ERROR handler tells them apart.
                if self.error_retry:
                    if not getattr(cfg, "AUTO_RETRY_ON_ERROR", True):
                        return
                    why = "engine-error"
                elif getattr(cfg, "VIEWER_WATCHDOG", True):
                    why = "viewer-watchdog"
                else:
                    return
            # ---- viewer alive but frames SILENT -> stall watchdog ----
            elif not frames_ok:
                if not getattr(cfg, "STALL_WATCHDOG", True):
                    return
                if since_start < grace:
                    return            # conservative: let a fresh START settle first
                why = "stall-watchdog"
            else:
                return

            # action chosen -> ARM the PENDING window under the lock NOW, then
            # release and do the blocking (re)START outside the lock.
            self._arm_pending(now)

        if why:
            self._auto_start(why)

    def resume_on_reconnect(self):
        """Called from run() right after a (re)connect reaches READY. If the
        operator already wanted video before the drop, auto re-START now with NO
        key press. The very first START is still an operator 's' (want_stream is
        only True after one)."""
        if not getattr(cfg, "AUTO_RESUME", True):
            return
        with self.sup_lock:
            if not self.want_stream or self.exhausted or self.conn is None:
                return
            # the old viewer was torn down by _on_link_lost; reissue cleanly.
            # Arm the PENDING window under the lock, then dispatch the blocking
            # (re)START outside it so an operator STOP is never blocked behind us.
            self._arm_pending(time.monotonic())
        self._auto_start("auto-resume")

    def console(self):
        print(HELP, flush=True)
        for raw in sys.stdin:
            if not self.running:
                break
            k = raw.strip().lower()
            if k == "s":
                self.cmd_start()
            elif k == "x":
                self.cmd_stop()
            elif k == "p":
                self.cmd_ping()
            elif k == "i":
                self.cmd_info()
            elif k == "q":
                print("quitting...", flush=True)
                self.cmd_stop()
                self._send({"type": P.MSG_BYE, "ts": now_iso()})
                self.running = False
                break
            elif k == "":
                continue
            else:
                print("unknown key '%s'\n%s" % (k, HELP), flush=True)

    # =======================================================================
    # telemetry: 1 Hz clean CSV (local quality + board summary + OpenWrt AP)
    # =======================================================================
    def telemetry_loop(self):
        while self.running:
            t0 = time.time()
            row = {c: "" for c in RX_TELEM_COLUMNS}
            row["ts"] = now_iso()

            # ---- local receive quality (from the viewer [fb] line) ----
            with self.telem_lock:
                q = dict(self.rx_quality)
                txd = dict(self.tx_summary)
            for k in ("recv_kbps", "loss_pct", "decode_fps", "detect_fps",
                      "jitter_ms"):
                if k in q:
                    row[k] = q[k]

            # ---- board's forwarded summary (MSG_TELEM) ----
            tx_map = {
                "fps_sent": "tx_fps_sent", "ae_exposure": "tx_ae_exposure",
                "ae_gain": "tx_ae_gain", "imu_roll_deg": "tx_imu_roll",
                "wifi_signal_dbm": "tx_wifi_signal_dbm",
                "wifi_tx_mbps": "tx_wifi_tx_mbps",
                "enc_target_kbps": "tx_enc_target_kbps",
                "aqc_level": "tx_aqc_level",
                "cpu_pct": "tx_cpu_pct", "temp_c": "tx_temp_c",
                "time_offset_s": "tx_time_offset_s",
                "time_applied_offset_s": "tx_time_applied_offset_s",
            }
            for src, dst in tx_map.items():
                if txd.get(src, "") != "":
                    row[dst] = txd[src]

            # ---- OpenWrt AP association data (best-effort, never blocks) ----
            try:
                for k, v in self.ap.sample().items():
                    row[k] = v
            except Exception as e:
                self.evt.log("ap_sample_fail", str(e), echo=False)

            try:
                self.telem.write(row)
            except Exception as e:
                self.evt.log("telem_write_fail", str(e))

            # ---- feed the RX's own quality back to the TX so its CSV is complete
            if q and self.conn is not None:
                self._send({"type": P.MSG_TELEM, "ts": now_iso(),
                            "d": {"loss_pct": q.get("loss_pct"),
                                  "recv_kbps": q.get("recv_kbps"),
                                  "decode_fps": q.get("decode_fps")}})

            dt = time.time() - t0
            time.sleep(max(0.2, cfg.TELEM_INTERVAL - dt))

    # =======================================================================
    def run(self):
        self.evt.log("rx_boot", "ground_rx supervisor starting")
        threading.Thread(target=self.telemetry_loop, daemon=True).start()
        self.evt.log("telem_start", os.path.basename(self.telem.fh.name))
        # The operator console reads stdin in its own thread for the whole life
        # of the program, so a dropped TX link can trigger an automatic
        # re-discovery WITHOUT needing the operator to press a key.
        threading.Thread(target=self.console, daemon=True).start()
        # The stream-supervisor reconciles reality -> want_stream for the whole
        # life of the program (viewer-watchdog, stall-watchdog, error/backoff).
        threading.Thread(target=self.stream_supervisor, daemon=True).start()
        while self.running:
            ip, port, host = self.discover_tx()
            if ip is None:
                if not self.running:
                    break
                self.evt.log("discover_retry", "no TX, retrying")
                continue
            if not self.connect_tx(ip, port):
                time.sleep(2)
                continue
            print("\n>>> READY: connected to TX %s (%s). Press s=start x=stop q=quit\n"
                  % (host, ip), flush=True)
            # AUTO-RESUME: if the operator already wanted video before the link
            # dropped, re-issue START now WITHOUT a key press. (The very first
            # START is still an operator 's'; want_stream is only True after one.)
            self.resume_on_reconnect()
            # stay here while the control link is alive; re-discover when it drops
            while self.running and self.conn is not None:
                time.sleep(0.5)
            if not self.running:
                break
            self.evt.log("rediscover", "TX link gone, re-discovering")
            time.sleep(1)

        self.stop_viewer("shutdown")
        c = self.conn
        if c:
            try:
                c.close()
            except Exception:
                pass
        self.ap.close()
        self.telem.close()
        self.evt.log("rx_exit", "")
        self.evt.close()


HELP = ("OPERATOR KEYS:  s=START  x=STOP  p=PING  i=INFO  q=QUIT   "
        "(type the letter + Enter)")


def main():
    sup = RxSupervisor()
    try:
        sup.run()
    except KeyboardInterrupt:
        sup.running = False
        sup.stop_viewer("ctrl-c")
        sup.ap.close()
        sup.telem.close()
        sup.evt.log("rx_exit", "ctrl-c")
        sup.evt.close()


if __name__ == "__main__":
    main()
