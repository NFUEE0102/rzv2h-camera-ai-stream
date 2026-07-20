#!/usr/bin/env python3
"""
board_tx.py -- AUTONOMOUS TX supervisor for the RZ/V2H camera+AI video/detection link.

Runs UNATTENDED on the board (systemd unit board-tx.service). It never streams on
its own -- it comes up, waits for the wlx NIC, then makes itself DISCOVERABLE and
sits idle. A ground-station RX discovers it (UDP broadcast), connects (TCP), and
issues START / STOP. The supervisor wraps the already-proven app_m5 engine.

State machine:
    WAIT_NIC --(wlx has IP)--> DISCOVERABLE --(RX connects)--> READY
    READY --(START)--> RUNNING
    RUNNING --(STOP)--> READY                 (engine killed, supervisor lives)
    RUNNING --(engine crash/camera fail)--> ERROR --> READY  (reported to RX, lives)
    (RX disconnects at any point) --> back to DISCOVERABLE (engine torn down)

The program NEVER exits on an engine error or an RX disconnect. Only SIGTERM /
SIGINT (systemd stop) ends it.

Run (foreground, for testing):   sudo python3 board_tx.py
Run (production):                systemd unit board-tx.service (see README)
Needs root for SO_BINDTODEVICE on the wlx NIC.
"""
import sys
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

import os
import re
import fcntl
import socket
import struct
import signal
import subprocess
import threading
import time
import csv
from datetime import datetime

import tx_config as cfg
import link_proto as P


# ===========================================================================
# small utilities
# ===========================================================================
def now_iso():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def enable_keepalive(sock, idle=3, intvl=2, cnt=3):
    """Turn on TCP keepalive so a hard drop (no FIN, e.g. WiFi out of range) is
    detected in ~idle+intvl*cnt seconds instead of the multi-second/minute TCP
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


def get_iface_ip(iface):
    """IPv4 of a NIC via SIOCGIFADDR, or None if the NIC has no address yet."""
    if not iface:
        return None
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = struct.pack("256s", iface[:15].encode("utf-8"))
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, packed)[20:24])
    except OSError:
        return None
    finally:
        s.close()


class EventLog:
    """Thread-safe CSV event log: ts,event,detail."""
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fh = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.fh)
        self.w.writerow(["ts", "event", "detail"])
        self.fh.flush()
        self.lock = threading.Lock()

    def log(self, event, detail=""):
        line = "%s  [%s] %s" % (now_iso(), event, detail)
        print(line, flush=True)
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
# telemetry: a SEPARATE clean per-second CSV (distinct from the event log) plus
# a compact summary pushed to the RX each second (MSG_TELEM). Purely additive --
# every probe is wrapped so a missing/failing tool just leaves that column blank
# and never disturbs the streaming engine or the control link.
# ===========================================================================
TELEM_COLUMNS = [
    "ts", "fps_cap", "fps_sent", "fps_drop", "infer_fps",
    "ae_exposure", "ae_gain", "ae_luma", "clip_pct", "imu_roll_deg",
    "enc_target_kbps", "aqc_level", "wifi_signal_dbm", "wifi_tx_mbps",
    "wifi_tx_retries", "wifi_tx_failed", "wlx_tx_kbps", "wlx_rx_kbps",
    "cpu_pct", "temp_c", "rx_loss_pct", "rx_recv_kbps",
    "time_offset_s", "time_applied_offset_s",
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


def _read_first(path):
    try:
        with open(path) as f:
            return f.readline().strip()
    except Exception:
        return None


def sample_cpu_jiffies():
    """Return (busy, total) jiffies from the aggregate /proc/stat 'cpu' line."""
    line = _read_first("/proc/stat")
    if not line or not line.startswith("cpu "):
        return None
    parts = line.split()
    try:
        vals = [int(x) for x in parts[1:11]]
    except ValueError:
        return None
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
    total = sum(vals)
    return total - idle, total


def sample_temp_c():
    """Hottest of the SoC thermal zones, in degrees C (millidegree files)."""
    best = None
    for z in ("0", "1", "2", "3", "4"):
        v = _read_first("/sys/class/thermal/thermal_zone%s/temp" % z)
        if v is None:
            continue
        try:
            c = int(v) / 1000.0
        except ValueError:
            continue
        if best is None or c > best:
            best = c
    return best


def sample_net_dev(iface):
    """(rx_bytes, tx_bytes) for an iface from /proc/net/dev, or None."""
    try:
        with open("/proc/net/dev") as f:
            for line in f:
                if iface in line:
                    fields = line.split(":", 1)[1].split()
                    return int(fields[0]), int(fields[8])
    except Exception:
        pass
    return None


_NUM = r"(-?\d+(?:\.\d+)?)"


def sample_wifi(iface):
    """Parse `iw dev <iface> link` + `station dump` for the live RF metrics.
    Returns a dict with whatever could be parsed (missing keys stay absent)."""
    out = {}
    try:
        link = subprocess.run(["iw", "dev", iface, "link"],
                              capture_output=True, text=True, timeout=4).stdout
        m = re.search(r"signal:\s*" + _NUM + r"\s*dBm", link)
        if m:
            out["wifi_signal_dbm"] = float(m.group(1))
        m = re.search(r"tx bitrate:\s*" + _NUM + r"\s*MBit/s", link)
        if m:
            out["wifi_tx_mbps"] = float(m.group(1))
    except Exception:
        pass
    try:
        dump = subprocess.run(["iw", "dev", iface, "station", "dump"],
                              capture_output=True, text=True, timeout=4).stdout
        m = re.search(r"tx retries:\s*(\d+)", dump)
        if m:
            out["wifi_tx_retries"] = int(m.group(1))
        m = re.search(r"tx failed:\s*(\d+)", dump)
        if m:
            out["wifi_tx_failed"] = int(m.group(1))
        # station-dump tx bitrate is per-peer; prefer it if the link line missed
        if "wifi_tx_mbps" not in out:
            m = re.search(r"tx bitrate:\s*" + _NUM + r"\s*MBit/s", dump)
            if m:
                out["wifi_tx_mbps"] = float(m.group(1))
        if "wifi_signal_dbm" not in out:
            m = re.search(r"signal:\s*" + _NUM, dump)
            if m:
                out["wifi_signal_dbm"] = float(m.group(1))
    except Exception:
        pass
    return out


def sample_camera():
    """Live v4l2 controls (exposure, gain, brightness) -- readable even while the
    engine streams from /dev/video0. Returns {} on any failure."""
    out = {}
    try:
        r = subprocess.run(
            ["v4l2-ctl", "-d", "/dev/video0",
             "--get-ctrl=exposure,gain,brightness,exposure_mode"],
            capture_output=True, text=True, timeout=4)
        for line in r.stdout.splitlines():
            m = re.match(r"\s*(\w+):\s*(-?\d+)", line)
            if m:
                out["cam_" + m.group(1)] = int(m.group(2))
    except Exception:
        pass
    return out


# ===========================================================================
# supervisor
# ===========================================================================
class TxSupervisor:
    def __init__(self):
        self.running = True               # process-level run flag (SIGTERM clears)
        self.state = P.ST_WAIT_NIC
        self.iface_ip = None
        self.rx_addr = None               # (ip, port) of the connected RX
        self.engine = None                # subprocess.Popen of app_m5
        self.engine_lock = threading.Lock()
        self.want_stream = False          # True between START and STOP
        # ---- Phase 1.5: cam-prep robustness (anti-wedge) ----
        # cam_prep_lock is a NON-blocking single-flight guard: only ONE cam-60fps.sh
        # may be pending/running at a time. If a cam-prep is already in flight (e.g.
        # hung on a wedging CRU), a new START must NOT launch a second one -- the
        # piling of v4l2-ctl calls onto a mid-retry-init CRU is the wedge mechanism.
        self.cam_prep_lock = threading.Lock()
        self.cam_prep_fail_count = 0      # consecutive cam-prep timeouts (-> wedge)
        self.camera_wedged = False        # sticky: set on wedge, cleared only by a
                                          # process restart (systemd/ reboot). Once
                                          # set, every START is REFUSED (no cam-prep,
                                          # no v4l2-ctl -> nothing more piles on).
        self.last_engine_start_t = 0.0    # monotonic: last engine start (debounce)
        os.makedirs(cfg.LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.evt = EventLog(os.path.join(cfg.LOG_DIR, "tx_events_%s.csv" % ts))
        # ---- clean per-second telemetry CSV (separate from the event log) ----
        self.telem = TelemetryLog(
            os.path.join(cfg.LOG_DIR, "tx_telemetry_%s.csv" % ts), TELEM_COLUMNS)
        # latest engine/RX values the 1 Hz telemetry thread folds in. Guarded by
        # telem_lock; written by the engine-watch + control threads, read by the
        # sampler. Plain dict so a missing field is simply absent (-> blank cell).
        self.telem_lock = threading.Lock()
        self.eng_stats = {}        # fps_cap/fps_sent/fps_drop/infer_fps/ae_*/imu_roll
        self.rx_quality = {}       # rx_loss_pct / rx_recv_kbps fed back by the RX
        self._cpu_prev = None
        self._net_prev = None      # (t, rx_bytes, tx_bytes) for throughput deltas
        self._telem_conn = None    # current RX control socket for MSG_TELEM push

    # =======================================================================
    # telemetry: parse engine stderr lines + the 1 Hz sampler thread
    # =======================================================================
    def _parse_engine_line(self, line):
        """Fold an app_m5 stderr line into self.eng_stats (best-effort). Matches
        the proven formats:
          [app_m5 live] cap=60.0fps sent=60.0fps drop=0.0fps infer=30.0fps (...)
          [ae] luma=120.0 target=110.0 clip=0.12% exp=8000 gain=2 ...
          [imu] ... roll=...        (roll, if ever printed periodically)"""
        try:
            if "[app_m5 live]" in line:
                m = re.search(r"cap=" + _NUM + r"fps\s+sent=" + _NUM +
                              r"fps\s+drop=" + _NUM + r"fps\s+infer=" + _NUM + r"fps",
                              line)
                if m:
                    with self.telem_lock:
                        self.eng_stats["fps_cap"] = float(m.group(1))
                        self.eng_stats["fps_sent"] = float(m.group(2))
                        self.eng_stats["fps_drop"] = float(m.group(3))
                        self.eng_stats["infer_fps"] = float(m.group(4))
            elif line.startswith("[ae]") and "luma=" in line:
                d = {}
                m = re.search(r"luma=" + _NUM, line)
                if m:
                    d["ae_luma"] = float(m.group(1))
                m = re.search(r"clip=" + _NUM + r"%", line)
                if m:
                    d["clip_pct"] = float(m.group(1))
                m = re.search(r"exp=(\d+)", line)
                if m:
                    d["ae_exposure"] = int(m.group(1))
                m = re.search(r"gain=(\d+)", line)
                if m:
                    d["ae_gain"] = int(m.group(1))
                if d:
                    with self.telem_lock:
                        self.eng_stats.update(d)
            elif "[imu]" in line:
                m = re.search(r"roll=" + _NUM, line)
                if m:
                    try:
                        with self.telem_lock:
                            self.eng_stats["imu_roll_deg"] = float(m.group(1))
                    except Exception:
                        pass
            elif "[capture_encoder] AQC" in line and "level=" in line:
                # Phase1 AQC ladder: capture_encoder prints its CURRENT level + the actual
                # target bitrate it applied. Fold both into the telemetry snapshot so
                # tx_enc_target_kbps reflects the DYNAMIC value (not the static 4000)
                # and the level shows in aqc_level. Format:
                #   [capture_encoder] AQC level=L1 target=5000kbps (retries/s=.. sig=.. loss=..)
                d = {}
                m = re.search(r"level=(L\d)", line)
                if m:
                    d["aqc_level"] = m.group(1)
                m = re.search(r"target=(\d+)\s*kbps", line)
                if m:
                    d["enc_target_kbps"] = int(m.group(1))
                if d:
                    with self.telem_lock:
                        self.eng_stats.update(d)
        except Exception:
            pass

    def ingest_rx_quality(self, d):
        """Store the loss%/recv_kbps the RX reports over the control link."""
        q = {}
        if d.get("loss_pct") is not None:
            q["rx_loss_pct"] = d.get("loss_pct")
        if d.get("recv_kbps") is not None:
            q["rx_recv_kbps"] = d.get("recv_kbps")
        if q:
            with self.telem_lock:
                self.rx_quality.update(q)

    def telemetry_loop(self):
        """~1 Hz: gather a clean telemetry row, write the CSV, and (if an RX is
        connected) push a compact MSG_TELEM summary so the RX logs it too."""
        while self.running:
            t0 = time.time()
            row = {c: "" for c in TELEM_COLUMNS}
            row["ts"] = now_iso()

            # ---- engine + RX-fed values (snapshot under the lock) ----
            with self.telem_lock:
                row.update({k: v for k, v in self.eng_stats.items() if k in row})
                row.update({k: v for k, v in self.rx_quality.items() if k in row})

            # ---- encoder target bitrate ----
            # Phase1 AQC: capture_encoder drives target-bitrate at runtime (the 4-level
            # ladder), so the LIVE value is what it reports over stderr (folded into
            # eng_stats["enc_target_kbps"] + eng_stats["aqc_level"] by the engine
            # parser). Only if the engine hasn't reported yet (or isn't running) do
            # we fall back to the static configured ceiling.
            if not row.get("enc_target_kbps"):
                row["enc_target_kbps"] = int(cfg.STREAM_BITRATE / 1000)

            # ---- WiFi RF ----
            for k, v in sample_wifi(cfg.TX_IFACE).items():
                row[k] = v

            # ---- wlx throughput from /proc/net/dev deltas ----
            nd = sample_net_dev(cfg.TX_IFACE)
            now = time.time()
            if nd is not None:
                rxb, txb = nd
                if self._net_prev is not None:
                    pt, prx, ptx = self._net_prev
                    dt = now - pt
                    if dt > 0:
                        row["wlx_rx_kbps"] = round(8.0 * (rxb - prx) / 1000.0 / dt, 1)
                        row["wlx_tx_kbps"] = round(8.0 * (txb - ptx) / 1000.0 / dt, 1)
                self._net_prev = (now, rxb, txb)

            # ---- camera live controls ----
            cam = sample_camera()
            if "cam_exposure" in cam and not row["ae_exposure"]:
                row["ae_exposure"] = cam["cam_exposure"]
            if "cam_gain" in cam and not row["ae_gain"]:
                row["ae_gain"] = cam["cam_gain"]

            # ---- CPU %busy from /proc/stat deltas ----
            cj = sample_cpu_jiffies()
            if cj is not None:
                if self._cpu_prev is not None:
                    db = cj[0] - self._cpu_prev[0]
                    dtot = cj[1] - self._cpu_prev[1]
                    if dtot > 0:
                        row["cpu_pct"] = round(100.0 * db / dtot, 1)
                self._cpu_prev = cj

            # ---- temperature ----
            tc = sample_temp_c()
            if tc is not None:
                row["temp_c"] = round(tc, 1)

            try:
                self.telem.write(row)
            except Exception as e:
                self.evt.log("telem_write_fail", str(e))

            # ---- push a compact summary to the RX (if connected) ----
            conn = self._telem_conn
            if conn is not None:
                summary = {k: row[k] for k in (
                    "fps_sent", "ae_exposure", "ae_gain", "imu_roll_deg",
                    "wifi_signal_dbm", "wifi_tx_mbps", "wifi_tx_retries",
                    "wifi_tx_failed", "wlx_tx_kbps", "cpu_pct", "temp_c",
                    "enc_target_kbps", "aqc_level",
                    "time_offset_s", "time_applied_offset_s") if row.get(k) != ""}
                self._safe_send(conn, {"type": P.MSG_TELEM, "d": summary,
                                       "ts": now_iso()})

            # pace to ~1 Hz
            dt = time.time() - t0
            time.sleep(max(0.2, 1.0 - dt))

    # =======================================================================
    # Phase1 time-sync: the RX is the clock master. On MSG_TIME, compute
    # offset = rx_epoch - board_now; if it exceeds the threshold, set the board
    # clock (the supervisor is root via systemd -- no sudo needed) so every board
    # log shares the RX/ground timeline even after a reboot resets the board clock.
    # No-op when already aligned. Purely additive + fully wrapped: any failure just
    # logs and leaves the clock untouched; it never disturbs streaming or the link.
    # =======================================================================
    def apply_time_sync(self, rx_epoch):
        """Set the board clock to the RX-master epoch. Returns True IFF the clock
        was actually changed (so the caller can reset its RX-liveness timer, else
        the jump would look like the RX went silent -> spurious link teardown)."""
        try:
            rx_epoch = float(rx_epoch)
        except (TypeError, ValueError):
            return False
        # SANITY BOUND: never let a bogus/corrupt rx_epoch set the board wall clock
        # to an absurd year (this runs `date -s` as root, board-wide). Accept only a
        # plausible window ~2023-07 .. ~2100 (unix 1.70e9 .. 4.10e9).
        if not (1_700_000_000.0 < rx_epoch < 4_100_000_000.0):
            self.evt.log("time_sync_fail",
                         "rejected out-of-range rx_epoch=%.3f" % rx_epoch)
            return False
        board_now = time.time()
        offset = rx_epoch - board_now
        thr = getattr(cfg, "TIME_SYNC_THRESHOLD_S", 2.0)
        # record the last measured offset for telemetry regardless of action
        with self.telem_lock:
            self.eng_stats["time_offset_s"] = round(offset, 3)
        if abs(offset) <= thr:
            self.evt.log("time_sync",
                         "aligned: offset=%.3fs <= %.1fs -> no change" % (offset, thr))
            return False
        # drifted enough to correct: set the board clock to the RX epoch.
        ok = False
        try:
            r = subprocess.run(["date", "-s", "@%.3f" % rx_epoch],
                               capture_output=True, text=True, timeout=6)
            ok = (r.returncode == 0)
            if not ok:
                # supervisor may not be root in a manual foreground test -> try sudo
                r2 = subprocess.run(
                    ["sudo", "-n", "date", "-s", "@%.3f" % rx_epoch],
                    capture_output=True, text=True, timeout=6)
                ok = (r2.returncode == 0)
        except Exception as e:
            self.evt.log("time_sync_fail", "date -s: %s" % e)
            return False
        if ok:
            with self.telem_lock:
                self.eng_stats["time_applied_offset_s"] = round(offset, 3)
            self.evt.log("time_sync",
                         "APPLIED: board was %.3fs behind RX -> clock set to %.3f"
                         % (offset, rx_epoch))
            return True
        self.evt.log("time_sync_fail",
                     "date -s returned non-zero (not root?) offset=%.3fs" % offset)
        return False

    # ---- state helper: log + push to RX if connected ----
    def set_state(self, st, conn=None, detail=""):
        self.state = st
        self.evt.log("state", "%s %s" % (st, detail))
        if conn is not None:
            self._safe_send(conn, {"type": P.MSG_STATE, "state": st,
                                   "detail": detail, "ts": now_iso()})

    def _safe_send(self, conn, msg):
        try:
            P.send_msg(conn, msg)
            return True
        except Exception as e:
            self.evt.log("send_fail", str(e))
            return False

    # =======================================================================
    # NIC wait
    # =======================================================================
    def wait_for_nic(self):
        self.set_state(P.ST_WAIT_NIC, detail=cfg.TX_IFACE)
        while self.running:
            ip = get_iface_ip(cfg.TX_IFACE)
            if ip:
                self.iface_ip = ip
                self.evt.log("nic_up", "%s = %s" % (cfg.TX_IFACE, ip))
                return True
            time.sleep(2)
        return False

    # =======================================================================
    # discovery: listen for RX broadcast DISCOVER, reply OFFER (bound to wlx)
    # =======================================================================
    def open_discovery_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                         cfg.TX_IFACE.encode("utf-8"))
        except OSError as e:
            self.evt.log("bindtodevice_warn",
                         "%s on %s -- relying on routing" % (e, cfg.TX_IFACE))
        s.bind(("0.0.0.0", P.DISCOVERY_PORT))
        s.settimeout(1.0)
        return s

    def discovery_responder(self, disc_sock, hostname):
        """Background thread: answer every DISCOVER with an OFFER while alive."""
        offer = {
            "magic": P.OFFER_MAGIC, "hostname": hostname,
            "tx_ip": self.iface_ip, "control_port": P.CONTROL_PORT,
            "state": self.state, "ts": now_iso(),
        }
        while self.running:
            try:
                data, addr = disc_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = data.decode("utf-8", "replace")
            except Exception:
                continue
            if P.DISCOVER_MAGIC in msg:
                offer["state"] = self.state
                offer["ts"] = now_iso()
                try:
                    disc_sock.sendto(__import__("json").dumps(offer).encode("utf-8"),
                                     addr)
                    self.evt.log("discover", "DISCOVER from %s -> OFFER" % addr[0])
                except OSError:
                    pass

    # =======================================================================
    # engine control
    # =======================================================================
    # Phase 1.5: cam-prep robustness helpers (anti-wedge)
    # -----------------------------------------------------------------------
    def _dmesg_shows_cru_wedge(self):
        """Best-effort: scan the dmesg tail for the kernel signatures of a wedging
        rzg2l-cru / tevs sensor. A match is strong evidence the CRU is stuck (a
        cam-prep timeout alone could just be a slow v4l2-ctl; combined with these
        lines it is a genuine wedge). Fully wrapped: any failure -> False (we then
        fall back to the consecutive-timeout threshold)."""
        try:
            r = subprocess.run(["dmesg"], capture_output=True, text=True, timeout=5)
        except Exception:
            # non-root manual test may lack dmesg perms; try sudo -n, else give up
            try:
                r = subprocess.run(["sudo", "-n", "dmesg"],
                                   capture_output=True, text=True, timeout=5)
            except Exception:
                return False
        if r.returncode != 0 or not r.stdout:
            return False
        tail = "\n".join(r.stdout.splitlines()[-60:]).lower()
        for sig in ("cru retry init", "tevs_stop_streaming failed", "rzg2l-cru"):
            if sig in tail:
                self.evt.log("cam_wedge_dmesg", "matched %r in dmesg tail" % sig)
                return True
        return False

    def _declare_camera_wedged(self, conn, why):
        """Latch camera_wedged, report a NON-retryable MSG_ERROR to the RX, and go
        to ERROR. From here every START is refused (no cam-prep runs) until the
        process restarts (a board reboot restarts board-tx fresh -> flag cleared)."""
        self.camera_wedged = True
        detail = ("camera wedged -- reboot the board (%s; rapid START/STOP wedged "
                  "the rzg2l-cru, only a reboot recovers)" % why)
        self.evt.log("camera_wedged", detail)
        self._report_error(conn, detail)

    def _run_cam_prep(self, conn):
        """Single-flight cam-60fps.sh with a short timeout + wedge detection.
        Returns True IFF the prep completed cleanly and the engine may be spawned.
        On ANY failure returns False and does NOT let a second cam-prep pile on.

        Single-flight: cam_prep_lock is acquired NON-blocking. If it's already held
        (a previous cam-prep is still running -- e.g. hung on a wedging CRU), we
        refuse to launch a second one (that piling is the wedge). timeout is short
        (v4l2-ctl finishes in <2s normally); a hang past it = the CRU is wedging."""
        if self.camera_wedged:
            # already latched -- never touch the camera again this process life
            self._report_error(conn,
                "camera wedged -- reboot the board (START refused; cam-prep skipped)")
            return False
        # single-flight: do NOT block waiting -- a second cam-prep is exactly the
        # pile-up we must prevent. If one is already in flight, refuse this START.
        if not self.cam_prep_lock.acquire(blocking=False):
            self.evt.log("cam_prep_busy",
                         "a cam-prep is already in flight -> refusing to pile a 2nd")
            self._report_error(conn,
                "cam-prep already in progress -- START ignored (single-flight)")
            return False
        try:
            timeout = getattr(cfg, "CAM_PREP_TIMEOUT_S", 8.0)
            try:
                r = subprocess.run(["bash", cfg.CAM_60FPS_SH],
                                   capture_output=True, text=True, timeout=timeout)
            except subprocess.TimeoutExpired:
                # A HANG = the CRU is (very likely) wedging. Count it, do NOT spawn
                # the engine, do NOT retry-pile. NOTE: the timed-out bash/v4l2-ctl
                # child may be stuck in D-state; we deliberately do NOT kill -9 it
                # (that cannot free a D-state task and risks nothing useful). We
                # simply back off and, past the threshold, refuse all further START.
                self.cam_prep_fail_count += 1
                self.evt.log("cam_prep_timeout",
                             "cam-60fps.sh exceeded %.0fs (fail %d/%d) -- backing off, "
                             "NOT spawning engine"
                             % (timeout, self.cam_prep_fail_count,
                                getattr(cfg, "CAM_WEDGE_FAIL_THRESHOLD", 2)))
                thr = getattr(cfg, "CAM_WEDGE_FAIL_THRESHOLD", 2)
                wedged = (self.cam_prep_fail_count >= thr
                          or self._dmesg_shows_cru_wedge())
                if wedged:
                    self._declare_camera_wedged(
                        conn, "%d cam-prep timeouts" % self.cam_prep_fail_count)
                else:
                    # not yet at the threshold: report a plain (retryable) error but
                    # the min-interval debounce still spaces any retry >=4s apart.
                    self._report_error(conn,
                        "cam-prep timed out after %.0fs (%d/%d) -- CRU may be re-"
                        "initialising; backing off" % (timeout,
                        self.cam_prep_fail_count, thr))
                return False
            except Exception as e:
                self._report_error(conn, "cam-60fps.sh failed: %s" % e)
                return False
            # clean completion -> a healthy prep clears the consecutive-timeout run
            self.cam_prep_fail_count = 0
            last = (r.stdout or r.stderr).strip().splitlines()[-1:]
            self.evt.log("cam_prep", "rc=%d %s" % (r.returncode, last))
            return True
        finally:
            self.cam_prep_lock.release()

    def start_engine(self, rx_ip, conn):
        """Run cam-60fps.sh (single-flight, short timeout, wedge-aware), then spawn
        app_m5 streaming to rx_ip. Returns True/False.

        Phase 1.5 anti-wedge order:
          0) if camera already wedged -> refuse immediately (no cam-prep, no pile).
          1) MIN_ENGINE_RESTART_S debounce -- wait out the remainder since the last
             engine start so rapid s/x/s (or auto-retry) can never hammer the camera.
          2) single-flight cam-prep with the short timeout + wedge detection.
          3) spawn app_m5 only if cam-prep completed cleanly."""
        # 0) hard refuse if the camera is already declared wedged
        if self.camera_wedged:
            self._report_error(conn,
                "camera wedged -- reboot the board (START refused)")
            return False

        # 1) minimum engine-restart interval (debounce, ALL triggers)
        min_iv = getattr(cfg, "MIN_ENGINE_RESTART_S", 4.0)
        if self.last_engine_start_t:
            elapsed = time.monotonic() - self.last_engine_start_t
            if elapsed < min_iv:
                wait = min_iv - elapsed
                self.evt.log("engine_debounce",
                             "START %.2fs after last -> waiting %.2fs (min %.0fs)"
                             % (elapsed, wait, min_iv))
                # wait out the remainder so cam-prep is never hammered. Bail early if
                # the operator STOPed (want_stream cleared) or we're shutting down.
                deadline = time.monotonic() + wait
                while time.monotonic() < deadline and self.running and self.want_stream:
                    time.sleep(0.1)
                if not self.running or not self.want_stream:
                    self.evt.log("engine_debounce",
                                 "aborted during debounce (stop/shutdown)")
                    return False

        # 2) camera 60fps prep -- single-flight + timeout + wedge detection
        if not self._run_cam_prep(conn):
            return False

        # mark the engine-start time NOW (cam-prep succeeded -> we are spawning).
        # This is the debounce anchor for the NEXT start regardless of trigger.
        self.last_engine_start_t = time.monotonic()

        # 3) spawn app_m5
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = "./libshim:." + ":" + env.get("LD_LIBRARY_PATH", "")
        env["YOLO_SEI"] = "1"
        env["YOLO_HEADLESS"] = "1"
        env["YOLO_STREAM_HOST"] = rx_ip
        env["YOLO_STREAM_PORT"] = str(cfg.STREAM_RTP_PORT)
        env["YOLO_BBOX_PORT"] = str(cfg.STREAM_BBOX_PORT)
        env["YOLO_STREAM_BITRATE"] = str(cfg.STREAM_BITRATE)
        env["YOLO_ENC_BIN"] = "./" + cfg.ENGINE_CHILD
        # Phase1 AQC: hand the child the board WiFi NIC so it can self-sample
        # `iw dev <wlx> station dump` (tx retries/s + signal dBm) for the ladder.
        env["YOLO_WLX"] = cfg.TX_IFACE
        cmd = [cfg.ENGINE_BIN] + list(cfg.ENGINE_ARGS)
        try:
            with self.engine_lock:
                self.engine = subprocess.Popen(
                    cmd, cwd=cfg.ENGINE_DIR, env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    bufsize=1, universal_newlines=True,
                    preexec_fn=os.setsid)        # own process group -> clean kill
            self.evt.log("engine_spawn",
                         "pid=%d host=%s:%d" % (self.engine.pid, rx_ip,
                                                cfg.STREAM_RTP_PORT))
        except Exception as e:
            self._report_error(conn, "app_m5 spawn failed: %s" % e)
            return False

        # 3) monitor stderr/stdout in a thread; detect early death + error strings
        threading.Thread(target=self._engine_watch, args=(conn,), daemon=True).start()
        return True

    def _engine_watch(self, conn):
        """Read engine output; forward errors; on exit-while-RUNNING report + go READY."""
        eng = self.engine
        if eng is None:
            return
        spawn_t = time.time()
        last_fps_report = 0
        ERR_KEYS = ("fail", "error", "cannot", "unable", "VIDIOC", "EBUSY",
                    "no such", "not found", "abort", "Segmentation")
        try:
            for line in iter(eng.stdout.readline, ""):
                line = line.rstrip()
                if not line:
                    if eng.poll() is not None:
                        break
                    continue
                low = line.lower()
                # fold engine stat lines into the telemetry snapshot (best-effort)
                self._parse_engine_line(line)
                # forward genuine error lines to the RX (throttled by content)
                if any(k.lower() in low for k in ERR_KEYS):
                    self._safe_send(conn, {"type": P.MSG_LOG, "level": "warn",
                                           "line": line[:300], "ts": now_iso()})
                # surface fps/sent lines as engine stats
                if ("fps" in low or "sent" in low) and time.time() - last_fps_report > 2:
                    self._safe_send(conn, {"type": P.MSG_STAT, "engine": line[:200],
                                           "ts": now_iso()})
                    last_fps_report = time.time()
        except Exception:
            pass
        rc = eng.wait()
        with self.engine_lock:
            still_ours = (self.engine is eng)
            if still_ours:
                self.engine = None
        # If we still wanted the stream (RUNNING) and the engine died -> ERROR.
        if still_ours and self.want_stream:
            dt = time.time() - spawn_t
            self.want_stream = False
            self._report_error(conn,
                               "app_m5 exited rc=%d after %.1fs (camera/encoder/child)"
                               % (rc, dt))
            # back to READY -- supervisor stays alive, ready to retry
            if conn is not None:
                self.set_state(P.ST_READY, conn, detail="recovered after engine death")

    def stop_engine(self, reason="stop"):
        """Kill the tracked engine process group cleanly (never pkill -f)."""
        with self.engine_lock:
            eng = self.engine
            self.engine = None
        self.want_stream = False
        if eng is None:
            return
        try:
            os.killpg(os.getpgid(eng.pid), signal.SIGTERM)
        except Exception:
            try:
                eng.terminate()
            except Exception:
                pass
        try:
            eng.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(eng.pid), signal.SIGKILL)
            except Exception:
                pass
        self.evt.log("engine_stop", reason)

    def _report_error(self, conn, detail):
        self.evt.log("engine_error", detail)
        if conn is not None:
            self._safe_send(conn, {"type": P.MSG_ERROR, "detail": detail,
                                   "ts": now_iso()})
            self.set_state(P.ST_ERROR, conn, detail=detail)

    # =======================================================================
    # control session: handle one connected RX until it disconnects
    # =======================================================================
    def serve_rx(self, conn, addr, hostname):
        self.rx_addr = addr
        enable_keepalive(conn)           # detect a hard RX drop (no FIN) in seconds
        self._telem_conn = conn          # let the telemetry thread push to this RX
        self.evt.log("rx_connect", "%s:%d" % addr)
        self._safe_send(conn, {"type": P.MSG_HELLO, "hostname": hostname,
                               "tx_ip": self.iface_ip, "state": self.state,
                               "ts": now_iso()})
        self.set_state(P.ST_READY, conn, detail="RX %s connected" % addr[0])

        last_rx = time.time()
        last_stat = 0.0
        conn.settimeout(1.0)
        while self.running:
            # periodic STAT heartbeat so the RX knows the TX is alive
            if time.time() - last_stat >= cfg.STAT_INTERVAL:
                eng_alive = self.engine is not None and self.engine.poll() is None
                self._safe_send(conn, {"type": P.MSG_STAT, "state": self.state,
                                       "streaming": eng_alive,
                                       "ts": now_iso()})
                last_stat = time.time()

            msg = P.recv_msg(conn, timeout=1.0)
            if msg is None:
                # distinguish idle (timeout) from a dead link
                if time.time() - last_rx > cfg.PING_DEAD_TIMEOUT:
                    self.evt.log("rx_link_dead",
                                 ">%ds silent" % cfg.PING_DEAD_TIMEOUT)
                    break
                continue
            last_rx = time.time()
            mt = msg.get("type")

            if mt == P.MSG_START:
                if self.camera_wedged:
                    # Phase 1.5: camera is wedged -- REFUSE START outright. Do NOT run
                    # cam-prep (that would pile another v4l2-ctl on the stuck CRU) and
                    # do NOT flap through RUNNING. Re-assert the non-retryable error so
                    # a late-joining / reconnecting RX also learns it needs a reboot.
                    self.evt.log("start_refused", "camera wedged -> START refused")
                    self._report_error(conn,
                        "camera wedged -- reboot the board (START refused; "
                        "cam-prep skipped so nothing more piles on the CRU)")
                elif self.engine is None:
                    self.set_state(P.ST_RUNNING, conn, detail="START")
                    self.want_stream = True
                    ok = self.start_engine(addr[0], conn)
                    if not ok:
                        # start_engine already reported the error
                        self.want_stream = False
                        self.set_state(P.ST_READY, conn, detail="start failed")
                else:
                    self._safe_send(conn, {"type": P.MSG_STATE, "state": self.state,
                                           "detail": "already running", "ts": now_iso()})
            elif mt == P.MSG_STOP:
                self.stop_engine("RX STOP")
                self.set_state(P.ST_READY, conn, detail="STOP")
            elif mt == P.MSG_PING:
                self._safe_send(conn, {"type": P.MSG_PONG, "state": self.state,
                                       "ts": now_iso()})
            elif mt == P.MSG_TELEM:
                # RX-fed receive quality (loss% / recv_kbps / decode_fps) -> fold
                # into the TX telemetry CSV so it has the full both-sides picture.
                self.ingest_rx_quality(msg.get("d", {}))
            elif mt == P.MSG_TIME:
                # Phase1: the RX is the clock master. Snap the board clock onto its
                # timeline if we drifted (e.g. a reboot reset the board clock).
                if self.apply_time_sync(msg.get("epoch")):
                    # the clock just jumped forward/back; re-baseline the RX-liveness
                    # timers on the NEW clock so the jump is NOT mistaken for the RX
                    # going silent (which would tear this freshly-established link
                    # down at the pre-flight handshake).
                    last_rx = time.time()
                    last_stat = time.time()
            elif mt == P.MSG_BYE:
                self.evt.log("rx_bye", addr[0])
                break

        # RX gone (clean or dead): tear the engine down, return to DISCOVERABLE
        self._telem_conn = None
        self.stop_engine("rx session ended")
        try:
            conn.close()
        except Exception:
            pass
        self.rx_addr = None

    # =======================================================================
    # main loop
    # =======================================================================
    def run(self):
        hostname = socket.gethostname().split(".")[0]
        self.evt.log("tx_boot", "board_tx supervisor starting host=%s" % hostname)

        if not self.wait_for_nic():
            self.evt.log("tx_exit", "no NIC and shutting down")
            return

        # 1 Hz telemetry sampler runs for the whole life of the supervisor (it
        # pushes to the RX only when one is connected; it logs the board's own
        # WiFi/CPU/temp continuously regardless of streaming state).
        threading.Thread(target=self.telemetry_loop, daemon=True).start()
        self.evt.log("telem_start", os.path.basename(self.telem.fh.name))

        disc_sock = self.open_discovery_socket()
        threading.Thread(target=self.discovery_responder,
                         args=(disc_sock, hostname), daemon=True).start()

        # TCP control server -- TX is the server; RX connects in.
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                           cfg.TX_IFACE.encode("utf-8"))
        except OSError:
            pass
        srv.bind(("0.0.0.0", P.CONTROL_PORT))
        srv.listen(1)
        srv.settimeout(1.0)

        self.set_state(P.ST_DISCOVERABLE,
                       detail="listening on %s:%d" % (self.iface_ip, P.CONTROL_PORT))

        while self.running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                # NIC could have dropped; if so, fall back to WAIT_NIC
                if get_iface_ip(cfg.TX_IFACE) is None:
                    self.evt.log("nic_down", cfg.TX_IFACE)
                    self.wait_for_nic()
                    self.set_state(P.ST_DISCOVERABLE)
                continue
            except OSError:
                break
            try:
                self.serve_rx(conn, addr, hostname)
            except Exception as e:
                self.evt.log("serve_exception", str(e))
                self.stop_engine("serve exception")
            # after any RX session ends, we are discoverable again
            if self.running:
                self.set_state(P.ST_DISCOVERABLE)

        self.evt.log("tx_exit", "main loop ended")
        self.stop_engine("shutdown")
        try:
            srv.close(); disc_sock.close()
        except Exception:
            pass
        self.telem.close()
        self.evt.close()


def main():
    sup = TxSupervisor()

    def handler(sig, frame):
        sup.evt.log("signal", "got %d, shutting down" % sig)
        sup.running = False
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    sup.run()


if __name__ == "__main__":
    main()
