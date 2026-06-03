"""calib_common.py — shared bench-calibration helpers for radio-robot-c.

Mirrors the prior repo's test/calibrate helpers. Runs from THIS project's venv,
which provides both pyserial and the aprilcam camera client (declared as the
`calibrate` dependency group in pyproject.toml, installed by `uv sync`):

    cd /Volumes/Proj/proj/RobotProjects/radio-robot-c
    uv run python tests/calibrate/<prog>.py

Provides three things the calibration programs share:

  * Relay  — drives the robot over the RADIORELAY (RAW250 transparent data
             plane) with protocol-v2 commands. The relay's command plane is
             entered on open (DTR reset), configured, then `!GO` drops it into
             the transparent plane where everything we write goes to the robot.
  * Cam    — overhead aprilcam ground truth for the robot's AprilTag (tag 100),
             with retry/reconnect because gRPC get_tags() flakes intermittently.
  * config — load/save the per-robot calibration block in data/robots/<name>.json
             plus the OTOS linear-scalar int8 <-> float-scale conversions.

Units, confirmed against the firmware (source/app/CommandProcessor.cpp,
source/hal/OtosSensor.cpp):
  * SNAP   -> "TLM ... enc=L,R pose=x,y,h"  enc = cumulative encoder DEGREES;
             pose = FUSED odometry (mm, mm, centidegrees).
  * OP     -> "OK rawpos x=.. y=.. h=.. (raw LSB)"  raw OTOS position; the chip
             has already applied the linear scalar. 1 LSB = 0.305176 mm
             (INT16, +/-10 m full scale = 10000/32768 mm per LSB).
  * OL n   -> sets OTOS linear scalar int8; scale = 1.0 + n*0.001.
  * GET/SET ml,mr -> encoder mm-per-wheel-degree (left/right).
  * P 4 1  -> digital port 4 high (the laser). P 4 0 -> off.
"""

import json
import math
import re
import statistics
import time
from pathlib import Path

import serial

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
BAUD = 115200
RELAY_PORT_DEFAULT = "/dev/cu.usbmodem21421302"
ROBOT_TAG = 100                 # tovez wears AprilTag 100 (tag 1 is a field marker)
LASER_PORT = 4                  # J4 digital port — the line laser
OTOS_MM_PER_LSB = 10000.0 / 32768.0   # 0.305176 mm/LSB (SparkFun OTOS position)

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_CONFIG = REPO_ROOT / "data" / "robots" / "tovez.json"

_RE_ENC = re.compile(r"enc=(-?\d+),(-?\d+)")
_RE_POSE = re.compile(r"pose=(-?\d+),(-?\d+),(-?\d+)")
_RE_RAWPOS = re.compile(r"x=(-?\d+)\s+y=(-?\d+)\s+h=(-?\d+)")
_RE_SCALAR = re.compile(r"scalar=(-?\d+)")
_RE_ML = re.compile(r"\bml=(-?[\d.]+)")
_RE_MR = re.compile(r"\bmr=(-?[\d.]+)")


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def yaw_delta(a: float, b: float) -> float:
    """Smallest signed (b - a) in radians, wrapped to (-pi, pi]."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def dist2d(a, b) -> float | None:
    if not a or not b:
        return None
    return math.hypot(b[0] - a[0], b[1] - a[1])


# --------------------------------------------------------------------------- #
# OTOS scalar <-> scale conversions
# --------------------------------------------------------------------------- #
def scale_to_int8(scale: float) -> int:
    """OTOS linear/angular float scale -> int8 register value (clamped)."""
    return max(-128, min(127, round((scale - 1.0) / 0.001)))


def int8_to_scale(n: int) -> float:
    return 1.0 + n * 0.001


# --------------------------------------------------------------------------- #
# Per-robot calibration config (data/robots/<name>.json)
# --------------------------------------------------------------------------- #
def load_config(path: Path = ROBOT_CONFIG) -> dict:
    return json.loads(Path(path).read_text())


def save_updates(calibration: dict | None = None, vision: dict | None = None,
                 path: Path = ROBOT_CONFIG) -> None:
    """Merge updates into the config's `calibration` and/or `vision` blocks and
    write back, preserving every other field and the 2-space indentation."""
    cfg = load_config(path)
    if calibration:
        cfg.setdefault("calibration", {}).update(calibration)
    if vision:
        cfg.setdefault("vision", {}).update(vision)
    Path(path).write_text(json.dumps(cfg, indent=2) + "\n")


# --------------------------------------------------------------------------- #
# Overhead camera ground truth
# --------------------------------------------------------------------------- #
class Cam:
    """Robot-tag ground truth from the aprilcam daemon, hardened against the
    intermittent gRPC get_tags() failures seen at the bench."""

    def __init__(self, tag_id: int = ROBOT_TAG):
        self.tag_id = tag_id
        self._connect()

    def _connect(self):
        from aprilcam.client.control import DaemonControl
        from aprilcam.config import Config
        self._DaemonControl = DaemonControl
        self._Config = Config
        self.dc = DaemonControl.connect_default(Config.load())
        cams = self.dc.list_cameras()
        if not cams:
            raise SystemExit("aprilcam: no cameras open")
        c0 = cams[0]
        self.cam = c0 if isinstance(c0, str) else getattr(c0, "id", c0)

    def _reconnect(self):
        try:
            self.dc.close()
        except Exception:
            pass
        time.sleep(0.4)
        self._connect()

    def _read_once(self):
        """One frame -> list of (x_mm, y_mm, yaw_rad) for the robot tag."""
        out = []
        tf = self.dc.get_tags(self.cam)
        for t in tf.tags:
            if t.id == self.tag_id and getattr(t, "world_xy", None) is not None:
                out.append((float(t.world_xy[0]) * 10.0,   # cm -> mm
                            float(t.world_xy[1]) * 10.0,
                            float(t.yaw)))
        return out

    def pose(self, samples: int = 6, settle: float = 0.05):
        """Median (x_mm, y_mm, yaw_rad) of the robot tag over a few frames.
        Returns None if the tag is never seen. Retries/reconnects on gRPC error."""
        xs, ys, yaws = [], [], []
        attempts = 0
        while len(xs) < samples and attempts < samples * 4:
            attempts += 1
            try:
                for (x, y, yaw) in self._read_once():
                    xs.append(x); ys.append(y); yaws.append(yaw)
            except Exception:
                self._reconnect()
            time.sleep(settle)
        if not xs:
            return None
        cy = math.atan2(statistics.fmean(math.sin(v) for v in yaws),
                        statistics.fmean(math.cos(v) for v in yaws))
        return (statistics.median(xs), statistics.median(ys), cy)

    def close(self):
        try:
            self.dc.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Robot link over the RADIORELAY
# --------------------------------------------------------------------------- #
class Relay:
    """RADIORELAY command-plane handshake, then transparent v2 to the robot."""

    def __init__(self, port: str = RELAY_PORT_DEFAULT):
        self.s = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(2.0)                      # DTR reset -> command plane boot
        self.s.reset_input_buffer()

    # ---- command plane ----
    def _cmd(self, line: str, w: float = 0.4) -> str:
        self.s.write((line + "\n").encode()); self.s.flush(); time.sleep(w)
        return self.s.read(8192).decode(errors="replace")

    def configure_go(self) -> str:
        banner = self._cmd("HELLO")
        self._cmd("!MODE RAW250")
        self._cmd("!CG 0 10")                # channel 0, group 10 (matches robot)
        self._cmd("!P 7")
        self._cmd("!GO", 0.8)                 # drop into transparent data plane
        self.s.reset_input_buffer()
        return banner.strip()

    # ---- transparent data plane (v2 to the robot) ----
    def send(self, line: str):
        self.s.write((line + "\n").encode()); self.s.flush()

    def read_until(self, want: str, timeout: float) -> str:
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            buf += self.s.read(4096)
            if want in buf.decode(errors="replace"):
                return buf.decode(errors="replace")
            time.sleep(0.04)
        return buf.decode(errors="replace")

    def query(self, cmd: str, want: str, timeout: float = 2.0) -> str:
        self.s.reset_input_buffer(); self.send(cmd)
        return self.read_until(want, timeout).strip()

    # ---- liveness ----
    def ping(self, attempts: int = 3) -> bool:
        """True iff the ROBOT (not just the relay) answers PING over the radio."""
        for _ in range(attempts):
            if "pong" in self.query("PING", "pong", 2.0).lower():
                return True
            time.sleep(0.3)
        return False

    def preflight(self, verbose: bool = True) -> bool:
        """Confirm the robot is alive before doing anything. The relay being
        present is NOT enough — the robot must answer over the radio. Returns
        True iff PING round-trips; prints a clear diagnostic when it doesn't."""
        if self.ping():
            ident = self.query("ID", "ID ", 2.0)
            ver = self.query("VER", "VER", 2.0)
            if verbose:
                print(f"  robot: ALIVE — {ident or 'PING ok'}"
                      + (f"  {ver}" if ver else ""))
            return True
        if verbose:
            print("  robot: NOT RESPONDING.\n"
                  "    The relay is up, but no PING reply came back over the radio.\n"
                  "    Check: is the robot powered on? in radio range? on the right\n"
                  "    channel/group? flashed with current firmware? Aborting — will\n"
                  "    not drive a robot that isn't talking.")
        return False

    # ---- telemetry reads ----
    def snap(self) -> dict:
        """One SNAP -> {'enc': (L,R) degrees, 'pose': (x,y,h_cdeg)}."""
        self.s.reset_input_buffer(); self.send("SNAP")
        b = self.read_until("TLM", 2.5)
        d = {}
        m = _RE_ENC.findall(b)
        if m:
            d["enc"] = (int(m[-1][0]), int(m[-1][1]))
        m = _RE_POSE.findall(b)
        if m:
            d["pose"] = (int(m[-1][0]), int(m[-1][1]), int(m[-1][2]))
        return d

    def otos_raw(self):
        """OP -> (x, y, h) raw OTOS LSB, or None."""
        b = self.query("OP", "rawpos", 1.5)
        m = _RE_RAWPOS.search(b)
        return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None

    # ---- calibration get/set ----
    def get_encoder_cal(self):
        """GET ml mr -> (mmPerDegL, mmPerDegR) or None."""
        b = self.query("GET ml mr", "CFG", 1.5)
        ml = _RE_ML.search(b); mr = _RE_MR.search(b)
        return (float(ml.group(1)), float(mr.group(1))) if (ml and mr) else None

    def set_encoder_cal(self, ml: float, mr: float) -> str:
        return self.query(f"SET ml={ml:.5f} mr={mr:.5f}", "OK", 1.5)

    def get_otos_int8(self):
        b = self.query("OL", "scalar", 1.5)
        m = _RE_SCALAR.search(b)
        return int(m.group(1)) if m else None

    def set_otos_int8(self, n: int):
        """OL n -> set linear scalar; returns the read-back int8 (or None)."""
        b = self.query(f"OL {n}", "scalar", 1.5)
        m = _RE_SCALAR.search(b)
        return int(m.group(1)) if m else None

    def otos_init(self):
        self.query("OI", "OK", 1.5)

    def otos_zero(self):
        self.query("OZ", "OK", 1.5)

    def zero_enc_pose(self):
        self.query("ZERO enc", "OK", 1.5)
        self.query("ZERO pose", "OK", 1.5)

    def set_port(self, port: int, on: bool) -> str:
        return self.query(f"P {port} {1 if on else 0}", "OK", 1.5)

    # ---- motion ----
    def drive_distance(self, speed: int, mm: int) -> bool:
        """Blocking D drive of `mm` at `speed` mm/s; wait for EVT done D.
        Returns True if completion was seen. Always deliberate: one command."""
        timeout = mm / max(speed, 1) + 3.0 + 1.0   # travel time + margin
        self.s.reset_input_buffer()
        self.send(f"D {speed} {speed} {mm}")
        return "EVT done D" in self.read_until("EVT done D", timeout)

    def stop(self):
        try:
            self.send("STOP"); time.sleep(0.3)
        except Exception:
            pass

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass
