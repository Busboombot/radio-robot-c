"""
playfield_tour_drive.py — telemetry-driven playfield tour over single firmware G commands.

The robot tours the colored playfield targets. Each leg is ONE firmware `G`
command: the host reads the robot's pose from TELEMETRY (a SNAP frame), computes
the robot-relative offset to the next target, and sends a single
`G <fwd_mm> <left_mm> <speed>` — the firmware (not the host) does the driving.
The path that gets plotted is traced ENTIRELY from the telemetry that comes back.

Pose authority: the pose in those SNAP frames is the firmware EKF's fused pose,
synthesized from the wheels turning — on hardware that synthetic pose comes from
the **Bench OTOS** device (`DBG OTOS BENCH 1`) integrating commanded wheel
velocity while the robot is up on a stand ("driving in its dream"); in the host
sim it comes from the equivalent sim OTOS/encoder model. Same program, same
single-G control path, either way — flip MODE.

Two transports, one interface:
  - SimBackend   : the in-process firmware sim (robot_radio.io.SimConnection).
  - SerialBackend: a real robot over its USB serial port, RAW pyserial (DTR
                   asserted). NOT robot_radio.SerialConnection — its direct mode
                   drops the TLM/SNAP frames we need (finding F2).

Run directly (sim) to self-verify + save a plot:
    uv run python host_tests/playfield_tour/playfield_tour_drive.py
On hardware (robot on a stand, Bench OTOS):
    uv run python host_tests/playfield_tour/playfield_tour_drive.py --serial [/dev/cu.usbmodem...]
"""
from __future__ import annotations

import json
import math
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent      # host_tests/playfield_tour
REPO = HERE.parents[1]                               # repo root
HOST = REPO / "host"
if str(HOST) not in sys.path:
    sys.path.insert(0, str(HOST))


# ---------------------------------------------------------------------------
# Playfield geometry (deskewed image + A1-centred cm targets)
# ---------------------------------------------------------------------------

def load_playfield():
    """Return (img RGB ndarray, world_to_pixel fn, SITES, CMAP, dims)."""
    import cv2
    import numpy as np

    cal = json.loads((HERE / "playfield_calibration.json").read_text())
    Hmat = np.array(cal["homography"])
    w_cm, h_cm = cal["playfield"]["width"], cal["playfield"]["height"]
    a1cx, a1cy = cal["static_markers"]["apriltag:1"]["world"]
    scale = 7.0  # px/cm in the rectified image

    raw = cv2.imread(str(HERE / "playfield.jpg"))
    M = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1.0]]) @ Hmat
    img_w, img_h = int(round(w_cm * scale)), int(round(h_cm * scale))
    img = cv2.cvtColor(cv2.warpPerspective(raw, M, (img_w, img_h)), cv2.COLOR_BGR2RGB)

    def world_to_pixel(xc, yc):
        """A1-centred world cm (+x east, +y north) -> rectified image pixel."""
        return (xc + a1cx) * scale, (a1cy - yc) * scale

    sites = [
        ("purple", -35, 24), ("black", 0, 24), ("orange", 35, 24), ("red", 35, 0),
        ("green", 35, -24), ("magenta", 0, -24), ("blue", -35, -24), ("red", -35, 0),
        ("orange", -50, 30), ("green", 0, 30), ("orange", 50, 30), ("red", 50, 0),
        ("orange", 50, -30), ("yellow", 0, -30), ("orange", -50, -30), ("blue", -50, 0),
    ]
    cmap = {"purple": "#9b30ff", "black": "#101010", "orange": "#ff8c00",
            "red": "#ff2020", "green": "#18c018", "magenta": "#ff20c0",
            "blue": "#2060ff", "yellow": "#ffd000"}
    return img, world_to_pixel, sites, cmap, (img_w, img_h)


# ---------------------------------------------------------------------------
# Transport backends — common interface:
#   send(cmd)                fire a command, ignore reply
#   snap() -> (x_mm,y_mm,h_rad) | None     read one SNAP telemetry frame
#   settle(ms)               let time pass / sim advance so the robot moves
#   close()
# ---------------------------------------------------------------------------

def _parse_tlm_pose(line: str):
    if not line.startswith("TLM"):
        return None
    d = {t.split("=", 1)[0]: t.split("=", 1)[1] for t in line.split() if "=" in t}
    if "pose" not in d:
        return None
    x, y, h = d["pose"].split(",")
    return float(x), float(y), float(h) * math.pi / 18000.0  # mm, mm, cdeg->rad


class SimBackend:
    """In-process firmware sim. settle() advances sim time (the wheels turn)."""

    name = "sim"

    def __init__(self):
        from robot_radio.io.sim_conn import SimConnection
        import hashlib, shutil, tempfile
        # Load a FRESH copy of the just-built lib under a content-keyed name.
        # dlopen() caches by path, so a long-lived Jupyter kernel keeps using the
        # OLD libfirmware_host after a rebuild ("I rebuilt but nothing changed" —
        # which silently froze the sim robot at the origin). Keying the temp name
        # on the build's size+mtime means a rebuild dlopens fresh code, while an
        # unchanged build reuses the copy. No kernel restart needed.
        libname = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
        src = REPO / "host_tests" / "build" / libname
        if not src.exists():
            raise FileNotFoundError(
                f"{src} not found — build it first:\n"
                f"  cmake -S host_tests -B host_tests/build && cmake --build host_tests/build")
        st = src.stat()
        tag = hashlib.md5(f"{st.st_size}-{st.st_mtime_ns}".encode()).hexdigest()[:12]
        fresh = pathlib.Path(tempfile.gettempdir()) / f"libfirmware_host_{tag}{src.suffix}"
        if not fresh.exists():
            shutil.copy2(src, fresh)
        self.c = SimConnection(lib_path=str(fresh))
        self.c.connect()
        # Clean synthetic plant: no slip, no encoder noise.
        self.c.set_slip(0.0, 0.0)
        self.c.set_encoder_noise(0.0)

    def configure(self):
        self.c.send("SET sTimeout=60000")   # keep the watchdog out of the way
        self.c.send("SET turnGate=35")
        self.c.send("DBG OTOS BENCH 1")     # swap otos() to the firmware BenchOtosSensor
        self.c.enable_otos_fusion(True)     # run otosCorrect -> fuse the bench sensor pose
        self.c.set_enc(0.0, 0.0)
        self.c.tick(60)

    def send(self, cmd):
        self.c.send_fast(cmd)

    def snap(self, retries=6):
        # Robust: retry if a SNAP reply lacks a TLM frame, advancing a tick
        # between tries. A None here would empty the whole leg's path.
        for _ in range(retries):
            for ln in self.c.send("SNAP", read_ms=20)["responses"]:
                p = _parse_tlm_pose(ln)
                if p:
                    return p
            self.c.tick(10)
        return None

    def settle(self, ms):
        return self.c.tick(ms)

    def close(self):
        self.c.disconnect()


class SerialBackend:
    """Real robot over USB serial, RAW pyserial (DTR asserted)."""

    name = "serial"

    def __init__(self, port="/dev/cu.usbmodem2121102", baud=115200):
        import serial
        self.p = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.5)
        self.p.reset_input_buffer()

    def configure(self):
        for cmd in ("SET sTimeout=60000", "SET turnGate=35", "DBG OTOS BENCH 1", "ZERO enc"):
            self.send(cmd)
            time.sleep(0.2)
        self.p.reset_input_buffer()

    def send(self, cmd):
        self.p.write((cmd + "\n").encode())
        self.p.flush()

    def snap(self, retries=5):
        # Robust SNAP read: a single short window often misses the TLM frame on
        # hardware. Retry, accumulating bytes over a ~150 ms window each try,
        # until a TLM pose line parses. A failed read here corrupts the leg's
        # start pose / target ranking, so it must not silently return None.
        for _ in range(retries):
            self.p.reset_input_buffer()
            self.p.write(b"SNAP\n")
            self.p.flush()
            deadline = time.time() + 0.15
            buf = ""
            while time.time() < deadline:
                buf += self.p.read(4096).decode(errors="replace")
                for ln in buf.splitlines():
                    p = _parse_tlm_pose(ln)
                    if p:
                        return p
                time.sleep(0.01)
        return None

    def settle(self, ms):
        # Real time passes; the robot drives. SNAP polling (host activity) keeps
        # the safety watchdog fed.
        time.sleep(ms / 1000.0)
        return []

    def close(self):
        self.send("X")
        self.p.close()


class RelayBackend(SerialBackend):
    """Real robot over the RADIO RELAY (untethered — e.g. driving on the playfield).

    Uses the relay !GO data-plane protocol: open the relay port (DTR asserted),
    send !GO to enter the data plane, then send PLAIN commands that the relay
    forwards to the robot over the radio. On the playfield the robot uses its
    REAL OTOS (NO DBG OTOS BENCH) — the firmware fuses it automatically; the pose
    is real (relative to where the robot started)."""

    name = "relay"

    def __init__(self, port="/dev/cu.usbmodem2121402", baud=115200):
        import serial
        self.p = serial.Serial(port, baud, timeout=0.3)
        time.sleep(1.6)
        self.p.reset_input_buffer()
        self.p.write(b"!GO\n")
        self.p.flush()
        time.sleep(0.6)
        self.p.read(4096)   # consume "# entering data plane"

    def configure(self):
        # Playfield: REAL OTOS (no bench). otosCorrect fuses it on-robot.
        # X/STOP first to clear any latched motion/safety state from a prior run.
        for cmd in ("X", "STOP", "SET sTimeout=60000", "SET turnGate=35",
                    "ZERO enc", "SI 0 0 0"):
            self.send(cmd)
            time.sleep(0.2)
        self.p.reset_input_buffer()

    def settle(self, ms):
        # Real time passes; the robot drives. Send a '+' keepalive so the
        # all-motion safety watchdog stays fed over the radio link, then read.
        self.p.write(b"+\n")
        self.p.flush()
        time.sleep(ms / 1000.0)
        return []


# ---------------------------------------------------------------------------
# Tour: one single-G leg per target, path traced from telemetry
# ---------------------------------------------------------------------------

def drive_leg(be, tx_cm, ty_cm, speed=200, arrive_mm=20, max_secs=12.0, dt_ms=48):
    """ONE firmware G command to (tx,ty) world-cm. Return the telemetry path [cm].

    Reads the current pose from telemetry, converts the world delta to the
    robot-relative (fwd,left) the firmware G expects, sends a single G, then
    polls SNAP and records the telemetry pose until the telemetry shows arrival
    (within arrive_mm) or the firmware reports done. NO second G is issued.
    """
    tx, ty = tx_cm * 10.0, ty_cm * 10.0
    p = be.snap()
    if p is None:
        return []
    x, y, h = p
    dx, dy = tx - x, ty - y
    fwd = dx * math.cos(h) + dy * math.sin(h)
    lft = -dx * math.sin(h) + dy * math.cos(h)
    be.send(f"G {int(round(fwd))} {int(round(lft))} {int(speed)}")

    path = []
    steps = int((max_secs * 1000) / dt_ms)
    for _ in range(steps):
        lines = be.settle(dt_ms)
        p = be.snap()
        if p is not None:
            path.append((p[0] / 10.0, p[1] / 10.0))   # cm
            if math.hypot(tx - p[0], ty - p[1]) <= arrive_mm:
                break
        if any("done G" in ln for ln in lines):
            break
    be.send("X")
    return path


def plan_targets(iters=10, drop=4, seed=7):
    """Deterministic, pose-INDEPENDENT target sequence [(color, tx_cm, ty_cm)].

    Planned once on the ideal trajectory (each leg assumed to end on its target),
    so it does not depend on the live robot pose. Both the sim and the bench tour
    drive this SAME list -> they hit exactly the same points and are directly
    comparable; only the telemetry path between points differs.
    """
    import random
    _, _, sites, _, _ = load_playfield()
    rng = random.Random(seed)
    targets = []
    rx, ry = 0.0, 0.0   # ideal start at A1 centre
    for _ in range(iters):
        ranked = sorted(sites, key=lambda s: math.hypot(s[1] - rx, s[2] - ry))
        color, tx, ty = rng.choice(ranked[drop:] or ranked)
        targets.append((color, tx, ty))
        rx, ry = float(tx), float(ty)   # plan the next leg from this target
    return targets


def run_tour(be, targets=None, speed=200, iters=10, seed=7):
    """Drive a FIXED target list; return segments [(color,(sx,sy),path_cm,(tx,ty),err_mm)].

    Pass the same `targets` (from plan_targets) to two backends to make both
    tours hit exactly the same points. If targets is None, one is planned from
    (iters, seed).
    """
    if targets is None:
        targets = plan_targets(iters=iters, seed=seed)
    be.configure()

    segments = []
    for it, (color, tx, ty) in enumerate(targets):
        p = be.snap()
        rx, ry = (p[0] / 10.0, p[1] / 10.0) if p else (0.0, 0.0)
        path = drive_leg(be, tx, ty, speed=speed)
        fx, fy = (path[-1] if path else (rx, ry))
        err = math.hypot(fx - tx, fy - ty) * 10.0  # mm
        segments.append((color, (rx, ry), path, (tx, ty), err))
        print(f"[{it + 1:2d}] ({rx:+5.0f},{ry:+5.0f}) -> {color:8s} ({tx:+d},{ty:+d})  "
              f"{len(path):3d} pts  end=({fx:+5.0f},{fy:+5.0f})  err={err:4.0f}mm")

        # Stale-build / dead-plant guard: if the robot never moved on the first
        # leg, the telemetry pose is frozen at the origin (e.g. a sim lib that
        # predates the MockHAL bench wiring -> OTOS fusion drags the EKF to 0).
        # Fail loudly instead of silently drawing an empty plot.
        if it == 0:
            moved = max((math.hypot(px - rx, py - ry) for px, py in path), default=0.0)
            if moved < 2.0:   # cm
                raise RuntimeError(
                    "Robot did not move on leg 1 — the telemetry pose is frozen at "
                    "the origin. The host sim library is almost certainly STALE "
                    "(missing the MockHAL Bench-OTOS wiring). Rebuild it:\n"
                    "  cmake -S host_tests -B host_tests/build && "
                    "cmake --build host_tests/build")
    return segments


def plot_tour(segments, save_path=None, show=False, mode_label=""):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    img, world_to_pixel, sites, cmap, (img_w, img_h) = load_playfield()
    fig, ax = plt.subplots(figsize=(13, 9))
    ax.imshow(img)
    for color, x, y in sites:
        px, py = world_to_pixel(x, y)
        ax.plot(px, py, 'o', ms=12, mfc=cmap.get(color, 'white'), mec='white', mew=1.6, zorder=3)
    for color, (sx, sy), path, (tx, ty), err in segments:
        if not path:
            continue
        xy = [world_to_pixel(wx, wy) for (wx, wy) in [(sx, sy)] + path]
        ax.plot([p[0] for p in xy], [p[1] for p in xy], '-', lw=2.8,
                color=cmap.get(color, 'white'), alpha=0.95, zorder=4)
    spx, spy = world_to_pixel(0, 0)
    ax.plot(spx, spy, '*', ms=22, mfc='white', mec='black', mew=1.2, zorder=5)
    ax.set_xlim(0, img_w); ax.set_ylim(img_h, 0)
    label = f"  ·  {mode_label}" if mode_label else ""
    ax.set_title(f"Telemetry-traced single-G playfield tour{label}  ·  {len(segments)} hops  ·  "
                 "path drawn from SNAP pose", fontsize=12)
    ax.axis('off'); plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=110, bbox_inches="tight")
        print(f"saved {save_path}")
    if show:
        plt.show()
    return fig


def main():
    serial_mode = "--serial" in sys.argv
    if serial_mode:
        port = next((a for a in sys.argv[1:] if a.startswith("/dev/")), "/dev/cu.usbmodem2121102")
        be = SerialBackend(port=port)
    else:
        be = SimBackend()
    targets = plan_targets()
    try:
        segs = run_tour(be, targets)
    finally:
        be.close()
    n_ok = sum(1 for *_, err in segs if err <= 60.0)
    print(f"\n{n_ok}/{len(segs)} legs arrived within 60mm (telemetry)")
    out = HERE / f"playfield_tour_{be.name}.png"
    plot_tour(segs, save_path=out)


if __name__ == "__main__":
    main()
