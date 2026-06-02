"""robot_radio.sensors.motion_monitor — closed-loop "thrash" safety detector.

Closed-loop driving can go "crazy town": near a target the atan2 heading is
hypersensitive, so the robot oscillates / reverses wildly. This module watches
the *pose stream* and trips when that pattern appears, so the caller can STOP.

It is a pure-Python (math, time, csv) analyzer. It does NOT read sensors — the
caller FEEDS it pose samples via :meth:`ThrashMonitor.feed`. From consecutive
samples it derives a linear velocity vector and an angular velocity, then their
accelerations, and flags two kinds of thrash EVENTS:

  * **linear reversal** — the velocity vector swings around (large direction
    change) with a high linear acceleration. The robot lurched the other way.
  * **angular flip** — the angular velocity changes sign with a high angular
    acceleration. The robot snapped its spin direction.

A handful of such events inside a short time window ``window_s`` means we are
thrashing and :meth:`feed` returns ``True`` (also see :attr:`tripped`).

Thresholds are intentionally exposed as constructor args; they are initial
guesses meant to be tuned on the field.
"""

from __future__ import annotations

import csv
import math
import time
from typing import Optional


def _finite(*vals: float) -> bool:
    """True only if every value is a real, finite number (no None/NaN/inf)."""
    for v in vals:
        if v is None:
            return False
        try:
            f = float(v)
        except (TypeError, ValueError):
            return False
        if math.isnan(f) or math.isinf(f):
            return False
    return True


def _wrap_pi(a: float) -> float:
    """Wrap an angle (rad) to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _angle_between(vx0: float, vy0: float, vx1: float, vy1: float) -> float:
    """Angle (rad, 0..pi) between two velocity vectors via atan2(cross, dot)."""
    dot = vx0 * vx1 + vy0 * vy1
    cross = vx0 * vy1 - vy0 * vx1
    return abs(math.atan2(cross, dot))


class ThrashMonitor:
    """Detect oscillation / reversal "thrashing" from a fed pose stream.

    Feed it ``(t, x, y, yaw_rad)`` samples. It returns/raises a trip flag once
    enough thrash events land inside the rolling time window.
    """

    def __init__(self, csv_path: Optional[str] = None,
                 lin_accel_thresh: float = 150.0,   # cm/s^2
                 ang_accel_thresh: float = 1500.0,  # deg/s^2
                 min_speed: float = 3.0,            # cm/s, below this ignore linear dir
                 min_omega: float = 30.0,           # deg/s, below this ignore angular dir
                 reversal_deg: float = 120.0,       # vel-dir change counted as a reversal
                 window_s: float = 1.2,             # events within this window...
                 trip_count: int = 2,               # ...this many -> thrashing
                 gap_s: float = 0.5):               # dt over this resets velocity tracking
        self.csv_path = csv_path
        self.lin_accel_thresh = lin_accel_thresh
        self.ang_accel_thresh = ang_accel_thresh
        self.min_speed = min_speed
        self.min_omega = min_omega
        self.reversal_rad = math.radians(reversal_deg)
        self.window_s = window_s
        self.trip_count = trip_count
        self.gap_s = gap_s

        # Per-sample kinematic trace (list of dicts).
        self._trace: list[dict] = []
        # Detected thrash events.
        self._events: list[dict] = []
        # Rolling list of event timestamps for the window check.
        self._event_times: list[float] = []
        self._tripped = False

        # Previous-sample state for finite differencing.
        self._prev: Optional[dict] = None   # {t, x, y, yaw}
        # Previous velocity for acceleration / reversal detection.
        self._prev_vel: Optional[dict] = None  # {vx, vy, speed, omega}

    # -- public read-only views ------------------------------------------

    @property
    def tripped(self) -> bool:
        """True once thrashing has been detected (latches on)."""
        return self._tripped

    def events(self) -> list[dict]:
        """The detected thrash events (timestamp + type + detail)."""
        return list(self._events)

    def trace(self) -> list[dict]:
        """Every sample with derived kinematics.

        Keys: t, x, y, yaw, vx, vy, speed, omega, lin_acc, ang_acc.
        """
        return list(self._trace)

    # -- core ------------------------------------------------------------

    def feed(self, t: float, x: float, y: float, yaw_rad: float) -> bool:
        """Record one pose sample and detect thrash. Returns :attr:`tripped`.

        Computes the linear velocity vector and angular velocity (deg/s) vs the
        previous sample and their accelerations, flags reversal / flip events,
        and returns True once ``trip_count`` events fall inside ``window_s``.

        Bad samples (None/NaN/inf) are skipped entirely. A non-positive dt or a
        dt larger than ``gap_s`` stores the sample but resets velocity history
        (no velocity/event computed across the gap).
        """
        # Robust to junk: skip the sample outright.
        if not _finite(t, x, y, yaw_rad):
            return self._tripped

        vx = vy = speed = omega = lin_acc = ang_acc = 0.0

        if self._prev is None:
            # First good sample: nothing to difference against yet.
            self._record(t, x, y, yaw_rad, vx, vy, speed, omega, lin_acc, ang_acc)
            self._prev = {"t": t, "x": x, "y": y, "yaw": yaw_rad}
            return self._tripped

        dt = t - self._prev["t"]

        if dt <= 0 or dt > self.gap_s:
            # Time went backward or there was a long gap: don't trust a velocity
            # across it. Store the sample, reset velocity history, no event.
            self._record(t, x, y, yaw_rad, vx, vy, speed, omega, lin_acc, ang_acc)
            self._prev = {"t": t, "x": x, "y": y, "yaw": yaw_rad}
            self._prev_vel = None
            return self._tripped

        # --- derive velocities --------------------------------------------
        vx = (x - self._prev["x"]) / dt              # cm/s
        vy = (y - self._prev["y"]) / dt              # cm/s
        speed = math.hypot(vx, vy)                   # cm/s
        dyaw = _wrap_pi(yaw_rad - self._prev["yaw"])  # rad, shortest path
        omega = math.degrees(dyaw) / dt              # deg/s

        # --- derive accelerations + detect events vs previous velocity -----
        if self._prev_vel is not None:
            pv = self._prev_vel
            # Linear acceleration = magnitude of the velocity-vector change / dt.
            lin_acc = math.hypot(vx - pv["vx"], vy - pv["vy"]) / dt   # cm/s^2
            # Angular acceleration from the omega change.
            ang_acc = abs(omega - pv["omega"]) / dt                  # deg/s^2

            # Linear reversal: both samples actually moving, the velocity
            # direction swung past reversal_deg, and the lurch was hard.
            if (speed > self.min_speed and pv["speed"] > self.min_speed):
                turn = _angle_between(pv["vx"], pv["vy"], vx, vy)
                if turn > self.reversal_rad and lin_acc > self.lin_accel_thresh:
                    self._add_event(t, "linear_reversal", {
                        "dir_change_deg": round(math.degrees(turn), 1),
                        "lin_acc": round(lin_acc, 1),
                        "speed": round(speed, 2),
                        "prev_speed": round(pv["speed"], 2),
                    })

            # Angular flip: omega changed sign, both spins were real, hard snap.
            same_sign = (omega >= 0) == (pv["omega"] >= 0)
            if (not same_sign
                    and abs(omega) > self.min_omega
                    and abs(pv["omega"]) > self.min_omega
                    and ang_acc > self.ang_accel_thresh):
                self._add_event(t, "angular_flip", {
                    "omega": round(omega, 1),
                    "prev_omega": round(pv["omega"], 1),
                    "ang_acc": round(ang_acc, 1),
                })

        # Persist this sample's state for the next call.
        self._record(t, x, y, yaw_rad, vx, vy, speed, omega, lin_acc, ang_acc)
        self._prev = {"t": t, "x": x, "y": y, "yaw": yaw_rad}
        self._prev_vel = {"vx": vx, "vy": vy, "speed": speed, "omega": omega}
        return self._tripped

    # -- internals -------------------------------------------------------

    def _record(self, t, x, y, yaw, vx, vy, speed, omega, lin_acc, ang_acc) -> None:
        """Append one fully-derived sample row to the trace."""
        self._trace.append({
            "t": t, "x": x, "y": y, "yaw": yaw,
            "vx": vx, "vy": vy, "speed": speed, "omega": omega,
            "lin_acc": lin_acc, "ang_acc": ang_acc,
        })

    def _add_event(self, t: float, kind: str, detail: dict) -> None:
        """Record a thrash event and update the rolling window / trip flag."""
        self._events.append({"t": t, "type": kind, "detail": detail})
        self._event_times.append(t)
        # Drop event timestamps older than the window so the count is "recent".
        cutoff = t - self.window_s
        self._event_times = [et for et in self._event_times if et >= cutoff]
        if len(self._event_times) >= self.trip_count:
            self._tripped = True

    # -- reporting -------------------------------------------------------

    def summary(self) -> str:
        """Human-readable analysis of the run for the operator."""
        n = len(self._trace)
        if n == 0:
            return "ThrashMonitor: no samples."

        t0 = self._trace[0]["t"]
        t1 = self._trace[-1]["t"]
        duration = t1 - t0
        peak_lin = max((r["lin_acc"] for r in self._trace), default=0.0)
        peak_ang = max((r["ang_acc"] for r in self._trace), default=0.0)

        lines = []
        lines.append(
            f"ThrashMonitor: {n} samples over {duration:.2f}s, "
            f"{len(self._events)} thrash event(s), "
            f"tripped={self._tripped}"
        )
        lines.append(
            f"  peak linear accel = {peak_lin:.1f} cm/s^2 "
            f"(thresh {self.lin_accel_thresh:.0f})"
        )
        lines.append(
            f"  peak angular accel = {peak_ang:.1f} deg/s^2 "
            f"(thresh {self.ang_accel_thresh:.0f})"
        )
        for i, ev in enumerate(self._events, 1):
            rel = ev["t"] - t0
            # Where in the trace did this happen (sample index by timestamp)?
            idx = self._sample_index_at(ev["t"])
            detail = ", ".join(f"{k}={v}" for k, v in ev["detail"].items())
            lines.append(
                f"  event {i}: t={rel:.2f}s  {ev['type']}  "
                f"[sample ~{idx}/{n}]  {detail}"
            )
        return "\n".join(lines)

    def _sample_index_at(self, t: float) -> int:
        """Index of the trace sample whose timestamp matches ``t`` (else last)."""
        for i, r in enumerate(self._trace):
            if r["t"] == t:
                return i
        return len(self._trace) - 1

    def save(self) -> Optional[str]:
        """Write the full trace + an events section to ``csv_path``.

        Returns the path written, or None if no ``csv_path`` was configured.
        """
        if not self.csv_path:
            return None

        cols = ["t", "x", "y", "yaw", "vx", "vy", "speed",
                "omega", "lin_acc", "ang_acc"]
        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f)
            # --- per-sample kinematics section ---
            w.writerow(cols)
            for r in self._trace:
                w.writerow([f"{r[c]:.4f}" if isinstance(r[c], float) else r[c]
                            for c in cols])
            # --- events section (blank line separator) ---
            w.writerow([])
            w.writerow(["# EVENTS", f"tripped={self._tripped}",
                        f"count={len(self._events)}"])
            w.writerow(["event_t", "type", "detail"])
            for ev in self._events:
                detail = "; ".join(f"{k}={v}" for k, v in ev["detail"].items())
                w.writerow([f"{ev['t']:.4f}", ev["type"], detail])
        return self.csv_path
