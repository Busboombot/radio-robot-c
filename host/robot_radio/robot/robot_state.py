"""RobotState — composite motion-state dataclass for the Nezha robot.

Carries pose, fused body-frame velocity, OTOS acceleration, and a host-side
timestamp as one coherent snapshot.  Built by NezhaState from each TLM frame
that carries both ``pose=`` and ``twist=`` fields.

Unit conventions:
    pose.x / pose.y : centimetres (world frame, inheriting from Pose)
    pose.heading    : radians (CCW-positive, standard maths convention)
    v               : mm/s (body-frame forward speed, from EKF fusedV)
    omega           : rad/s (yaw rate, CCW-positive, from EKF fusedOmega)
    accel           : (ax_mmps2, ay_mmps2) — raw OTOS body-frame acceleration, or None
    stamp           : time.monotonic() seconds at the host when the frame was processed
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_radio.nav.pose import Pose


@dataclass(frozen=True)
class RobotState:
    """Composite frozen motion state for one TLM frame.

    Parameters
    ----------
    pose:
        Robot position and heading in world frame.  x/y in centimetres;
        heading in radians (CCW-positive).
    v:
        Body-frame forward speed in mm/s (EKF-fused).
    omega:
        Yaw rate in rad/s, CCW-positive (EKF-fused).
    accel:
        Body-frame linear acceleration as (ax_mmps2, ay_mmps2), or None when
        the ``twist=`` field is absent or the frame carries no accel data.
    stamp:
        ``time.monotonic()`` seconds at the host when this state was built.
    """

    pose: Pose
    v: float
    omega: float
    accel: tuple[float, float] | None
    stamp: float
