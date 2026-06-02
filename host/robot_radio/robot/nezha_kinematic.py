"""NezhaKinematic — differential-drive kinematic model for the Nezha robot.

Extends NezhaState with:
- (vx, omega) unicycle inputs converted to wheel speeds via DifferentialDriveKinematics
- WPILib DifferentialDriveOdometry for pose tracking (encoder deltas + OTOS yaw)
- World-frame pose properties in project conventions (CCW-positive yaw, mm/cm distances)
- anchor() to reset odometry to a camera-supplied world pose

Coordinate conventions:
- Distances: mm internally; cm in public world_pos_cm; m in WPILib calls
- Yaw: CCW-positive radians in all public API. 0 = +X world axis, matching WPILib's
  Rotation2d and aprilcam.Tag.orientation conventions.

Usage (synchronous):
    k = NezhaKinematic(conn, trackwidth_mm=126)
    k.anchor((50.0, 30.0), 0.0)      # set world origin from camera
    k.update(vx=0.2, omega=0.0)      # drive forward at 0.2 m/s, read sensors
    print(k.world_pos_cm, k.world_yaw)

Usage (async):
    k.start_async()
    k.wheel_speeds = [100, 100]      # or set via update(vx=...)
    time.sleep(1.0)
    print(k.world_pos_cm)
    k.stop_async()
"""

from __future__ import annotations

import math
import time
from typing import Callable

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import DifferentialDriveOdometry

from robot_radio.kinematics.differential_drive import DifferentialDriveKinematics
from robot_radio.robot.nezha_state import NezhaState
from robot_radio.robot.protocol import NezhaProtocol


def _wrap_angle(a: float) -> float:
    """Normalise angle *a* to ``[-π, π]``."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class NezhaKinematic(NezhaState):
    """Kinematic model layered on top of NezhaState.

    Args:
        proto: NezhaProtocol wrapping the serial connection.
        trackwidth_mm: Distance between wheel contact patches in mm.
        wheel_diameter_mm: Wheel diameter in mm. Reserved for future encoder
            calibration use; the firmware already reports encoder values in mm,
            so this parameter is not used in the current delta calculation.
    """

    def __init__(
        self,
        proto: NezhaProtocol,
        trackwidth_mm: float,
        wheel_diameter_mm: float | None = None,
    ) -> None:
        super().__init__(proto)
        self._kinematics = DifferentialDriveKinematics(trackwidth_mm)
        self._trackwidth_m = trackwidth_mm / 1000.0
        self._wheel_diameter_mm = wheel_diameter_mm  # stored but not needed currently

        self._odometry = DifferentialDriveOdometry(Rotation2d(0), 0.0, 0.0)
        self._prev_encoders: tuple[int, int] = (0, 0)
        # Cumulative encoder offset applied at anchor() so we can pass running totals
        # to WPILib (which expects cumulative distances) while resetting pose origin.
        self._encoder_offset_m: tuple[float, float] = (0.0, 0.0)
        self._velocity: float = 0.0
        self._angular_velocity: float = 0.0

    # ------------------------------------------------------------------
    # Update cycle override
    # ------------------------------------------------------------------

    def update(self, vx: float | None = None, omega: float | None = None) -> None:  # type: ignore[override]
        """Send wheel speeds and update odometry.

        Args:
            vx: Forward speed in m/s (None = keep current wheel speeds).
            omega: Angular velocity in CCW-positive rad/s (None = keep current).

        When either vx or omega is provided, both are used (defaulting the
        other to 0.0) and converted to wheel speeds via DifferentialDriveKinematics.
        When neither is provided, the current wheel_speeds are re-sent unchanged
        (keepalive behaviour inherited from NezhaState).
        """
        if vx is not None or omega is not None:
            vx = vx or 0.0
            omega = omega or 0.0
            left_mms, right_mms = self._kinematics.inverse(vx, omega)
            super().update(int(left_mms), int(right_mms))
        else:
            super().update()
        self._update_odometry()

    # ------------------------------------------------------------------
    # Odometry
    # ------------------------------------------------------------------

    def _update_odometry(self) -> None:
        """Update WPILib odometry with current encoder totals and OTOS heading.

        WPILib DifferentialDriveOdometry.update() takes cumulative distances
        (total traveled since odometry was initialised), not per-cycle deltas.
        We pass the raw cumulative encoder values converted to metres, then
        subtract an offset that is reset each time anchor() is called.
        Velocity is derived from the per-cycle encoder delta.
        """
        with self._lock:
            enc = self.encoders
            otos = self.otos_pose
            dt = self.dt_s

        # Cumulative mm → cumulative metres relative to last anchor
        left_total_m = enc[0] / 1000.0 - self._encoder_offset_m[0]
        right_total_m = enc[1] / 1000.0 - self._encoder_offset_m[1]

        # Encoder delta for velocity computation
        left_delta_m = (enc[0] - self._prev_encoders[0]) / 1000.0
        right_delta_m = (enc[1] - self._prev_encoders[1]) / 1000.0
        self._prev_encoders = enc

        # OTOS yaw is CCW-positive degrees; WPILib Rotation2d needs CCW-positive radians
        heading = Rotation2d(math.radians(otos[2]))
        self._odometry.update(heading, left_total_m, right_total_m)

        if dt > 0:
            self._velocity = (left_delta_m + right_delta_m) / (2.0 * dt)
            # CCW-positive angular velocity, matching WPILib convention
            omega_ccw = (right_delta_m - left_delta_m) / (self._trackwidth_m * dt)
            self._angular_velocity = omega_ccw

    # ------------------------------------------------------------------
    # Pose anchor
    # ------------------------------------------------------------------

    def anchor(
        self,
        world_pos_cm: tuple[float, float],
        world_yaw_rad: float,
    ) -> None:
        """Reset odometry and OTOS to a camera-supplied world pose.

        Zeros the OTOS position sensor, writes the world pose back via
        set_world_pose(), and reinitialises the WPILib odometry at the same
        pose so encoder-based tracking picks up from the right origin.

        Args:
            world_pos_cm: (x_cm, y_cm) world position from camera.
            world_yaw_rad: Heading in CCW-positive radians (0 = +X).
        """
        self.zero_otos()

        x_mm = world_pos_cm[0] * 10.0
        y_mm = world_pos_cm[1] * 10.0
        # CCW radians → CCW degrees for set_world_pose (OTOS convention)
        h_deg = math.degrees(world_yaw_rad)
        self.set_world_pose(x_mm, y_mm, h_deg)

        # WPILib Pose2d uses metres and CCW-positive rotation
        init_pose = Pose2d(
            Translation2d(world_pos_cm[0] / 100.0, world_pos_cm[1] / 100.0),
            Rotation2d(world_yaw_rad),
        )
        self._odometry = DifferentialDriveOdometry(
            Rotation2d(world_yaw_rad),
            0.0,
            0.0,
            init_pose,
        )

        with self._lock:
            enc = self.encoders
        self._prev_encoders = enc
        # Record the current raw encoder total as the zero-point for this
        # odometry session; _update_odometry subtracts this offset so that
        # WPILib always receives distances relative to the anchor point.
        self._encoder_offset_m = (enc[0] / 1000.0, enc[1] / 1000.0)

    # ------------------------------------------------------------------
    # Public pose properties
    # ------------------------------------------------------------------

    @property
    def pose(self) -> Pose2d:
        """Current odometry pose as a WPILib Pose2d (metres, CCW-positive)."""
        return self._odometry.getPose()

    @property
    def world_pos_cm(self) -> tuple[float, float]:
        """Current world position as (x_cm, y_cm)."""
        p = self._odometry.getPose().translation()
        return (p.x * 100.0, p.y * 100.0)

    @property
    def world_yaw(self) -> float:
        """Current heading in CCW-positive radians (0 = +X world axis)."""
        return self._odometry.getPose().rotation().radians()

    # ------------------------------------------------------------------
    # Velocity properties
    # ------------------------------------------------------------------

    @property
    def velocity(self) -> float:
        """Forward speed in m/s derived from the most recent encoder delta."""
        return self._velocity

    @property
    def angular_velocity(self) -> float:
        """Angular velocity in CCW-positive rad/s derived from encoder deltas."""
        return self._angular_velocity

    # ------------------------------------------------------------------
    # World-frame motion primitives
    # ------------------------------------------------------------------

    def go_to_world(
        self,
        target_cm: tuple[float, float],
        *,
        speed_mms: int = 200,
        timeout_s: float = 15.0,
        on_tick: Callable[[], None] | None = None,
    ) -> str:
        """Drive to a world-frame position using the firmware G arc command.

        Converts *target_cm* from world XY (cm) to robot-relative mm using the
        current world pose, then sends ``self._proto.go_to(dx_mm, dy_mm,
        speed_mms)`` and polls for the firmware completion token.

        Coordinate convention (CCW-positive yaw, 0 = +X world axis):
            dx_robot =  dx_world * cos(yaw) + dy_world * sin(yaw)
            dy_robot = -dx_world * sin(yaw) + dy_world * cos(yaw)

        Args:
            target_cm: Target position in world frame as (x_cm, y_cm).
            speed_mms: Drive speed in mm/s (default 200).
            timeout_s: Host-side wall-clock timeout in seconds (default 15.0).
            on_tick: Optional zero-argument callable invoked once per poll cycle.

        Returns:
            ``"DONE"`` when the firmware reports ``G+DONE``,
            ``"TIMEOUT"`` when the firmware reports ``G+TIMEOUT``,
            ``"HOST_TIMEOUT"`` if the host deadline expires first.
        """
        cur_x_cm, cur_y_cm = self.world_pos_cm
        yaw = self.world_yaw

        dx_w = (target_cm[0] - cur_x_cm) * 10.0  # cm → mm
        dy_w = (target_cm[1] - cur_y_cm) * 10.0

        dx_robot = dx_w * math.cos(yaw) + dy_w * math.sin(yaw)
        dy_robot = -dx_w * math.sin(yaw) + dy_w * math.cos(yaw)

        self._proto.go_to(int(round(dx_robot)), int(round(dy_robot)), int(speed_mms))

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            lines = self._proto.read_lines(50)
            for line in lines:
                # v2 protocol: EVT done G / EVT safety_stop
                if "EVT done G" in line:
                    self.update()
                    if on_tick:
                        on_tick()
                    return "DONE"
                if "EVT safety_stop" in line:
                    self.update()
                    if on_tick:
                        on_tick()
                    return "TIMEOUT"
            self.update()
            if on_tick:
                on_tick()

        return "HOST_TIMEOUT"

    def turn_to_heading(
        self,
        target_yaw_rad: float,
        *,
        tol_rad: float = math.radians(3.0),
        max_omega: float = 1.0,
        kp: float = 2.5,
        timeout_s: float = 8.0,
        tick_hz: float = 20.0,
        on_tick: Callable[[], None] | None = None,
    ) -> bool:
        """Proportional closed-loop turn to a world-frame heading.

        Uses ``self.update(vx=0, omega)`` each tick to rotate in place until
        the yaw error is within *tol_rad*.

        Coordinate convention: CCW-positive radians, 0 = +X world axis.

        Args:
            target_yaw_rad: Desired heading in CCW-positive radians.
            tol_rad: Convergence tolerance in radians (default 3°).
            max_omega: Clamp magnitude on angular velocity command (default 1.0 rad/s).
            kp: Proportional gain (default 2.5).
            timeout_s: Wall-clock timeout in seconds (default 8.0).
            tick_hz: Control loop rate in Hz (default 20).
            on_tick: Optional zero-argument callable invoked once per control tick.

        Returns:
            ``True`` if the heading converged within *tol_rad*, ``False`` if
            the timeout expired first.
        """
        tick_s = 1.0 / tick_hz
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            err = _wrap_angle(target_yaw_rad - self.world_yaw)
            if abs(err) < tol_rad:
                self.update(0, 0)
                if on_tick:
                    on_tick()
                return True

            omega = max(-max_omega, min(max_omega, kp * err))
            self.update(0, omega)
            if on_tick:
                on_tick()
            time.sleep(tick_s)

        self.update(0, 0)
        return False
