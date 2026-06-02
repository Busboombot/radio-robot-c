"""Navigator — autonomous navigation fusing Robot commands with camera feedback."""

import json
import math
import os
import time
from typing import Any

from aprilcam import Camera, Playfield, Tag

from robot_radio.controllers.pid import PID, normalize_angle
from robot_radio.nav.nav_params import NavParams
from robot_radio.sensors.odometry import Odometry
from robot_radio.nav.pose import Pose, Waypoint, heading_error
from robot_radio.controllers import CONTROLLERS
from robot_radio.path.builder import build_path
import robot_radio.io.preview as _preview_mod
from robot_radio.nav._approach_utils import (
    choose_phase,
    compute_far_command,
    load_approach_calibration,
)

def _build_controller(ctrl_cls, params, **kwargs):
    """Instantiate a controller class from NavParams with optional overrides.

    Parameters are read from *params* and may be overridden via *kwargs*.
    ``PurePursuitTracker`` requires a placeholder path at construction; callers
    must call ``set_path`` before ``compute``.

    Returns
    -------
    Controller
        A concrete controller instance.
    """
    import math as _math  # noqa: PLC0415
    from robot_radio.controllers import PurePursuitTracker, StanleyController  # noqa: PLC0415

    if ctrl_cls is PurePursuitTracker:
        lookahead = kwargs.get("lookahead", getattr(params, "lookahead", 15.0))
        trackwidth = kwargs.get("trackwidth", getattr(params, "trackwidth", 9.0))
        base_speed = kwargs.get("base_speed", getattr(params, "base_speed", 40.0))
        stop_dist = kwargs.get("stop_dist", getattr(params, "stop_dist", 5.0))
        placeholder: list[tuple[float, float]] = [(0.0, 0.0), (1.0, 0.0)]
        return PurePursuitTracker(
            path=placeholder,
            lookahead=float(lookahead),
            trackwidth=float(trackwidth),
            base_speed=float(base_speed),
            stop_dist=float(stop_dist),
        )

    if ctrl_cls is StanleyController:
        k = kwargs.get("k", getattr(params, "stanley_k", 0.8))
        v_soft = kwargs.get("v_soft", getattr(params, "stanley_v_soft", 0.1))
        omega_gain = kwargs.get("omega_gain", getattr(params, "stanley_omega_gain", 2.0))
        goal_tolerance = kwargs.get("goal_tolerance", getattr(params, "stop_dist", 9.0))
        base_speed = kwargs.get("base_speed", getattr(params, "base_speed", 40.0))
        trackwidth = kwargs.get("trackwidth", getattr(params, "trackwidth", 9.0))
        max_delta = kwargs.get(
            "max_delta", getattr(params, "stanley_max_delta", _math.pi / 2)
        )
        return StanleyController(
            k=float(k),
            v_soft=float(v_soft),
            omega_gain=float(omega_gain),
            goal_tolerance=float(goal_tolerance),
            base_speed=float(base_speed),
            trackwidth=float(trackwidth),
            max_delta=float(max_delta),
        )

    # Generic fallback: pass no args; caller must configure the controller
    return ctrl_cls()


def _load_motor_deadband() -> int:
    try:
        from robot_radio.config.robot_config import get_robot_config
        cfg = get_robot_config()
        if cfg is not None and cfg.motor_deadband is not None:
            return int(cfg.motor_deadband)
    except Exception:
        pass
    return 35

MOTOR_DEADBAND = _load_motor_deadband()


def log_record(file_path: str, record: dict[str, Any]) -> dict[str, Any]:
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "a") as f:
        f.write(json.dumps(record) + "\n")
    with open(file_path) as f:
        count = sum(1 for _ in f)
    return {"written": True, "file": file_path, "total_records": count}


class ChaseController:
    """Dual-PID controller: one for speed (distance), one for steering (angle)."""

    def __init__(self, speed_pid: PID, steer_pid: PID, params: dict[str, float]):
        self.speed_pid = speed_pid
        self.steer_pid = steer_pid
        self.p = params

    @property
    def max_speed(self): return self.p["max_speed"]
    @property
    def stop_dist(self): return self.p["stop_dist"]
    @property
    def TURN_THRESHOLD(self): return math.radians(self.p["turn_threshold"])
    @property
    def CRAWL_DIST(self): return self.p["crawl_dist"]
    @property
    def CRAWL_CM_PER_PULSE(self): return self.p["crawl_cm_per_pulse"]
    @property
    def CRAWL_DEG_PER_PULSE(self): return self.p["crawl_deg_per_pulse"]
    @property
    def CRAWL_SPEED(self): return int(self.p["crawl_speed"])
    @property
    def CRAWL_MS(self): return int(self.p["crawl_ms"])

    def compute(self, robot_pos, robot_yaw, target_pos, now,
                forward_only=False):
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.stop_dist:
            self.speed_pid.reset()
            self.steer_pid.reset()
            return 0, 0

        bearing = math.atan2(dy, dx)
        angle_err = normalize_angle(bearing - robot_yaw)

        reversing = False
        if not forward_only and abs(angle_err) > math.pi / 2:
            reverse_bearing = normalize_angle(bearing - math.pi)
            angle_err = normalize_angle(reverse_bearing - robot_yaw)
            reversing = True

        # SPIN when heading is off by more than 45°
        # Proportional speed: faster for large errors, but always above deadband
        if abs(angle_err) > math.radians(45):
            self.speed_pid.reset()
            abs_err_deg = abs(math.degrees(angle_err))
            if abs_err_deg > 90:
                turn_speed = 60
            elif abs_err_deg > 60:
                turn_speed = 55
            else:
                turn_speed = 50
            sign = 1 if angle_err > 0 else -1
            return sign * turn_speed, -sign * turn_speed

        # DRIVE with proportional steering (P-only, no I/D)
        base = self.speed_pid.update(dist, now)
        # Clamp base above motor deadband so the robot actually moves
        base = max(MOTOR_DEADBAND + 5, min(self.max_speed, base))

        if reversing:
            base = -base

        steer = self.p["steer_kp"] * angle_err

        left = base + steer
        right = base - steer

        return max(-100, min(100, left)), max(-100, min(100, right))

    def compute_crawl(self, robot_pos, robot_yaw, target_pos):
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.stop_dist:
            return "arrived", 0, 0, 0

        bearing = math.atan2(dy, dx)
        angle_err = normalize_angle(bearing - robot_yaw)

        spd = self.CRAWL_SPEED

        # Large angle: pure spin
        if abs(angle_err) > math.radians(60):
            count = max(1, min(int(self.p["crawl_spin_max"]),
                               int(abs(math.degrees(angle_err)) / self.CRAWL_DEG_PER_PULSE)))
            if angle_err > 0:
                return "spin", spd, -spd, count
            else:
                return "spin", -spd, spd, count

        # Steered crawl: forward with differential
        steer_gain = 0.8
        steer = max(-1.0, min(1.0, angle_err / math.radians(60)))
        steer_amount = steer * steer_gain * spd

        left = int(spd + steer_amount)
        right = int(spd - steer_amount)
        left = max(-100, min(100, left))
        right = max(-100, min(100, right))

        count = max(1, int(dist / self.CRAWL_CM_PER_PULSE))
        count = min(count, int(self.p["crawl_fwd_max"]))

        return "steer", left, right, count


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_DATA_DIR = os.path.join(_SCRIPT_DIR, "..", "data")


class Navigator:
    """Autonomous navigation using a Robot + camera position source.

    ## approach()

    ``approach(target_xy, tolerance_mm=5, timeout=20.0)`` drives the robot
    tag to a world-coordinate target using a two-phase closed-loop controller:

    - **Far phase** (r > 100 mm): issue a single calibrated ``speed_for_time``
      command, re-read pose, repeat.
    - **Near phase** (r ≤ 100 mm): issue short crawl pulses and re-read pose
      after each pulse.
    - **Done** (r ≤ tolerance_mm): stop and return.

    Returns a dict with keys:
    ``success``, ``elapsed_s``, ``final_error_mm``, ``phases_used``,
    ``n_far_commands``, ``n_crawl_pulses``.
    """

    def __init__(self, robot, params: NavParams | None = None,
                 data_dir: str | None = None, otos=None):
        self._robot = robot
        self.params = params or NavParams()
        self._otos = otos  # optional Otos instance for dead-reckoning fallback
        self._field: Playfield | None = None
        self._camera: Camera | None = None
        self._camera_index: int | None = None

        # Load approach calibration once at construction time.
        _data = data_dir or _DEFAULT_DATA_DIR
        self._approach_cal = load_approach_calibration(
            linear_path=os.path.join(_data, "linear_calibration.json"),
            crawl_path=os.path.join(_data, "crawl_calibration.json"),
        )

    # -- Playfield management --

    def _get_playfield(self, camera_index: int) -> Playfield:
        """Get or create the Playfield for the given camera."""
        if self._field is not None and self._camera_index == camera_index:
            return self._field
        # Stop existing playfield if switching cameras
        if self._field is not None:
            self._field.stop()
            self._camera.close()
        self._camera = Camera(camera_index)
        self._field = Playfield(self._camera, width_cm=101.0, height_cm=89.0)
        self._field.start()
        self._camera_index = camera_index
        return self._field

    def reset_camera(self) -> dict[str, Any]:
        """Stop the playfield and release the camera."""
        if self._field is None:
            return {"reset": False, "reason": "not_initialized"}
        self._field.stop()
        if self._camera:
            self._camera.close()
        self._field = None
        self._camera = None
        self._camera_index = None
        return {"reset": True}

    def status(self) -> dict[str, Any]:
        """Introspection dict for MCP `status` / CLI debugging."""
        return {
            "nav_module": self.__class__.__module__,
            "camera_active": self._field is not None,
            "camera_index": self._camera_index,
        }

    def get_next_tags(self, camera_index: int = 3) -> list:
        field = self._get_playfield(camera_index)
        return list(field.tags().values())

    # -- Core navigation --

    def navigate(self, target_xy, camera_index=3, robot_tag=1,
                 timeout=30.0, forward_only=False) -> dict[str, Any]:
        """Run the dual-PID navigation loop. Blocks until arrived or timed out."""
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        p = self.params.as_dict()

        try:
            field = self._get_playfield(camera_index)

            speed_pid = PID(kp=p["speed_kp"], ki=p["speed_ki"], kd=p["speed_kd"],
                            out_min=0, out_max=p["max_speed"])
            steer_pid = PID(kp=p["steer_kp"], ki=p["steer_ki"], kd=p["steer_kd"],
                            out_min=-40, out_max=40)
            ctrl = ChaseController(speed_pid, steer_pid, p)

            nav_hz = p.get("nav_loop_hz", 15)
            cmd_ms = int(p.get("cmd_duration_ms", 150))
            frames_stop = int(p.get("frames_before_stop", 5))
            period = 1.0 / nav_hz
            start = time.monotonic()
            frames_no_robot = 0
            backwards_recoveries = 0
            max_recoveries = 3
            last_dist = None
            first_tag_ms = None
            first_motion_ms = None
            log_lines: list[str] = []
            frame_count = 0

            for tags in field.stream():
                t0 = time.monotonic()
                elapsed = t0 - start
                frame_count += 1

                if elapsed > timeout:
                    self._robot.stop()
                    return {"success": False, "reason": "timeout",
                            "elapsed": round(elapsed, 1),
                            "last_dist": round(last_dist, 1) if last_dist else None,
                            "frames": frame_count,
                            "fps": round(frame_count / elapsed, 1),
                            "log": log_lines[-20:]}

                robot_pos, robot_yaw = None, None
                for t in tags:
                    if t.id == robot_tag and t.wx is not None:
                        if t.age > 0.3:
                            continue
                        robot_pos = (t.wx, t.wy)
                        robot_yaw = t.orientation

                if robot_pos is None:
                    frames_no_robot += 1
                    if frames_no_robot >= frames_stop:
                        self._robot.stop()
                    if frames_no_robot > nav_hz * 2:
                        if backwards_recoveries >= max_recoveries:
                            return {"success": False, "reason": "lost_robot_tag",
                                    "elapsed": round(elapsed, 1),
                                    "last_dist": round(last_dist, 1) if last_dist else None,
                                    "frames": frame_count,
                                    "log": log_lines[-20:]}
                        log_lines.append("lost tag — driving backwards to recover")
                        self._robot.speed_for_time(-30, -30, 300)
                        self._robot.stop()
                        frames_no_robot = 0
                        backwards_recoveries += 1
                    continue
                else:
                    frames_no_robot = 0
                    if first_tag_ms is None:
                        first_tag_ms = int(elapsed * 1000)

                dx = target_xy[0] - robot_pos[0]
                dy = target_xy[1] - robot_pos[1]
                dist = math.sqrt(dx * dx + dy * dy)
                last_dist = dist

                # Check if arrived
                if dist <= ctrl.stop_dist:
                    self._robot.stop()
                    return {"success": True,
                            "final_dist": round(dist, 1),
                            "final_pos": [round(robot_pos[0], 1), round(robot_pos[1], 1)],
                            "elapsed": round(elapsed, 1),
                            "frames": frame_count,
                            "fps": round(frame_count / elapsed, 1) if elapsed > 0 else 0,
                            "log": log_lines[-20:]}

                # CRAWL MODE
                if dist < ctrl.CRAWL_DIST:
                    self._robot.stop()
                    ctype, cl, cr, ccount = ctrl.compute_crawl(
                        robot_pos, robot_yaw, target_xy)
                    if ctype == "arrived":
                        return {"success": True,
                                "final_dist": round(dist, 1),
                                "final_pos": [round(robot_pos[0], 1), round(robot_pos[1], 1)],
                                "elapsed": round(elapsed, 1),
                                "frames": frame_count,
                                "fps": round(frame_count / elapsed, 1) if elapsed > 0 else 0,
                                "log": log_lines[-20:]}
                    self._robot.speed_for_time(cl, cr, ctrl.CRAWL_MS * ccount)
                    status = (f"d={dist:.1f}cm CW {ctype} "
                              f"L={cl:+d} R={cr:+d} x{ccount} t={elapsed:.1f}s")
                    log_lines.append(status)
                    if first_motion_ms is None:
                        first_motion_ms = int(elapsed * 1000)
                    continue

                # PID MODE
                left, right = ctrl.compute(robot_pos, robot_yaw, target_xy, t0,
                                            forward_only=forward_only)

                if left == 0 and right == 0:
                    continue

                self._robot.speed_for_time(int(left), int(right), cmd_ms)
                if first_motion_ms is None and (int(left) != 0 or int(right) != 0):
                    first_motion_ms = int(elapsed * 1000)

                status = (f"d={dist:.1f}cm "
                          f"L={left:+.0f} R={right:+.0f} t={elapsed:.1f}s")
                log_lines.append(status)

                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)

        except Exception as exc:
            self._robot.stop()
            return {"error": str(exc)}

    def _run_controller(
        self,
        tracker: Any,
        field,
        odom: Odometry,
        timeout: float,
        start_time: float,
        initial_frame_count: int = 0,
    ) -> dict[str, Any]:
        """Run the path-following control loop until arrival, timeout, or tag loss.

        This private helper is shared by ``follow_path`` and
        ``follow_pose_path`` so they both use identical controller semantics.
        The *tracker* argument must satisfy the ``PathFollower`` protocol — any
        object returned by ``make_controller`` qualifies.

        Parameters
        ----------
        tracker:
            A ``PathFollower`` instance (e.g. ``PurePursuitTracker`` or
            ``StanleyController``).  Must already have its path loaded via
            ``set_path``.
        field:
            Active ``Playfield`` already started.
        odom:
            ``Odometry`` instance bound to *field*.
        timeout:
            Absolute wall-clock deadline measured from *start_time*.
        start_time:
            ``time.monotonic()`` value captured before this call — used for
            elapsed calculations and timeout checking.
        initial_frame_count:
            Frame counter value at entry (e.g. frames consumed by spin-align).

        Returns
        -------
        dict
            ``{"success": True, "final_pos": [...], "elapsed": float,
            "frames": int}`` on arrival, or an error/timeout dict.
            Motors are stopped by this method on every exit path.
        """
        p = self.params.as_dict()
        nav_hz = p.get("nav_loop_hz", 15)
        cmd_ms = int(p.get("cmd_duration_ms", 150))
        frames_stop = int(p.get("frames_before_stop", 5))

        frame_count = initial_frame_count
        frames_no_robot = 0

        for tags in field.stream():
            frame_count += 1
            elapsed = time.monotonic() - start_time

            if elapsed > timeout:
                self._robot.stop()
                return {
                    "success": False,
                    "reason": "timeout",
                    "elapsed": round(elapsed, 1),
                    "frames": frame_count,
                }

            odom.update(tags)
            if not odom.is_valid:
                frames_no_robot += 1
                if frames_no_robot >= frames_stop:
                    self._robot.stop()
                if frames_no_robot > nav_hz * 2:
                    return {
                        "success": False,
                        "reason": "lost_robot_tag",
                        "elapsed": round(elapsed, 1),
                        "frames": frame_count,
                    }
                continue

            frames_no_robot = 0
            left, right = tracker.compute((odom.x, odom.y), odom.yaw)

            if left == 0.0 and right == 0.0:
                self._robot.stop()
                return {
                    "success": True,
                    "final_pos": [round(odom.x, 1), round(odom.y, 1)],
                    "elapsed": round(elapsed, 1),
                    "frames": frame_count,
                }

            self._robot.speed_for_time(int(left), int(right), cmd_ms)

        # Stream exhausted (shouldn't happen in normal operation)
        self._robot.stop()
        return {
            "success": False,
            "reason": "stream_exhausted",
            "elapsed": round(time.monotonic() - start_time, 1),
            "frames": frame_count,
        }

    def follow_path(
        self,
        path: list[tuple[float, float]],
        camera_index: int = 3,
        robot_tag: int = 1,
        timeout: float = 30.0,
        lookahead: float = 15.0,
        trackwidth: float = 9.0,
        base_speed: float = 40.0,
        stop_dist: float = 5.0,
        controller: str = "pure_pursuit",
    ) -> dict[str, Any]:
        """Follow a multi-waypoint path using the selected path-following controller.

        Uses ``Odometry`` for camera-based pose reading and the controller
        selected by *controller* for differential-drive wheel commands.

        The loop iterates over frames from the playfield stream.  Each
        iteration:

        1. ``odom.update(tags)`` refreshes the robot pose from the current
           frame (avoids a second camera fetch).
        2. ``ctrl.compute(pos, yaw)`` returns ``(left, right)`` motor
           commands.  A ``(0.0, 0.0)`` sentinel means the robot has arrived
           within ``stop_dist`` of the final waypoint.
        3. ``speed_for_time(left, right, cmd_ms)`` drives the motors for the
           configured command duration.

        Parameters
        ----------
        path:
            Ordered list of ``(x, y)`` world-coordinate waypoints in cm.
            Must contain at least two points.
        camera_index:
            Camera index to use for AprilTag detection (default 3 — B&W).
        robot_tag:
            AprilTag ID mounted on the robot (default 1).
        timeout:
            Maximum wall-clock seconds before giving up.
        lookahead:
            Lookahead circle radius in cm (default 15.0); used by
            ``pure_pursuit``.
        trackwidth:
            Wheel-to-wheel spacing in cm (default 9.0 for QBot Pro).
        base_speed:
            Nominal forward motor command 0-100 (default 40.0).
        stop_dist:
            Distance from final waypoint at which arrival is declared,
            in cm (default 5.0).
        controller:
            Controller name to use.  Accepted values: ``"pure_pursuit"``
            (default), ``"stanley"``.  The controller is instantiated via
            ``make_controller(controller, self.params)``.

        Returns
        -------
        dict
            On arrival::

                {"success": True, "final_pos": [x, y],
                 "elapsed": float, "frames": int}

            On timeout::

                {"success": False, "reason": "timeout", "elapsed": float,
                 "frames": int}

            When robot tag is lost for too long::

                {"success": False, "reason": "lost_robot_tag",
                 "elapsed": float, "frames": int}

            On exception::

                {"error": str}

        Motors are stopped on every exit path (success, timeout, or
        exception).
        """
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        try:
            field = self._get_playfield(camera_index)
            odom = Odometry(field, robot_tag, otos=self._otos, params=self.params)
            ctrl_cls = CONTROLLERS.get(controller)
            if ctrl_cls is None:
                raise ValueError(
                    f"Unknown controller {controller!r}. "
                    f"Supported values: {list(CONTROLLERS)}"
                )
            ctrl = _build_controller(ctrl_cls, self.params,
                                     lookahead=lookahead,
                                     trackwidth=trackwidth,
                                     base_speed=base_speed,
                                     stop_dist=stop_dist)
            ctrl.set_path(path)
            start = time.monotonic()
            return self._run_controller(ctrl, field, odom, timeout, start)

        except Exception as exc:
            self._robot.stop()
            return {"error": str(exc)}

    def _spin_to_heading(
        self,
        target_heading_rad: float,
        field,
        odom: Odometry,
        tolerance_deg: float,
        max_frames: int,
        speed: float,
    ) -> dict[str, Any]:
        """Rotate in place until heading error is within *tolerance_deg*.

        Sends ``speed_for_time`` spin commands each frame and re-reads yaw
        via *odom*.  Returns after the first frame where the error is within
        tolerance, or after *max_frames* attempts.

        Parameters
        ----------
        target_heading_rad:
            Desired heading in radians (standard math convention).
        field:
            Active ``Playfield``.
        odom:
            ``Odometry`` bound to *field*.
        tolerance_deg:
            Stop spinning when ``|error| < tolerance_deg``.
        max_frames:
            Maximum camera frames to consume before giving up.
        speed:
            Motor speed magnitude for in-place spin (0-100).

        Returns
        -------
        dict
            ``{"aligned": bool, "heading_error_deg": float, "frames": int}``
        """
        p = self.params.as_dict()
        cmd_ms = int(p.get("cmd_duration_ms", 150))
        frames_stop = int(p.get("frames_before_stop", 5))

        frame_count = 0
        frames_no_robot = 0
        tol_rad = math.radians(tolerance_deg)
        spd = int(speed)

        for tags in field.stream():
            frame_count += 1
            if frame_count > max_frames:
                break

            odom.update(tags)
            if not odom.is_valid:
                frames_no_robot += 1
                if frames_no_robot >= frames_stop:
                    self._robot.stop()
                continue

            frames_no_robot = 0
            err = heading_error(odom.yaw, target_heading_rad)

            if abs(err) < tol_rad:
                self._robot.stop()
                return {
                    "aligned": True,
                    "heading_error_deg": round(math.degrees(err), 2),
                    "frames": frame_count,
                }

            # Spin: positive error → turn left (CCW) → left=-spd, right=+spd
            if err > 0:
                self._robot.speed_for_time(-spd, spd, cmd_ms)
            else:
                self._robot.speed_for_time(spd, -spd, cmd_ms)

        self._robot.stop()
        # Read final heading for reporting
        err_final = 0.0
        if odom.is_valid:
            err_final = heading_error(odom.yaw, target_heading_rad)
        return {
            "aligned": False,
            "heading_error_deg": round(math.degrees(err_final), 2),
            "frames": frame_count,
        }

    def follow_pose_path(
        self,
        end_pose: Pose,
        start_pose: Pose | None = None,
        waypoints: list[Waypoint] | None = None,
        method: str = "bezier",
        preview: bool = True,
        camera_index: int = 3,
        robot_tag: int = 1,
        timeout: float = 30.0,
        lookahead: float = 15.0,
        trackwidth: float = 9.0,
        base_speed: float = 40.0,
        stop_dist: float = 5.0,
        controller: str = "pure_pursuit",
    ) -> dict[str, Any]:
        """Plan a curved path and drive to the end pose in three phases.

        **Phase 1 — Spin-align:** If the heading error between the robot's
        current yaw and the initial path tangent exceeds
        ``params.spin_align_threshold_deg``, rotate in place until the error
        is below ``params.spin_align_tolerance_deg``.

        **Phase 2 — Path following:** Track the sampled path polyline using
        the controller selected by *controller* until within *stop_dist* of
        the final waypoint.

        **Phase 3 — Final turn:** Rotate in place until the heading error to
        ``end_pose.heading`` is below ``params.final_turn_tolerance_deg``.

        Parameters
        ----------
        end_pose:
            Target pose (position + heading) the robot should reach.
        start_pose:
            Starting pose.  When ``None``, the current pose is read from the
            camera.  If the camera cannot locate the robot tag, an error dict
            is returned.
        waypoints:
            Intermediate ``Waypoint`` objects (optional).
        method:
            Path builder name, default ``"bezier"``.
        preview:
            When ``True``, log/preview the planned polyline via
            ``preview_polyline`` before driving.
        camera_index:
            Camera index (default 3, B&W).
        robot_tag:
            AprilTag ID on the robot (default 1).
        timeout:
            Total wall-clock deadline in seconds.
        lookahead:
            Lookahead radius in cm; used by ``pure_pursuit``.
        trackwidth:
            Wheel-to-wheel spacing in cm.
        base_speed:
            Nominal forward motor command (0-100).
        stop_dist:
            Arrival threshold in cm.
        controller:
            Controller name to use for path tracking.  Accepted values:
            ``"pure_pursuit"`` (default), ``"stanley"``.  Instantiated via
            ``make_controller(controller, self.params)``.

        Returns
        -------
        dict
            ``{success, planned_path, traversed_frames, elapsed_s,
            final_pose, final_heading_error_deg}`` on success or
            ``{..., error}`` on failure.
        """
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        p = self.params.as_dict()
        spin_align_threshold_deg = p.get("spin_align_threshold_deg", 90.0)
        spin_align_tolerance_deg = p.get("spin_align_tolerance_deg", 15.0)
        spin_align_max_frames = int(p.get("spin_align_max_frames", 60))
        final_turn_tolerance_deg = p.get("final_turn_tolerance_deg", 5.0)
        final_turn_speed = p.get("final_turn_speed", 45.0)
        final_turn_max_frames = int(p.get("final_turn_max_frames", 60))
        spin_speed = p.get("spin_speed", 45.0)

        start_time = time.monotonic()

        try:
            field = self._get_playfield(camera_index)
            odom = Odometry(field, robot_tag, otos=self._otos, params=self.params)

            # -- Step 1: Resolve start_pose --
            if start_pose is None:
                odom.update()
                if not odom.is_valid:
                    return {"error": "Robot tag not found; cannot determine start pose."}
                start_pose = Pose(x=odom.x, y=odom.y, heading=odom.yaw)

            # -- Step 2: Build path --
            path = build_path(
                method,
                start_pose,
                end_pose,
                waypoints or [],
                spacing_cm=p.get("path_spacing_cm", 1.0),
                tangent_frac=p.get("bezier_tangent_frac", 0.33),
            )
            planned_path_dict = path.to_dict()

            # -- Step 3: Preview --
            if preview:
                _preview_mod.preview_polyline(path.points)

            # -- Step 4: Spin-align --
            spin_frames = 0
            if path.headings:
                initial_tangent = path.headings[0]
                # Read current yaw (odom may already be valid from step 1)
                if not odom.is_valid:
                    odom.update()
                if odom.is_valid:
                    err_rad = heading_error(odom.yaw, initial_tangent)
                    if abs(math.degrees(err_rad)) > spin_align_threshold_deg:
                        spin_result = self._spin_to_heading(
                            target_heading_rad=initial_tangent,
                            field=field,
                            odom=odom,
                            tolerance_deg=spin_align_tolerance_deg,
                            max_frames=spin_align_max_frames,
                            speed=spin_speed,
                        )
                        spin_frames = spin_result.get("frames", 0)

            # -- Step 5: Path following --
            ctrl_cls = CONTROLLERS.get(controller)
            if ctrl_cls is None:
                raise ValueError(
                    f"Unknown controller {controller!r}. "
                    f"Supported values: {list(CONTROLLERS)}"
                )
            ctrl = _build_controller(ctrl_cls, self.params,
                                     lookahead=lookahead,
                                     trackwidth=trackwidth,
                                     base_speed=base_speed,
                                     stop_dist=stop_dist)
            ctrl.set_path(path.points)
            pursuit_result = self._run_controller(
                ctrl, field, odom, timeout, start_time,
                initial_frame_count=spin_frames,
            )

            traversed_frames = pursuit_result.get("frames", spin_frames)
            elapsed_s = round(time.monotonic() - start_time, 2)

            # Build final_pose from odom (may be None if tag lost)
            if odom.is_valid:
                final_pose_dict = {
                    "x": round(odom.x, 2),
                    "y": round(odom.y, 2),
                    "heading": round(odom.yaw, 4),
                }
            else:
                final_pose_dict = None

            if not pursuit_result.get("success"):
                return {
                    "success": False,
                    "planned_path": planned_path_dict,
                    "traversed_frames": traversed_frames,
                    "elapsed_s": elapsed_s,
                    "final_pose": final_pose_dict,
                    "final_heading_error_deg": None,
                    "error": pursuit_result.get("reason", "pursuit_failed"),
                }

            # -- Step 6: Final turn --
            final_turn_result = self._spin_to_heading(
                target_heading_rad=end_pose.heading,
                field=field,
                odom=odom,
                tolerance_deg=final_turn_tolerance_deg,
                max_frames=final_turn_max_frames,
                speed=final_turn_speed,
            )
            traversed_frames += final_turn_result.get("frames", 0)

            # Re-read final pose
            if odom.is_valid:
                final_pose_dict = {
                    "x": round(odom.x, 2),
                    "y": round(odom.y, 2),
                    "heading": round(odom.yaw, 4),
                }

            final_heading_error_deg = None
            if odom.is_valid:
                err = heading_error(odom.yaw, end_pose.heading)
                final_heading_error_deg = round(math.degrees(err), 2)

            elapsed_s = round(time.monotonic() - start_time, 2)
            return {
                "success": True,
                "planned_path": planned_path_dict,
                "traversed_frames": traversed_frames,
                "elapsed_s": elapsed_s,
                "final_pose": final_pose_dict,
                "final_heading_error_deg": final_heading_error_deg,
            }

        except Exception as exc:
            self._robot.stop()
            return {"error": str(exc)}

    def read_pose(self, camera_index=3, robot_tag=1) -> dict[str, Any]:
        """Read robot position and orientation from camera."""
        field = self._get_playfield(camera_index)
        frame_count = 0
        for tags in field.stream():
            frame_count += 1
            if frame_count > 30:
                break
            for t in tags:
                if t.id == robot_tag and t.wx is not None:
                    if t.age > 0.3:
                        continue
                    return {
                        "x": round(t.wx, 2),
                        "y": round(t.wy, 2),
                        "yaw": round(t.orientation, 4),
                        "tag_id": robot_tag,
                    }
        return {"error": "Robot tag not found"}

    def visit_tags(self, tag_ids, camera_index=3, robot_tag=1,
                   per_tag_timeout=15.0) -> dict[str, Any]:
        """Visit a list of tags in sequence. Returns results for each tag."""
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        results = []
        total_start = time.monotonic()

        for tag_id in tag_ids:
            # First, find the tag's current position
            field = self._get_playfield(camera_index)
            target_pos = None
            frame_count = 0
            for tags in field.stream():
                frame_count += 1
                if frame_count > 30:
                    break
                for t in tags:
                    if t.id == tag_id and t.wx is not None:
                        target_pos = (t.wx, t.wy)
                        break
                if target_pos:
                    break

            if not target_pos:
                results.append({"tag": tag_id, "success": False, "reason": "tag_not_found"})
                continue

            result = self.navigate((target_pos[0], target_pos[1]),
                                   camera_index=camera_index,
                                   robot_tag=robot_tag,
                                   timeout=per_tag_timeout)
            result["tag"] = tag_id
            results.append(result)

        total_elapsed = time.monotonic() - total_start
        succeeded = sum(1 for r in results if r.get("success"))
        return {
            "results": results,
            "total_elapsed": round(total_elapsed, 1),
            "succeeded": succeeded,
            "total": len(tag_ids),
        }

    def adaptive_turn(self, target_deg, camera_index=3, robot_tag=1,
                      tolerance=5.0, max_steps=15):
        """Closed-loop turn using adaptive GO commands.

        Scales speed and duration to remaining angle for fast convergence.
        Returns (actual_rotation_deg, steps, elapsed_seconds).
        """
        field = self._get_playfield(camera_index)

        # Read initial yaw
        yaw0 = None
        frame_count = 0
        for tags in field.stream():
            frame_count += 1
            if frame_count > 15:
                break
            for t in tags:
                if t.id == robot_tag and t.wx is not None:
                    yaw0 = math.degrees(t.orientation)
                    break
            if yaw0 is not None:
                break
        if yaw0 is None:
            return None, 0, 0

        target_yaw = normalize_angle(math.radians(yaw0 + target_deg))
        target_yaw_deg = math.degrees(target_yaw)
        start = time.monotonic()
        steps = 0

        for _ in range(max_steps):
            yaw = None
            for tags in field.stream():
                for t in tags:
                    if t.id == robot_tag and t.wx is not None:
                        yaw = math.degrees(t.orientation)
                        break
                if yaw is not None:
                    break
            if yaw is None:
                continue

            remaining = math.degrees(normalize_angle(math.radians(target_yaw_deg - yaw)))
            if abs(remaining) < tolerance:
                break

            if abs(remaining) > 60:
                ms, spd = 150, 60
            elif abs(remaining) > 30:
                ms, spd = 100, 50
            elif abs(remaining) > 10:
                ms, spd = 60, 40
            else:
                ms, spd = 40, 30

            if remaining > 0:
                self._robot.speed_for_time(spd, -spd, ms)
            else:
                self._robot.speed_for_time(-spd, spd, ms)
            time.sleep(max(ms / 1000.0, 0.04))
            steps += 1

        # Final reading
        yaw_final = None
        for tags in field.stream():
            for t in tags:
                if t.id == robot_tag and t.wx is not None:
                    yaw_final = math.degrees(t.orientation)
                    break
            if yaw_final is not None:
                break

        actual = math.degrees(normalize_angle(math.radians(yaw_final - yaw0))) if yaw_final else None
        return actual, steps, time.monotonic() - start

    # -- Gripper navigation --

    def gripper_position(self, tag_pos, yaw):
        """Compute the gripper center position given tag position and yaw."""
        offset = self._robot.gripper_offset
        return (tag_pos[0] + offset * math.cos(yaw),
                tag_pos[1] + offset * math.sin(yaw))

    def _read_pose_from_field(self, field, robot_tag, max_frames=15):
        """Read robot pose from the playfield stream."""
        frame_count = 0
        for tags in field.stream():
            frame_count += 1
            if frame_count > max_frames:
                break
            for t in tags:
                if t.id == robot_tag and t.wx is not None:
                    if t.age > 0.3:
                        continue
                    return (t.wx, t.wy), t.orientation
        return None, None

    def _drive_straight(self, distance_cm, speed=35):
        """Drive forward (positive) or backward (negative) a fixed distance.
        Uses timed GO command — no steering.
        """
        # Calibration: ~0.7cm per 100ms at speed 35
        ms = max(30, int(abs(distance_cm) / 0.7 * 100))
        ms = min(ms, 800)
        if distance_cm >= 0:
            self._robot.speed_for_time(speed, speed, ms)
        else:
            self._robot.speed_for_time(-speed, -speed, ms)

    def grab_at(self, target_xy, camera_index=3, robot_tag=1,
                timeout=25.0, max_iters=5, tolerance=3.0) -> dict[str, Any]:
        """Navigate gripper to target using closed-loop correction.

        Phase 1: Navigate the tag to roughly gripper_offset behind the target
                 (using the bearing from current position to target).
        Phase 2: Iteratively correct by measuring actual gripper position
                 and shifting the tag by a dampened error vector.

        The heading is unpredictable after navigate_to, so phase 2 measures
        the actual gripper position each iteration and corrects.
        """
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        gripper_offset = self._robot.gripper_offset
        field = self._get_playfield(camera_index)
        log = []
        start = time.monotonic()

        # Phase 1: Get the tag roughly behind the target
        robot_pos, robot_yaw = self._read_pose_from_field(field, robot_tag)
        if robot_pos is None:
            return {"error": "Robot tag not found"}

        bearing = math.atan2(target_xy[1] - robot_pos[1],
                              target_xy[0] - robot_pos[0])
        rough_tag_x = target_xy[0] - gripper_offset * math.cos(bearing)
        rough_tag_y = target_xy[1] - gripper_offset * math.sin(bearing)

        log.append(f"phase1: tag=({robot_pos[0]:.1f},{robot_pos[1]:.1f}) "
                   f"rough_target=({rough_tag_x:.1f},{rough_tag_y:.1f})")

        result = self.navigate((rough_tag_x, rough_tag_y),
                               camera_index=camera_index,
                               robot_tag=robot_tag,
                               timeout=min(12, timeout * 0.5))
        tag_dist = result.get('final_dist', -1)
        log.append(f"phase1 done: tag_dist={tag_dist:.1f}")
        time.sleep(0.3)

        # Phase 2: Closed-loop gripper correction
        for i in range(max_iters):
            if time.monotonic() - start > timeout:
                log.append(f"iter{i}: timeout")
                break

            robot_pos, robot_yaw = self._read_pose_from_field(field, robot_tag)
            if robot_pos is None:
                return {"error": f"Robot tag not found at iter {i}"}

            grip_x, grip_y = self.gripper_position(robot_pos, robot_yaw)
            grip_err = math.sqrt((target_xy[0] - grip_x)**2 +
                                  (target_xy[1] - grip_y)**2)

            log.append(f"iter{i}: tag=({robot_pos[0]:.1f},{robot_pos[1]:.1f}) "
                       f"grip=({grip_x:.1f},{grip_y:.1f}) err={grip_err:.1f}cm")

            if grip_err <= tolerance:
                log.append(f"converged at iter {i}")
                return {
                    "success": True,
                    "gripper_dist": round(grip_err, 1),
                    "gripper_pos": [round(grip_x, 1), round(grip_y, 1)],
                    "tag_pos": [round(robot_pos[0], 1), round(robot_pos[1], 1)],
                    "iterations": i,
                    "elapsed": round(time.monotonic() - start, 1),
                    "log": log,
                }

            # Dampened correction: use gain < 1 to avoid overshooting.
            # When the tag moves, the heading changes, which rotates the
            # gripper offset vector. A full correction overshoots.
            gain = 0.6
            err_x = target_xy[0] - grip_x
            err_y = target_xy[1] - grip_y
            tag_target_x = robot_pos[0] + gain * err_x
            tag_target_y = robot_pos[1] + gain * err_y

            log.append(f"  correction: ({gain*err_x:.1f},{gain*err_y:.1f}) "
                       f"-> tag ({tag_target_x:.1f},{tag_target_y:.1f})")

            result = self.navigate((tag_target_x, tag_target_y),
                                   camera_index=camera_index,
                                   robot_tag=robot_tag,
                                   timeout=min(8, timeout - (time.monotonic() - start)))

            tag_dist = result.get('final_dist', -1)
            log.append(f"  nav done: tag_dist={tag_dist:.1f}")

            time.sleep(0.2)

        # Final measurement
        robot_pos, robot_yaw = self._read_pose_from_field(field, robot_tag)
        if robot_pos is None:
            log.append("lost tag at final")
            return {"success": False, "gripper_dist": None, "log": log}

        grip_x, grip_y = self.gripper_position(robot_pos, robot_yaw)
        final_dist = math.sqrt((target_xy[0] - grip_x)**2 +
                                (target_xy[1] - grip_y)**2)
        log.append(f"final: grip=({grip_x:.1f},{grip_y:.1f}) err={final_dist:.1f}cm")

        return {
            "success": final_dist <= tolerance,
            "gripper_dist": round(final_dist, 1),
            "gripper_pos": [round(grip_x, 1), round(grip_y, 1)],
            "tag_pos": [round(robot_pos[0], 1), round(robot_pos[1], 1)],
            "iterations": max_iters,
            "elapsed": round(time.monotonic() - start, 1),
            "log": log,
        }

    def release_at(self, target_xy, camera_index=3, robot_tag=1,
                   timeout=15.0) -> dict[str, Any]:
        """Navigate so the gripper is over target_xy for releasing."""
        return self.grab_at(target_xy, camera_index=camera_index,
                            robot_tag=robot_tag, timeout=timeout)

    # -- Two-phase approach controller --

    def approach(
        self,
        target_xy: tuple[float, float],
        camera_index: int = 3,
        robot_tag: int = 1,
        tolerance_mm: float = 5.0,
        timeout: float = 20.0,
        far_threshold_mm: float = 100.0,
    ) -> dict[str, Any]:
        """Drive the robot to *target_xy* using a two-phase closed-loop controller.

        **Far phase** (r > far_threshold_mm):
            Issue a single ``speed_for_time`` command derived from the linear
            calibration model, re-read pose, repeat.

        **Near phase** (tolerance_mm < r <= far_threshold_mm):
            Issue crawl pulses at the global_best crawl settings and re-read
            pose after each pulse.

        **Done** (r <= tolerance_mm):
            Stop and return success dict.

        Args:
            target_xy: World coordinate (x, y) in **centimetres**.
            camera_index: AprilCam camera index.
            robot_tag: Tag ID of the robot.
            tolerance_mm: Arrival radius in millimetres (default 5).
            timeout: Maximum seconds to run (default 20).
            far_threshold_mm: Radius (mm) below which we switch to near phase
                (default 100).

        Returns:
            ``{success, elapsed_s, final_error_mm, phases_used,
            n_far_commands, n_crawl_pulses}``
        """
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        field = self._get_playfield(camera_index)
        cal = self._approach_cal

        start = time.monotonic()
        n_far = 0
        n_crawl = 0
        phases_used: set[str] = set()
        last_r_mm: float = 0.0

        try:
            while True:
                # ── Read pose (non-blocking poll on field.tag) ─────────────
                # NB: field.stream() can hang indefinitely if the pipeline
                # stops yielding; field.tag() polls the ring buffer and
                # returns None if the tag isn't fresh.
                deadline = time.monotonic() + 2.5
                robot_pos = None
                while time.monotonic() < deadline:
                    t = field.tag(robot_tag)
                    if t is not None and t.wx is not None:
                        robot_pos = (t.wx, t.wy)
                        break
                    time.sleep(0.05)
                if robot_pos is None:
                    self._robot.stop()
                    return {
                        "success": False,
                        "reason": "lost_robot_tag",
                        "elapsed_s": round(time.monotonic() - start, 2),
                        "final_error_mm": round(last_r_mm, 1),
                        "phases_used": sorted(phases_used),
                        "n_far_commands": n_far,
                        "n_crawl_pulses": n_crawl,
                    }

                dx = (target_xy[0] - robot_pos[0]) * 10.0  # cm → mm
                dy = (target_xy[1] - robot_pos[1]) * 10.0
                r_mm = math.sqrt(dx * dx + dy * dy)
                last_r_mm = r_mm

                # ── Choose phase ───────────────────────────────────────────
                phase = choose_phase(r_mm,
                                     far_threshold_mm=far_threshold_mm,
                                     tolerance_mm=tolerance_mm)

                if phase == "done":
                    self._robot.stop()
                    return {
                        "success": True,
                        "elapsed_s": round(time.monotonic() - start, 2),
                        "final_error_mm": round(r_mm, 1),
                        "phases_used": sorted(phases_used),
                        "n_far_commands": n_far,
                        "n_crawl_pulses": n_crawl,
                    }

                # ── Timeout check ──────────────────────────────────────────
                elapsed = time.monotonic() - start
                if elapsed > timeout:
                    self._robot.stop()
                    return {
                        "success": False,
                        "reason": "timeout",
                        "elapsed_s": round(elapsed, 2),
                        "final_error_mm": round(r_mm, 1),
                        "phases_used": sorted(phases_used),
                        "n_far_commands": n_far,
                        "n_crawl_pulses": n_crawl,
                    }

                if phase == "far":
                    v, t_ms = compute_far_command(r_mm, cal)
                    if t_ms > 0:
                        # speed_for_time is blocking; it returns after the
                        # firmware finishes the drive.  No extra sleep needed.
                        self._robot.speed_for_time(int(v), int(v), t_ms)
                    n_far += 1
                    phases_used.add("far")

                else:  # near
                    crawl = cal["crawl"]
                    spd = int(crawl["speed_mms"])
                    pulse_ms = int(crawl["pulse_ms"])
                    delay_ms = int(crawl["delay_ms"])
                    self._robot.speed_for_time(spd, spd, pulse_ms)
                    time.sleep(delay_ms / 1000.0)
                    n_crawl += 1
                    phases_used.add("near")

        except Exception as exc:
            self._robot.stop()
            return {"error": str(exc)}
