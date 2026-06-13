"""Navigator — autonomous navigation via firmware G commands with camera feedback.

After ticket 035-002 (pose-authority sprint A1), Navigator is a route planner:
it sequences firmware G commands and reads camera pose for corrections.  The
host-side dual-PID steering loop (ChaseController, _run_controller, follow_pose_path)
has been deleted.  The firmware G path is the sole steering loop.

Retained methods: navigate (G-wrapper), follow_path (G-wrapper),
  visit_tags, approach, grab_at, release_at, read_pose,
  adaptive_turn, gripper_position, _read_pose_from_field,
  _drive_straight, _get_playfield, reset_camera, status, get_next_tags.
"""

import json
import math
import os
import time
from typing import Any

from aprilcam import Camera, Playfield, Tag

from robot_radio.controllers.pid import PID, normalize_angle
from robot_radio.nav.nav_params import NavParams
from robot_radio.nav.pose import Pose, Waypoint, heading_error
from robot_radio.nav._approach_utils import (
    choose_phase,
    compute_far_command,
    load_approach_calibration,
)

# Default navigation speed for firmware G commands (mm/s).
# Corresponds roughly to motor command 40 at the robot's calibration.
_DEFAULT_NAV_SPEED_MMS = 200


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


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_DATA_DIR = os.path.join(_SCRIPT_DIR, "..", "data")


class Navigator:
    """Route planner that delegates point-to-point motion to the firmware G command.

    ## navigate()

    ``navigate(target_xy, timeout=30.0, ...)`` drives the robot to a world-
    coordinate target by:
    1. Reading the robot's current world pose from the camera.
    2. Computing the robot-relative displacement in mm.
    3. Issuing the firmware ``G <dx> <dy> <speed>`` command.
    4. Waiting for ``EVT done G`` (blocking, via ``self._robot.go_to``).

    Returns a dict with keys: ``success``, ``elapsed``, and optionally
    ``outcome`` (the raw firmware outcome string).

    ## approach()

    ``approach(target_xy, tolerance_mm=5, timeout=20.0)`` drives the robot
    tag to a world-coordinate target using a two-phase closed-loop controller:

    - **Far phase** (r > 100 mm): issue a single calibrated ``speed_for_time``
      command, re-read pose, repeat.
    - **Near phase** (r <= 100 mm): issue short crawl pulses and re-read pose
      after each pulse.
    - **Done** (r <= tolerance_mm): stop and return.

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

    # -- Core navigation (G-command wrappers, ticket 035-002) --

    def navigate(self, target_xy, camera_index=3, robot_tag=1,
                 timeout=30.0, speed_mms: int = _DEFAULT_NAV_SPEED_MMS,
                 forward_only=False) -> dict[str, Any]:
        """Drive to *target_xy* via the firmware G command (035-002).

        Reads the robot's current world pose from the camera, converts the
        world-cm target to robot-relative mm, and issues a firmware G command.
        Blocks until ``EVT done G`` is received or *timeout* expires.

        Parameters
        ----------
        target_xy:
            ``(x, y)`` in world-frame centimetres.
        camera_index:
            AprilCam camera index (default 3, B&W).
        robot_tag:
            AprilTag ID on the robot (default 1).
        timeout:
            Maximum wall-clock seconds to wait for completion (default 30).
        speed_mms:
            Navigation speed in mm/s sent to the firmware G command
            (default 200).
        forward_only:
            Ignored in the G-command implementation (firmware chooses its own
            approach geometry).  Accepted for API compatibility.

        Returns
        -------
        dict
            ``{"success": True, "elapsed": float, "outcome": str}`` on success,
            ``{"success": False, "elapsed": float, "reason": str}`` on failure,
            or ``{"error": str}`` on exception.
        """
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        start = time.monotonic()

        try:
            field = self._get_playfield(camera_index)

            # Read current robot world pose from camera.
            robot_pos, robot_yaw = self._read_pose_from_field(field, robot_tag)
            if robot_pos is None:
                return {
                    "success": False,
                    "reason": "robot_tag_not_found",
                    "elapsed": round(time.monotonic() - start, 1),
                }

            # Convert world-cm target to robot-relative mm.
            # The firmware G command takes robot-relative coordinates:
            #   dx_robot =  dx_world_mm * cos(yaw) + dy_world_mm * sin(yaw)
            #   dy_robot = -dx_world_mm * sin(yaw) + dy_world_mm * cos(yaw)
            dx_mm = (target_xy[0] - robot_pos[0]) * 10.0  # cm → mm
            dy_mm = (target_xy[1] - robot_pos[1]) * 10.0
            dx_robot = int(round(dx_mm * math.cos(robot_yaw) + dy_mm * math.sin(robot_yaw)))
            dy_robot = int(round(-dx_mm * math.sin(robot_yaw) + dy_mm * math.cos(robot_yaw)))

            # Issue the firmware G command and wait for EVT done G.
            enc_l, enc_r, outcome = self._robot.go_to(
                dx_robot, dy_robot, speed_mms, timeout_s=timeout
            )

            elapsed = round(time.monotonic() - start, 1)
            if outcome == "done":
                return {"success": True, "elapsed": elapsed, "outcome": outcome}
            else:
                return {
                    "success": False,
                    "reason": outcome,
                    "elapsed": elapsed,
                    "outcome": outcome,
                }

        except Exception as exc:
            return {"error": str(exc)}

    def follow_path(
        self,
        path: list[tuple[float, float]],
        camera_index: int = 3,
        robot_tag: int = 1,
        timeout: float = 30.0,
        speed_mms: int = _DEFAULT_NAV_SPEED_MMS,
        # Legacy parameters accepted for API compatibility but unused:
        lookahead: float = 15.0,
        trackwidth: float = 9.0,
        base_speed: float = 40.0,
        stop_dist: float = 5.0,
        controller: str = "g_command",
    ) -> dict[str, Any]:
        """Sequence one G command per consecutive waypoint (035-002).

        Issues a firmware G command for each point in *path* in order,
        waiting for ``EVT done G`` before advancing to the next waypoint.

        Parameters
        ----------
        path:
            Ordered list of ``(x, y)`` world-coordinate waypoints in cm.
            Must contain at least one point.
        camera_index:
            Camera index (default 3, B&W).
        robot_tag:
            AprilTag ID on the robot (default 1).
        timeout:
            Per-waypoint timeout in seconds (default 30.0).
        speed_mms:
            Navigation speed in mm/s for each G command (default 200).
        lookahead, trackwidth, base_speed, stop_dist, controller:
            Accepted for API compatibility; ignored.

        Returns
        -------
        dict
            ``{"success": True, "waypoints_completed": int, "elapsed": float}``
            on full completion, or
            ``{"success": False, "reason": str, "waypoints_completed": int,
            "elapsed": float}`` on failure.
        """
        if not self._robot.is_connected():
            return {"error": "Not connected. Call connect first."}

        start = time.monotonic()
        completed = 0

        for wp in path:
            result = self.navigate(
                wp,
                camera_index=camera_index,
                robot_tag=robot_tag,
                timeout=timeout,
                speed_mms=speed_mms,
            )
            if result.get("error"):
                return {
                    "success": False,
                    "reason": result["error"],
                    "waypoints_completed": completed,
                    "elapsed": round(time.monotonic() - start, 1),
                }
            if not result.get("success"):
                return {
                    "success": False,
                    "reason": result.get("reason", "navigate_failed"),
                    "waypoints_completed": completed,
                    "elapsed": round(time.monotonic() - start, 1),
                }
            completed += 1

        return {
            "success": True,
            "waypoints_completed": completed,
            "elapsed": round(time.monotonic() - start, 1),
        }

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
        tag_dist = result.get('final_dist', result.get('elapsed', -1))
        log.append(f"phase1 done: navigate={'ok' if result.get('success') else 'failed'}")
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

            log.append(f"  nav done: success={result.get('success')}")

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
