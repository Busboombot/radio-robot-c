"""
Robot MCP server — thin dispatch shell for serial control, vision navigation,
and radio bridge discovery.

All business logic lives in SerialConnection, Nezha, Navigator, and NavParams.
This module only wires MCP tool definitions to those objects.

Run with:
  uv run python -m robot_radio.io.robot_mcp
"""

import asyncio
import importlib
import json
import os
from typing import Any

import mcp.server.stdio
import mcp.types as types
from mcp.server import Server

from robot_radio.media import movie
from robot_radio.io.serial_conn import (
    SerialConnection, list_serial_ports, probe_devices, DEFAULT_PORT,
)
from robot_radio.robot import Nezha, NezhaProtocol
from robot_radio.robot.connection import make_robot as _make_robot
from robot_radio.calibration.push import push_calibration
from robot_radio.nav.navigator import Navigator, log_record
from robot_radio.nav.nav_params import NavParams
from robot_radio.sensors.otos import Otos
from robot_radio.path import build_path
from robot_radio.nav.pose import Pose, Waypoint
import robot_radio.io.preview as preview_mod
from robot_radio.config.robot_config import get_robot_config, RobotConfig

server = Server("robot")

# ── Module-level state ──────────────────────────────────────────────────────

_conn: SerialConnection | None = None
_robot: Nezha | None = None
_navigator: Navigator | None = None
_otos: Otos | None = None
_config: RobotConfig | None = None


# ── Helpers ─────────────────────────────────────────────────────────────────


def _default_robot_tag() -> int:
    return _config.robot_tag_id if _config is not None else 1

def _mock_args(port: str, mode: str | None):
    """Create a minimal args namespace for make_robot().

    ``make_robot`` accepts a CLI-style args object for backward compatibility
    with the argparse Namespace used by the CLI.  The MCP server does not have
    argparse; this factory builds a minimal namespace with only the attributes
    that make_robot needs (``port``).
    """
    import types
    ns = types.SimpleNamespace()
    ns.port = port  # Non-None forces cache bypass when an explicit port is given.
    return ns


def _connect(port: str, mode: str | None) -> dict[str, Any]:
    """Open a serial connection to the robot.

    Routes through ``robot_radio.robot.connection.make_robot`` so the MCP
    server and the CLI share the same port-resolution, HELLO handshake, mode
    detection, and session-cache logic.

    Calibration push is delegated to
    ``robot_radio.calibration.push.push_calibration``.
    """
    global _conn, _robot, _navigator, _otos, _config
    try:
        robot, conn, result = _make_robot(
            port=port if port != DEFAULT_PORT else None,
            mode=mode,
            verbose=False,
            args=_mock_args(port if port != DEFAULT_PORT else None, mode),
        )
    except SystemExit as exc:
        # make_robot calls sys.exit() on connection failure; convert to error
        # dict so the MCP tool can return an error result instead of crashing.
        return {"error": str(exc)}
    _conn = conn
    _robot = robot
    _otos = Otos(_conn)
    _navigator = Navigator(_robot, otos=_otos)
    _config = get_robot_config()
    if _config is not None:
        result["calibration"] = push_calibration(_robot._proto, _config)
    return result


def _disconnect() -> dict[str, Any]:
    global _conn, _robot, _navigator, _otos
    if _navigator:
        _navigator.reset_camera()
    result = _conn.disconnect() if _conn else {"status": "not_connected"}
    _conn = None
    _robot = None
    _navigator = None
    _otos = None
    return result


def _reload_nav() -> dict[str, Any]:
    global _navigator
    try:
        import robot_radio.nav.navigator as nav_mod
        import robot_radio.controllers.pid as pid_mod
        import robot_radio.nav.nav_params as params_mod
        pid_mod = importlib.reload(pid_mod)
        params_mod = importlib.reload(params_mod)
        nav_mod = importlib.reload(nav_mod)
        old_params = _navigator.params.as_dict() if _navigator else {}
        _navigator = nav_mod.Navigator(_robot, otos=_otos)
        if old_params:
            _navigator.params.update(**old_params)
        return {"reloaded": True, "module": nav_mod.__file__}
    except Exception as exc:
        return {"error": str(exc)}


def _require_robot() -> dict[str, Any] | None:
    """Return an error dict if robot is not connected, else None."""
    if _robot is None or not _robot.is_connected():
        return {"error": "Not connected. Call connect first."}
    return None


def _object_record_to_dict(o: Any) -> dict[str, Any]:
    """Serialize an aprilcam ObjectRecord to a wire-friendly dict."""
    return {
        "center_px": list(o.center_px),
        "world_xy": list(o.world_xy) if o.world_xy else None,
        "color": o.color,
        "object_type": o.object_type,
        "confidence": o.confidence,
        "area_px": o.area_px,
    }


def _require_navigator() -> dict[str, Any] | None:
    """Return an error dict if navigator is not available, else None."""
    if _navigator is None:
        return {"error": "Not connected. Call connect first."}
    return None


def _require_otos() -> dict[str, Any] | None:
    """Return an error dict if the OTOS wrapper is not available, else None."""
    if _otos is None:
        return {"error": "Not connected. Call connect first."}
    return None


# ── Tool definitions ────────────────────────────────────────────────────────

@server.list_tools()
async def list_tools() -> list[types.Tool]:
    return [
        # Connection
        types.Tool(name="connect",
                   description="Open serial connection to robot relay.",
                   inputSchema={"type": "object", "properties": {
                       "port": {"type": "string", "default": DEFAULT_PORT},
                       "mode": {"type": "string", "enum": ["relay", "direct"]},
                   }, "required": []}),
        types.Tool(name="disconnect",
                   description="Close serial connection.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="status",
                   description="Show connection and camera status.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),

        # Motor control
        types.Tool(name="go",
                   description="Drive wheels at left/right speeds (mm/s) for a duration in ms. Returns final encoder positions.",
                   inputSchema={"type": "object", "properties": {
                       "ms": {"type": "integer"},
                       "left": {"type": "integer"},
                       "right": {"type": "integer"},
                   }, "required": ["ms", "left", "right"]}),
        types.Tool(name="goto",
                   description="Drive robot to a relative XY position (mm) using the firmware G command. "
                               "Robot-relative: X=forward, Y=left. Pre-rotates if heading error > KGT degrees, "
                               "then follows a pure-pursuit arc. Waits for G+DONE (firmware sprint 005+).",
                   inputSchema={"type": "object", "properties": {
                       "x_mm": {"type": "integer", "description": "Target X in mm (forward)"},
                       "y_mm": {"type": "integer", "description": "Target Y in mm (left)"},
                       "speed_mm_s": {"type": "integer", "description": "Drive speed in mm/s (1–999)"},
                       "timeout": {"type": "number", "default": 30, "description": "Seconds to wait for G+DONE"},
                   }, "required": ["x_mm", "y_mm", "speed_mm_s"]}),
        types.Tool(name="stop",
                   description="Stop motors.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="grip",
                   description="Set gripper servo angle (0=open, 180=closed).",
                   inputSchema={"type": "object", "properties": {
                       "angle": {"type": "integer"},
                   }, "required": ["angle"]}),
        types.Tool(name="send",
                   description="Send arbitrary command string.",
                   inputSchema={"type": "object", "properties": {
                       "message": {"type": "string"},
                       "read_ms": {"type": "integer", "default": 500},
                   }, "required": ["message"]}),

        # Navigation
        types.Tool(name="navigate_to",
                   description="PID-navigate to world coordinate (cm) using camera feedback at ~30fps.",
                   inputSchema={"type": "object", "properties": {
                       "x": {"type": "number"},
                       "y": {"type": "number"},
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                       "timeout": {"type": "number", "default": 30},
                       "forward_only": {"type": "boolean", "default": False},
                   }, "required": ["x", "y"]}),
        types.Tool(name="visit_tags",
                   description="Visit a list of tags in sequence, navigating to each one.",
                   inputSchema={"type": "object", "properties": {
                       "tags": {"type": "array", "items": {"type": "integer"},
                                "description": "Tag IDs to visit in order"},
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                       "timeout": {"type": "number", "default": 15,
                                   "description": "Per-tag timeout in seconds"},
                   }, "required": ["tags"]}),
        types.Tool(name="read_pose",
                   description="Read robot position and orientation from camera.",
                   inputSchema={"type": "object", "properties": {
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                   }, "required": []}),
        types.Tool(name="tune",
                   description="Get or set ALL navigation parameters. Changes take effect immediately.",
                   inputSchema={"type": "object",
                                "properties": {k: {"type": "number", "description": k}
                                               for k in NavParams().as_dict()},
                                "required": []}),

        types.Tool(name="approach",
                   description="Drive the robot tag to a world coordinate (cm) using a "
                               "two-phase closed-loop controller: far phase (r > 100 mm) "
                               "uses a single calibrated drive command; near phase "
                               "(r <= 100 mm) uses crawl pulses. "
                               "Returns success/elapsed_s/final_error_mm/phases_used/"
                               "n_far_commands/n_crawl_pulses.",
                   inputSchema={"type": "object", "properties": {
                       "target": {"type": "array", "items": {"type": "number"},
                                  "minItems": 2, "maxItems": 2,
                                  "description": "[x, y] target in cm"},
                       "tolerance_mm": {"type": "number", "default": 5,
                                        "description": "Arrival radius in mm"},
                       "timeout": {"type": "number", "default": 20,
                                   "description": "Max seconds to run"},
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                   }, "required": ["target"]}),

        types.Tool(name="follow_path",
                   description="Follow a multi-waypoint path using a path-following controller and camera feedback. "
                               "path is a list of [x, y] world coordinates in cm. "
                               "The robot tracks the path using the selected controller; arrival is declared "
                               "when within stop_dist cm of the final waypoint. "
                               "Use controller='pure_pursuit' (default) or controller='stanley'.",
                   inputSchema={"type": "object", "properties": {
                       "path": {"type": "array",
                                "items": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
                                "description": "Ordered list of [x, y] waypoints in cm"},
                       "camera_index": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                       "timeout": {"type": "number", "default": 30.0},
                       "lookahead": {"type": "number", "default": 15.0,
                                     "description": "Lookahead circle radius in cm (used by pure_pursuit)"},
                       "trackwidth": {"type": "number", "default": 9.0,
                                      "description": "Wheel-to-wheel spacing in cm"},
                       "base_speed": {"type": "number", "default": 40.0,
                                      "description": "Nominal forward motor command (0-100)"},
                       "stop_dist": {"type": "number", "default": 5.0,
                                     "description": "Arrival threshold in cm"},
                       "controller": {"type": "string", "default": "pure_pursuit",
                                      "description": "Path-following controller: 'pure_pursuit' or 'stanley'"},
                   }, "required": ["path"]}),

        # Gripper navigation
        types.Tool(name="grab_at",
                   description="Navigate gripper to world coordinate and prepare to grab. "
                               "Accounts for 7cm offset between tag and gripper. "
                               "Caller must send GRIP 180 after to close gripper.",
                   inputSchema={"type": "object", "properties": {
                       "x": {"type": "number", "description": "Target x in cm"},
                       "y": {"type": "number", "description": "Target y in cm"},
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                       "timeout": {"type": "number", "default": 20},
                   }, "required": ["x", "y"]}),
        types.Tool(name="release_at",
                   description="Navigate gripper to world coordinate for releasing. "
                               "Accounts for 7cm offset. Caller must send GRIP 0 after.",
                   inputSchema={"type": "object", "properties": {
                       "x": {"type": "number", "description": "Drop-off x in cm"},
                       "y": {"type": "number", "description": "Drop-off y in cm"},
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                       "timeout": {"type": "number", "default": 15},
                   }, "required": ["x", "y"]}),

        # Device discovery
        types.Tool(name="list_serial_ports",
                   description="List USB modem serial ports.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="probe_devices",
                   description="Probe each USB modem port for device announcements.",
                   inputSchema={"type": "object", "properties": {
                       "read_ms": {"type": "integer", "default": 1200},
                   }, "required": []}),

        # Utility
        types.Tool(name="reload_nav",
                   description="Hot-reload navigator and pid modules. Code changes take effect immediately.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="reset_camera",
                   description="Release the camera / tag generator.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="detect_objects",
                   description="Detect colored cubes on the playfield. Returns world positions and colors.",
                   inputSchema={"type": "object", "properties": {
                       "bw_camera": {"type": "integer", "default": 3,
                                     "description": "B&W camera index for shape detection"},
                       "color_camera": {"type": "integer", "default": 2,
                                        "description": "Color camera index for classification"},
                   }, "required": []}),
        types.Tool(name="aprilcam_help",
                   description="Get the AprilCam library usage guide.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="list_movie_sessions",
                   description="List all available movie recording sessions.",
                   inputSchema={"type": "object", "properties": {
                       "root_dir": {"type": "string", "description": "Root directory for sessions (default: data/recordings/movies)"},
                   }, "required": []}),
        types.Tool(name="save_movie_frames",
                   description="Capture de-skewed playfield frames into a session directory during a task.",
                   inputSchema={"type": "object", "properties": {
                       "camera": {"type": "integer", "default": 3},
                       "duration_s": {"type": "number", "default": 10},
                       "max_frames": {"type": "integer", "default": 0},
                       "root_dir": {"type": "string"},
                       "session_name": {"type": "string"},
                       "pixels_per_cm": {"type": "number", "default": 8.0},
                       "diff_threshold": {"type": "number", "default": 2.0},
                       "min_interval_ms": {"type": "integer", "default": 0},
                       "max_gap_s": {"type": "number", "default": 1.0},
                       "image_format": {"type": "string", "enum": ["jpg", "png"], "default": "jpg"},
                       "jpeg_quality": {"type": "integer", "default": 90},
                   }, "required": []}),
        types.Tool(name="make_movie",
                   description="Encode saved session frames into an MP4 video.",
                   inputSchema={"type": "object", "properties": {
                       "session": {"type": "string"},
                       "root_dir": {"type": "string"},
                       "output_path": {"type": "string"},
                       "fps": {"type": "number", "default": 15.0},
                       "codec": {"type": "string", "default": "mp4v"},
                       "use_annotated": {"type": "boolean", "default": False},
                       "overlay_tags": {"type": "boolean", "default": False},
                       "overlay_frame_index": {"type": "boolean", "default": False},
                   }, "required": []}),
        types.Tool(name="log_record",
                   description="Append JSON record to JSONL file.",
                   inputSchema={"type": "object", "properties": {
                       "record": {"type": "object"},
                       "file": {"type": "string"},
                   }, "required": ["record"]}),

        # Path planning
        types.Tool(name="plan_path",
                   description="Plan a curved path between two poses using the Bezier builder. "
                               "Pure computation — no robot connection required. "
                               "Returns a path dict with points, headings, total_length_cm, and builder_name.",
                   inputSchema={
                       "type": "object",
                       "properties": {
                           "start": {
                               "type": "object",
                               "description": "Start pose: {x, y, heading} — x/y in cm, heading in radians.",
                               "properties": {
                                   "x": {"type": "number"},
                                   "y": {"type": "number"},
                                   "heading": {"type": "number"},
                               },
                               "required": ["x", "y", "heading"],
                           },
                           "end": {
                               "type": "object",
                               "description": "End pose: {x, y, heading} — x/y in cm, heading in radians.",
                               "properties": {
                                   "x": {"type": "number"},
                                   "y": {"type": "number"},
                                   "heading": {"type": "number"},
                               },
                               "required": ["x", "y", "heading"],
                           },
                           "waypoints": {
                               "type": "array",
                               "description": "Intermediate waypoints. Each is {x, y, heading?} — heading optional.",
                               "items": {
                                   "type": "object",
                                   "properties": {
                                       "x": {"type": "number"},
                                       "y": {"type": "number"},
                                       "heading": {"type": "number"},
                                   },
                                   "required": ["x", "y"],
                               },
                               "default": [],
                           },
                           "method": {
                               "type": "string",
                               "description": "Path builder name.",
                               "default": "bezier",
                           },
                           "spacing_cm": {
                               "type": "number",
                               "description": "Sample spacing along the path in cm.",
                               "default": 1.0,
                           },
                           "tangent_frac": {
                               "type": "number",
                               "description": "Control-point distance as a fraction of chord length (Bezier).",
                               "default": 0.33,
                           },
                       },
                       "required": ["start", "end"],
                   }),
        types.Tool(name="preview_path",
                   description="Log a planned path polyline (stub — AprilCam draw tool not yet available). "
                               "Accepts the dict returned by plan_path. Returns {status, points}.",
                   inputSchema={
                       "type": "object",
                       "properties": {
                           "path": {
                               "type": "object",
                               "description": "Path dict as returned by plan_path.",
                           },
                       },
                       "required": ["path"],
                   }),

        types.Tool(name="follow_pose_path",
                   description=(
                       "Plan a curved path between two poses and drive the robot along it. "
                       "Three phases: (1) spin-align to face the initial path tangent, "
                       "(2) path-following controller to track the path, (3) final in-place turn to match "
                       "end_pose.heading. "
                       "If start_pose is omitted, current pose is read from the camera. "
                       "Use controller='pure_pursuit' (default) or controller='stanley'. "
                       "Returns {success, planned_path, traversed_frames, elapsed_s, "
                       "final_pose, final_heading_error_deg}."
                   ),
                   inputSchema={
                       "type": "object",
                       "properties": {
                           "end_pose": {
                               "type": "object",
                               "description": "Target pose: {x, y, heading} — x/y in cm, heading in radians.",
                               "properties": {
                                   "x": {"type": "number"},
                                   "y": {"type": "number"},
                                   "heading": {"type": "number"},
                               },
                               "required": ["x", "y", "heading"],
                           },
                           "start_pose": {
                               "type": "object",
                               "description": "Start pose: {x, y, heading}. Omit to read from camera.",
                               "properties": {
                                   "x": {"type": "number"},
                                   "y": {"type": "number"},
                                   "heading": {"type": "number"},
                               },
                           },
                           "waypoints": {
                               "type": "array",
                               "description": "Intermediate waypoints. Each is {x, y, heading?}.",
                               "items": {
                                   "type": "object",
                                   "properties": {
                                       "x": {"type": "number"},
                                       "y": {"type": "number"},
                                       "heading": {"type": "number"},
                                   },
                                   "required": ["x", "y"],
                               },
                               "default": [],
                           },
                           "method": {
                               "type": "string",
                               "description": "Path builder name.",
                               "default": "bezier",
                           },
                           "preview": {
                               "type": "boolean",
                               "description": "Log/preview the planned polyline before driving.",
                               "default": True,
                           },
                           "camera_index": {"type": "integer", "default": 3},
                           "robot_tag": {"type": "integer", "default": 1},
                           "timeout": {"type": "number", "default": 30.0},
                           "lookahead": {
                               "type": "number",
                               "description": "Pure-pursuit lookahead radius in cm.",
                               "default": 15.0,
                           },
                           "trackwidth": {
                               "type": "number",
                               "description": "Wheel-to-wheel spacing in cm.",
                               "default": 9.0,
                           },
                           "base_speed": {
                               "type": "number",
                               "description": "Nominal forward motor command (0-100).",
                               "default": 40.0,
                           },
                           "stop_dist": {
                               "type": "number",
                               "description": "Arrival threshold in cm.",
                               "default": 5.0,
                           },
                           "controller": {
                               "type": "string",
                               "description": "Path-following controller: 'pure_pursuit' or 'stanley'.",
                               "default": "pure_pursuit",
                           },
                       },
                       "required": ["end_pose"],
                   }),

        # OTOS sensor tools
        types.Tool(name="otos_init",
                   description="Send OI to the OTOS sensor (enable signal processing). "
                               "Call once after connecting before using OTOS pose.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="otos_calibrate",
                   description="Send OK to calibrate OTOS IMU offsets. "
                               "Robot must be completely still. Returns ACK:OK 255 on success.",
                   inputSchema={"type": "object", "properties": {}, "required": []}),
        types.Tool(name="otos_align",
                   description="Read the current camera pose for the robot tag and align "
                               "the OTOS sensor to that world frame. "
                               "After this call, read_pose_fused will report OTOS in camera coordinates.",
                   inputSchema={"type": "object", "properties": {
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                       "settle_frames": {"type": "integer", "default": 5,
                                         "description": "Camera frames to average before aligning"},
                       "timeout_s": {"type": "number", "default": 4.0,
                                     "description": "Max seconds to wait for valid camera pose"},
                   }, "required": []}),
        types.Tool(name="read_pose_fused",
                   description="Diagnostic: read robot pose from camera and OTOS and report both. "
                               "Returns {camera, otos, source} — no motion.",
                   inputSchema={"type": "object", "properties": {
                       "camera": {"type": "integer", "default": 3},
                       "robot_tag": {"type": "integer", "default": 1},
                   }, "required": []}),
    ]


# ── Tool dispatch ───────────────────────────────────────────────────────────

@server.call_tool()
async def call_tool(name: str, arguments: dict[str, Any]) -> list[types.TextContent]:
    result: Any

    # -- Connection --

    if name == "connect":
        port = str(arguments.get("port", DEFAULT_PORT))
        mode = arguments.get("mode")
        if mode is not None:
            mode = str(mode)
        result = _connect(port, mode)

    elif name == "disconnect":
        result = _disconnect()

    elif name == "status":
        nav_status = _navigator.status() if _navigator else {
            "nav_module": None, "camera_active": False, "camera_index": None,
        }
        result = {
            "connected": _conn.is_open if _conn else False,
            "port": _conn.port if _conn else None,
            "mode": _conn.mode if _conn else "relay",
            "robot_config": {
                "robot_name": _config.robot_name,
                "robot_tag_id": _config.robot_tag_id,
                "has_gripper": _config.has_gripper,
            } if _config else None,
            **nav_status,
        }

    # -- Motor control --

    elif name == "go":
        err = _require_robot()
        if err:
            result = err
        else:
            left_enc, right_enc = await asyncio.to_thread(
                _robot.speed_for_time,
                int(arguments["left"]), int(arguments["right"]), int(arguments["ms"]))
            result = {"left_enc_mm": left_enc, "right_enc_mm": right_enc}

    elif name == "goto":
        err = _require_robot()
        if err:
            result = err
        else:
            x_mm = int(arguments["x_mm"])
            y_mm = int(arguments["y_mm"])
            speed = max(1, min(999, int(arguments["speed_mm_s"])))
            timeout = float(arguments.get("timeout", 30))

            def _s(v: int) -> str:
                return f"+{v}" if v >= 0 else str(v)

            cmd = f"G{_s(x_mm)}{_s(y_mm)}{_s(speed)}"

            import time
            def _goto_blocking() -> dict:
                _robot.send(cmd, read_ms=300)
                deadline = time.time() + timeout
                while time.time() < deadline:
                    for line in _robot._conn.read_lines(duration_ms=500):
                        if "G+DONE" in line:
                            return {"status": "done", "elapsed_s": round(timeout - (deadline - time.time()), 2)}
                return {"status": "timeout", "elapsed_s": timeout}

            result = await asyncio.to_thread(_goto_blocking)

    elif name == "stop":
        err = _require_robot()
        if err:
            result = err
        else:
            _robot.stop()
            result = {"sent": "STOP"}

    elif name == "grip":
        err = _require_robot()
        if err:
            result = err
        else:
            _robot.grip(arguments["angle"])
            result = {"sent": f"GRIP {arguments['angle']}"}

    elif name == "send":
        err = _require_robot()
        if err:
            result = err
        else:
            result = _robot.send(str(arguments["message"]),
                                 int(arguments.get("read_ms", 500)))

    # -- Navigation --

    elif name == "approach":
        err = _require_navigator()
        if err:
            result = err
        else:
            target = arguments["target"]
            result = await asyncio.to_thread(
                _navigator.approach,
                (float(target[0]), float(target[1])),
                camera_index=int(arguments.get("camera", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                tolerance_mm=float(arguments.get("tolerance_mm", 5)),
                timeout=float(arguments.get("timeout", 20)))

    elif name == "navigate_to":
        err = _require_navigator()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(
                _navigator.navigate,
                (float(arguments["x"]), float(arguments["y"])),
                camera_index=int(arguments.get("camera", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                timeout=float(arguments.get("timeout", 30)),
                forward_only=bool(arguments.get("forward_only", False)))

    elif name == "follow_path":
        err = _require_navigator()
        if err:
            result = err
        else:
            path = [(float(p[0]), float(p[1])) for p in arguments["path"]]
            result = await asyncio.to_thread(
                _navigator.follow_path,
                path,
                camera_index=int(arguments.get("camera_index", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                timeout=float(arguments.get("timeout", 30.0)),
                lookahead=float(arguments.get("lookahead", 15.0)),
                trackwidth=float(arguments.get("trackwidth", 9.0)),
                base_speed=float(arguments.get("base_speed", 40.0)),
                stop_dist=float(arguments.get("stop_dist", 5.0)),
                controller=str(arguments.get("controller", "pure_pursuit")))

    elif name == "visit_tags":
        err = _require_navigator()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(
                _navigator.visit_tags, arguments["tags"],
                camera_index=int(arguments.get("camera", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                per_tag_timeout=float(arguments.get("timeout", 15)))

    elif name == "read_pose":
        err = _require_navigator()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(
                _navigator.read_pose,
                camera_index=int(arguments.get("camera", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())))

    elif name == "tune":
        err = _require_navigator()
        if err:
            result = err
        else:
            updated = _navigator.params.update(
                **{k: float(v) for k, v in arguments.items()
                   if hasattr(_navigator.params, k)})
            result = {"params": _navigator.params.as_dict()}
            if updated:
                result["updated"] = updated

    elif name == "grab_at":
        err = _require_navigator()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(
                _navigator.grab_at,
                (float(arguments["x"]), float(arguments["y"])),
                camera_index=int(arguments.get("camera", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                timeout=float(arguments.get("timeout", 20)))

    elif name == "release_at":
        err = _require_navigator()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(
                _navigator.release_at,
                (float(arguments["x"]), float(arguments["y"])),
                camera_index=int(arguments.get("camera", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                timeout=float(arguments.get("timeout", 15)))

    elif name == "reset_camera":
        result = _navigator.reset_camera() if _navigator else {"error": "No navigator"}

    # -- Device discovery --

    elif name == "list_serial_ports":
        result = {"ports": list_serial_ports()}

    elif name == "probe_devices":
        result = await asyncio.to_thread(
            probe_devices, int(arguments.get("read_ms", 1200)))

    # -- Utility --

    elif name == "reload_nav":
        result = _reload_nav()

    elif name == "detect_objects":
        from aprilcam.stream import detect_objects as _detect_objects
        objects = await asyncio.to_thread(
            _detect_objects,
            camera=int(arguments.get("bw_camera", 3)),
            color_camera=int(arguments.get("color_camera", 2)),
            data_dir=os.path.join(os.path.dirname(__file__), "..", "..", "data"))
        result = [_object_record_to_dict(o) for o in objects]

    elif name == "aprilcam_help":
        import aprilcam
        result = aprilcam.help()

    elif name == "list_movie_sessions":
        root_dir = str(arguments.get("root_dir", movie.DEFAULT_MOVIE_ROOT))
        result = {"sessions": movie.list_movie_sessions(root_dir)}

    elif name == "save_movie_frames":
        result = await asyncio.to_thread(
            movie.save_movie_frames,
            camera=int(arguments.get("camera", 3)),
            duration_s=float(arguments.get("duration_s", 10)),
            max_frames=int(arguments.get("max_frames", 0)),
            root_dir=str(arguments.get("root_dir", movie.DEFAULT_MOVIE_ROOT)),
            session_name=(None if arguments.get("session_name") is None
                          else str(arguments.get("session_name"))),
            pixels_per_cm=float(arguments.get("pixels_per_cm", 8.0)),
            diff_threshold=float(arguments.get("diff_threshold", 2.0)),
            min_interval_ms=int(arguments.get("min_interval_ms", 0)),
            max_gap_s=float(arguments.get("max_gap_s", 1.0)),
            image_format=str(arguments.get("image_format", "jpg")),
            jpeg_quality=int(arguments.get("jpeg_quality", 90)),
        )

    elif name == "make_movie":
        result = await asyncio.to_thread(
            movie.make_movie,
            session=(None if arguments.get("session") is None
                     else str(arguments.get("session"))),
            root_dir=str(arguments.get("root_dir", movie.DEFAULT_MOVIE_ROOT)),
            output_path=(None if arguments.get("output_path") is None
                         else str(arguments.get("output_path"))),
            fps=float(arguments.get("fps", 15.0)),
            codec=str(arguments.get("codec", "mp4v")),
            use_annotated=bool(arguments.get("use_annotated", False)),
            overlay_tags=bool(arguments.get("overlay_tags", False)),
            overlay_frame_index=bool(arguments.get("overlay_frame_index", False)),
        )

    elif name == "log_record":
        record = arguments["record"]
        file_path = str(arguments.get("file",
                        os.path.join(os.path.dirname(__file__), "..", "..", "test", "tuning", "nav_calibration.jsonl")))
        result = log_record(file_path, record)

    # -- Path planning --

    elif name == "plan_path":
        raw_start = arguments["start"]
        raw_end = arguments["end"]
        raw_waypoints = arguments.get("waypoints") or []
        method = str(arguments.get("method", "bezier"))
        spacing_cm = float(arguments.get("spacing_cm", 1.0))
        tangent_frac = float(arguments.get("tangent_frac", 0.33))

        start = Pose(
            x=float(raw_start["x"]),
            y=float(raw_start["y"]),
            heading=float(raw_start["heading"]),
        )
        end = Pose(
            x=float(raw_end["x"]),
            y=float(raw_end["y"]),
            heading=float(raw_end["heading"]),
        )
        waypoints = [
            Waypoint(
                x=float(wp["x"]),
                y=float(wp["y"]),
                heading=float(wp["heading"]) if "heading" in wp and wp["heading"] is not None else None,
            )
            for wp in raw_waypoints
        ]

        try:
            path = build_path(method, start, end, waypoints,
                              spacing_cm=spacing_cm, tangent_frac=tangent_frac)
            result = path.to_dict()
        except KeyError as exc:
            result = {"error": str(exc)}

    elif name == "preview_path":
        path_dict = arguments["path"]
        raw_points = path_dict.get("points", [])
        points = [(float(p[0]), float(p[1])) for p in raw_points]
        stub = preview_mod.preview_polyline(points)
        result = {"status": "ok", "points": stub["points"]}

    elif name == "follow_pose_path":
        err = _require_navigator()
        if err:
            result = err
        else:
            raw_end = arguments["end_pose"]
            end_pose = Pose(
                x=float(raw_end["x"]),
                y=float(raw_end["y"]),
                heading=float(raw_end["heading"]),
            )
            raw_start = arguments.get("start_pose")
            start_pose = None
            if raw_start is not None:
                start_pose = Pose(
                    x=float(raw_start["x"]),
                    y=float(raw_start["y"]),
                    heading=float(raw_start["heading"]),
                )
            raw_waypoints = arguments.get("waypoints") or []
            waypoints = [
                Waypoint(
                    x=float(wp["x"]),
                    y=float(wp["y"]),
                    heading=(float(wp["heading"])
                             if "heading" in wp and wp["heading"] is not None
                             else None),
                )
                for wp in raw_waypoints
            ]
            result = await asyncio.to_thread(
                _navigator.follow_pose_path,
                end_pose=end_pose,
                start_pose=start_pose,
                waypoints=waypoints,
                method=str(arguments.get("method", "bezier")),
                preview=bool(arguments.get("preview", True)),
                camera_index=int(arguments.get("camera_index", 3)),
                robot_tag=int(arguments.get("robot_tag", _default_robot_tag())),
                timeout=float(arguments.get("timeout", 30.0)),
                lookahead=float(arguments.get("lookahead", 15.0)),
                trackwidth=float(arguments.get("trackwidth", 9.0)),
                base_speed=float(arguments.get("base_speed", 40.0)),
                stop_dist=float(arguments.get("stop_dist", 5.0)),
                controller=str(arguments.get("controller", "pure_pursuit")),
            )

    # -- OTOS sensor --

    elif name == "otos_init":
        err = _require_otos()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(_otos.init)

    elif name == "otos_calibrate":
        err = _require_otos()
        if err:
            result = err
        else:
            result = await asyncio.to_thread(_otos.calibrate_imu)

    elif name == "otos_align":
        err = _require_otos()
        if err:
            result = err
        else:
            from robot_radio.nav.pose_align import align_otos_to_camera
            camera_index = int(arguments.get("camera", 3))
            robot_tag = int(arguments.get("robot_tag", _default_robot_tag()))
            settle_frames = int(arguments.get("settle_frames", 5))
            timeout_s = float(arguments.get("timeout_s", 4.0))

            def _do_align():
                import aprilcam
                from robot_radio.sensors.odometry import Odometry
                field = aprilcam.Playfield.open(camera_index)
                odom = Odometry(field, robot_tag=robot_tag)
                try:
                    return align_otos_to_camera(
                        _otos, odom,
                        settle_frames=settle_frames,
                        timeout_s=timeout_s,
                    )
                finally:
                    field.close()

            result = await asyncio.to_thread(_do_align)

    elif name == "read_pose_fused":
        err = _require_otos()
        if err:
            result = err
        else:
            camera_index = int(arguments.get("camera", 3))
            robot_tag = int(arguments.get("robot_tag", _default_robot_tag()))

            def _do_fused():
                import aprilcam
                from robot_radio.sensors.odometry import Odometry
                field = aprilcam.Playfield.open(camera_index)
                odom = Odometry(field, robot_tag=robot_tag)
                try:
                    odom.update()
                    camera_pose = None
                    if odom.is_valid:
                        camera_pose = {"x": odom.x, "y": odom.y, "heading": odom.yaw}
                    otos_pose_obj = _otos.read_world_pose()
                    otos_pose = None
                    if otos_pose_obj is not None:
                        otos_pose = {
                            "x": otos_pose_obj.x,
                            "y": otos_pose_obj.y,
                            "heading": otos_pose_obj.heading,
                        }
                    # Source selection: prefer camera
                    if camera_pose is not None:
                        source = "camera"
                        fused = camera_pose
                    elif otos_pose is not None:
                        source = "otos"
                        fused = otos_pose
                    else:
                        source = "none"
                        fused = None
                    return {
                        "camera": camera_pose,
                        "otos": otos_pose,
                        "fused": fused,
                        "source": source,
                    }
                finally:
                    field.close()

            result = await asyncio.to_thread(_do_fused)

    else:
        result = {"error": f"Unknown tool: {name}"}

    text = json.dumps(result) if isinstance(result, (dict, list)) else str(result)
    return [types.TextContent(type="text", text=text)]


async def _main() -> None:
    async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream,
                         server.create_initialization_options())


def main():
    asyncio.run(_main())


if __name__ == "__main__":
    main()
