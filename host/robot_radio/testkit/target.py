"""robot_radio.testkit.target — TestRobot dataclass and make_target factory.

Provides a single entry point for constructing a target-appropriate Nezha
instance (sim, bench, or production) along with its connection, optional
playfield, pose source, and metadata.

Usage::

    from robot_radio.testkit import make_target, TestRobot

    # Sim (no hardware): SimConnection drives libfirmware_host.
    tr = make_target("sim")

    # Bench (real robot, sim OTOS enabled for bench calibration):
    tr = make_target("bench", port="/dev/cu.usbmodem...")

    # Production (real OTOS, optional camera pose):
    tr = make_target("production", port="/dev/cu.usbmodem...", camera="arducam-ov9782")

All imports of optional dependencies (aprilcam, daemon) are deferred so that
``import robot_radio.testkit.target`` works without a live camera daemon.
"""

from __future__ import annotations

import types
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from robot_radio.robot.nezha import Nezha
    from robot_radio.field.playfield import Playfield
    from robot_radio.testkit.pose import PoseSource


@dataclass
class TestRobot:
    """Container returned by make_target.

    Fields
    ------
    robot:
        Connected Nezha driver.
    conn:
        Underlying connection (SimConnection or SerialConnection).
    playfield:
        Open Playfield instance, or None when no camera was requested.
    pose:
        PoseSource for this target (FirmwarePose or CameraPose).
    target:
        One of "sim", "bench", "production".
    real_time:
        Whether real-time pacing was requested (sim only; ignored for
        hardware targets).
    """

    robot: "Nezha"
    conn: Any
    playfield: "Playfield | None"
    pose: "PoseSource"
    target: str
    real_time: bool = False


def make_target(
    target: str,
    *,
    real_time: bool = False,
    sim_otos: bool | None = None,
    port: str | None = None,
    camera: str | None = None,
    config: Any = None,
) -> TestRobot:
    """Construct and connect a target-appropriate Nezha robot.

    Parameters
    ----------
    target:
        One of "sim", "bench", or "production".
    real_time:
        (sim only) When True, sim ticks sleep to match wall-clock time.
        Ignored for hardware targets.  Has no effect until SimConnection
        gains the real_time parameter (ticket 002); passed through
        forward-compatibly.
    sim_otos:
        Override the per-target OTOS mode.  None (default) uses the
        per-target default:
          - "sim"        → True  (DBG OTOS BENCH 1)
          - "bench"      → True  (DBG OTOS BENCH 1)
          - "production" → False (real OTOS)
        Explicit True/False overrides regardless of target.
    port:
        Serial port string for bench/production targets.  None = auto-detect.
    camera:
        Camera name (as known to the aprilcam daemon) for production targets
        that want camera-based pose.  None = firmware pose.
    config:
        Reserved for future use (config overrides).

    Returns
    -------
    TestRobot
        A populated TestRobot with a connected robot, conn, optional playfield,
        and pose source appropriate to the target.

    Raises
    ------
    ValueError
        If target is not one of "sim", "bench", "production".
    """
    target = target.lower()
    if target not in ("sim", "bench", "production"):
        raise ValueError(
            f"target must be 'sim', 'bench', or 'production'; got {target!r}"
        )

    # ------------------------------------------------------------------ #
    # Per-target sim_otos default                                          #
    # ------------------------------------------------------------------ #
    if sim_otos is None:
        sim_otos = target in ("sim", "bench")

    # ------------------------------------------------------------------ #
    # sim target                                                           #
    # ------------------------------------------------------------------ #
    if target == "sim":
        from robot_radio.io.sim_conn import SimConnection
        from robot_radio.robot.protocol import NezhaProtocol
        from robot_radio.robot.nezha import Nezha
        from robot_radio.testkit.pose import FirmwarePose

        # Pass real_time forward-compatibly: SimConnection does not yet have
        # this param (ticket 002 adds it).  We accept it here so callers can
        # pass it, and wire it in once available.
        try:
            conn = SimConnection(real_time=real_time)
        except TypeError:
            # SimConnection does not yet accept real_time — ignore for now.
            conn = SimConnection()

        result = conn.connect()
        if "error" in result:
            raise RuntimeError(
                f"SimConnection.connect() failed: {result['error']}\n"
                "Make sure the firmware sim library is built: "
                "cmake -S host_tests -B host_tests/build && "
                "cmake --build host_tests/build"
            )

        proto = NezhaProtocol(conn)
        robot = Nezha(proto)

        if sim_otos:
            conn.send("DBG OTOS BENCH 1", 200)

        pose = FirmwarePose(robot)
        return TestRobot(
            robot=robot,
            conn=conn,
            playfield=None,
            pose=pose,
            target=target,
            real_time=real_time,
        )

    # ------------------------------------------------------------------ #
    # bench / production targets                                           #
    # ------------------------------------------------------------------ #
    from robot_radio.robot.connection import make_robot

    # make_robot requires an args namespace with a .port attribute.
    args = types.SimpleNamespace(port=port)
    robot, conn, _result = make_robot(
        port=port,
        mode=None,
        verbose=False,
        args=args,
    )

    if sim_otos:
        robot._proto.send("DBG OTOS BENCH 1", 200)

    # Open playfield lazily (only when a camera name is requested and we are
    # on the production target path).
    playfield = None
    if camera is not None:
        from robot_radio.field.playfield import Playfield  # aprilcam import deferred

        playfield = Playfield.open(camera)

    # Determine pose source.
    from robot_radio.testkit.pose import FirmwarePose, CameraPose

    if target == "production" and playfield is not None:
        pose: PoseSource = CameraPose(playfield, tag_id=100)
    else:
        pose = FirmwarePose(robot)

    return TestRobot(
        robot=robot,
        conn=conn,
        playfield=playfield,
        pose=pose,
        target=target,
        real_time=real_time,
    )
