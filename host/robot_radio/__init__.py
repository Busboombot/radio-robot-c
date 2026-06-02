"""robot_radio — robot controller package for micro:bit robots.

Core modules (no optional dependencies required):
  robot_radio.robot.protocol  — v2 wire protocol encode/parse
  robot_radio.robot.nezha     — Nezha robot driver
  robot_radio.robot.nezha_state — NezhaState hardware state manager
  robot_radio.io.serial_conn  — serial connection management

Optional modules (require additional dependencies):
  robot_radio.config          — robot configuration (pydantic)
  robot_radio.nav             — navigation stack (numpy, wpimath)
  robot_radio.sensors         — sensor models (numpy)
  robot_radio.controllers     — controllers (numpy, wpimath)
  robot_radio.kinematics      — kinematics (wpimath)
"""


def __getattr__(name: str):
    """Lazy import all submodule symbols to avoid hard dependency at import time."""
    if name == "SerialConnection":
        from robot_radio.io.serial_conn import SerialConnection
        return SerialConnection
    if name == "NezhaProtocol":
        from robot_radio.robot.protocol import NezhaProtocol
        return NezhaProtocol
    if name == "Nezha":
        from robot_radio.robot.nezha import Nezha
        return Nezha
    if name == "Robot":
        from robot_radio.robot.robot import Robot
        return Robot
    if name == "get_robot_config":
        from robot_radio.config import get_robot_config
        return get_robot_config
    if name == "Odometry":
        from robot_radio.sensors import Odometry
        return Odometry
    if name in ("NavParams", "Pose"):
        import robot_radio.nav as _nav
        return getattr(_nav, name)
    if name == "PID":
        from robot_radio.controllers.pid import PID
        return PID
    if name == "Navigator":
        from robot_radio.nav.navigator import Navigator
        return Navigator
    if name == "PurePursuitTracker":
        from robot_radio.controllers.pure_pursuit import PurePursuitTracker
        return PurePursuitTracker
    if name == "QBotPro":
        from robot_radio.robot.cutebot import QBotPro
        return QBotPro
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
