"""robot_radio.robot — hardware driver subpackage (protocol v2).

Core exports (no optional dependencies):
  Robot, NezhaProtocol, TLMFrame, ParsedResponse, parse_response, parse_tlm,
  parse_cfg, Nezha, NezhaState.

Optional exports (require wpimath):
  NezhaKinematic

Optional exports (may require wpimath, numpy):
  Cutebot, QBotPro
"""

from robot_radio.robot.robot import Robot
from robot_radio.robot.protocol import (
    NezhaProtocol,
    TLMFrame,
    ParsedResponse,
    parse_response,
    parse_tlm,
    parse_cfg,
)
from robot_radio.robot.nezha import Nezha
from robot_radio.robot.nezha_state import NezhaState
from robot_radio.robot.clock_sync import ClockSync


def __getattr__(name: str):
    """Lazy import for optional-dependency submodules."""
    if name == "NezhaKinematic":
        from robot_radio.robot.nezha_kinematic import NezhaKinematic
        return NezhaKinematic
    if name in ("Cutebot", "QBotPro"):
        from robot_radio.robot import cutebot as _cutebot
        return getattr(_cutebot, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "Robot",
    "NezhaProtocol",
    "TLMFrame",
    "ParsedResponse",
    "parse_response",
    "parse_tlm",
    "parse_cfg",
    "Nezha",
    "NezhaState",
    "ClockSync",
    "NezhaKinematic",    # lazy
    "Cutebot",           # lazy
    "QBotPro",           # lazy
]
