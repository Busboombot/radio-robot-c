"""bench_safety.py — re-export shim for robot_radio.testkit.safety.

BenchRun is now SafeRun.  All new code should import SafeRun directly::

    from robot_radio.testkit import SafeRun

Existing bench scripts that do ``from bench_safety import BenchRun`` continue
to work unchanged via this shim.  The full BenchRun API (proto or Nezha or
TestRobot, max_seconds, SIGINT stop, wall-clock cap) is preserved.
"""

from robot_radio.testkit.safety import (  # noqa: F401
    SafeRun as BenchRun,
    RobotSilentError,
    RunawayAbortError,
)
