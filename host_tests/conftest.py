"""
conftest.py — pytest fixtures for host simulation tests (ticket 020-004).

build_lib  — session-scoped, autouse: cmake configure + build before any test.
sim        — function-scoped: creates one Sim per test and destroys it after.
"""
import pathlib
import subprocess
import sys

import pytest

_HOST_TESTS = pathlib.Path(__file__).parent
_BUILD_DIR = _HOST_TESTS / "build"

# Tests live in unit/ (and other subdirs); make the shared helper modules in
# this directory (firmware.py, etc.) importable from anywhere in the tree.
if str(_HOST_TESTS) not in sys.path:
    sys.path.insert(0, str(_HOST_TESTS))


@pytest.fixture(scope="session", autouse=True)
def build_lib():
    """Configure and build libfirmware_host if needed."""
    _BUILD_DIR.mkdir(exist_ok=True)
    subprocess.run(
        ["cmake", "-S", str(_HOST_TESTS), "-B", str(_BUILD_DIR)],
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(_BUILD_DIR)],
        check=True,
    )


@pytest.fixture
def sim(build_lib):
    """Create a fresh Sim instance for the test; destroy it afterwards.

    The keepalive watchdog (sTimeoutMs, default 500 ms) is extended to 60 s
    so that tests which don't re-send keepalives don't hit the safety-stop.
    Tests that explicitly want to test watchdog behaviour should override via
    ``sim.send_command("SET sTimeout=<value>")``.
    """
    # Import here so the module is only loaded after build_lib runs.
    from firmware import Sim  # noqa: PLC0415

    with Sim() as s:
        # Extend watchdog timeout so single-shot S/VW commands don't trigger
        # the safety-stop during tick_for loops that don't send keepalives.
        s.send_command("SET sTimeout=60000")
        yield s


@pytest.fixture
def sim_field_profile(build_lib):
    """Create a Sim pre-configured with the field profile (turn slip + OTOS fusion).

    Mirrors the ``sim`` fixture but additionally applies the field profile:
    turn-slip over-report (slipTurnExtra = 0.26) and OTOS EKF fusion enabled.
    Use this fixture for tests that must pass under field conditions.

    The watchdog is extended to 60 s identically to the ``sim`` fixture.
    """
    from firmware import Sim  # noqa: PLC0415

    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
        yield s
