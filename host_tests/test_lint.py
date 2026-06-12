"""test_lint.py — static source-code lint checks for the radio-robot-c project.

Checks that are fast, grep-based, and should never regress.
"""
from __future__ import annotations

import pathlib
import subprocess


_REPO = pathlib.Path(__file__).resolve().parents[1]
_SOURCE = _REPO / "source"
_HOST_TESTS = _REPO / "host_tests"


def test_no_must_mirror_comment() -> None:
    """Assert that no 'MUST mirror' comment exists in source/ or host_tests/.

    The pattern 'MUST mirror' flags hand-mirrored code that must stay in sync
    between the sim and the firmware.  Sprint 026-001 removed the last such
    instance; this test prevents the pattern from being reintroduced.
    """
    # Exclude the lint test file itself (the pattern appears in its own docstring
    # and grep argument) and compiled __pycache__ bytecode.  All other files
    # under source/ and host_tests/ are scanned.
    result = subprocess.run(
        [
            "grep", "-rn",
            "--exclude=test_lint.py",
            "--exclude-dir=__pycache__",
            "MUST mirror",
            str(_SOURCE),
            str(_HOST_TESTS),
        ],
        capture_output=True,
        text=True,
    )
    matches = result.stdout.strip()
    assert matches == "", (
        "Found 'MUST mirror' comment(s) — remove the hand-mirrored pattern "
        "and replace with a shared abstraction:\n" + matches
    )
