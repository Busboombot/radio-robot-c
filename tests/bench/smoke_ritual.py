"""smoke_ritual.py — sprint acceptance smoke ritual for radio-robot-c.

Run after a clean firmware flash (mbdeploy deploy robot --clean) to confirm
the refactored firmware behaves correctly on the real robot.

Five ritual steps (run in order):

  1. Safety check   — SAFE query must return 'on'.
  2. TURN ×4 closure — four sequential TURN 9000 commands; robot must return
                       within 15° of starting heading (OTOS before = after ±15°).
  3. G square       — drive G to each of four corners of a 300×300 mm square;
                       return to origin; position error < 100 mm from OTOS.
  4. No double-OK   — assert no '#id' correlation tag appears twice in the same
                       reply burst during the G square run.
  5. Stream aliveness — STREAM 40; run T 2000; stream must not go silent during
                        the drive (EVT done T arrives and stream continues).

Usage:
    uv run python tests/bench/smoke_ritual.py --port /dev/cu.usbmodem2121302
    uv run python tests/bench/smoke_ritual.py          # auto-detects relay port

Outputs a summary table of pass/fail per step.  Any FAIL exits non-zero.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import pathlib

_REPO = pathlib.Path(__file__).resolve().parents[2]
_HOST = _REPO / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_response, parse_tlm


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

STEP_PASS = "PASS"
STEP_FAIL = "FAIL"
STEP_SKIP = "SKIP"


def _banner(title: str) -> None:
    width = 60
    print()
    print("=" * width)
    print(f"  {title}")
    print("=" * width)


def _result(step: int, name: str, status: str, note: str = "") -> None:
    tick = "+" if status == STEP_PASS else ("~" if status == STEP_SKIP else "X")
    extra = f"  ({note})" if note else ""
    print(f"  [{tick}] Step {step}: {name} — {status}{extra}")


def _wrap_deg(a: float) -> float:
    """Wrap angle (degrees) to [-180, 180]."""
    return math.degrees(math.atan2(math.sin(math.radians(a)),
                                   math.cos(math.radians(a))))


def _otos_heading_deg(proto: NezhaProtocol) -> float | None:
    """Read OTOS heading in degrees via SNAP."""
    frame = proto.snap()
    if frame is None or frame.otos is None:
        return None
    return frame.otos[2] / 100.0  # cdeg -> deg


def _otos_pos_mm(proto: NezhaProtocol) -> tuple[int, int] | None:
    """Read OTOS x,y position in mm via SNAP."""
    frame = proto.snap()
    if frame is None or frame.otos is None:
        return None
    return frame.otos[0], frame.otos[1]


def _keepalive_thread(proto: NezhaProtocol, stop: list[bool]) -> None:
    """Background keepalive sender (+ every 200 ms)."""
    while not stop[0]:
        proto.send_fast("+")
        time.sleep(0.2)


# ---------------------------------------------------------------------------
# Step 1: Safety check
# ---------------------------------------------------------------------------

def step1_safety_check(proto: NezhaProtocol) -> str:
    """SAFE query must return 'on'."""
    _banner("Step 1: Safety check (SAFE query)")
    resp = proto.send("SAFE", read_ms=400)
    for line in resp.get("responses", []):
        r = parse_response(line)
        if r and r.tag == "OK" and "safety" in r.tokens:
            idx = r.tokens.index("safety")
            if idx + 1 < len(r.tokens):
                state = r.tokens[idx + 1]
                print(f"  SAFE query response: {line.strip()}")
                if state == "on":
                    print("  Safety watchdog is ON.")
                    return STEP_PASS
                else:
                    print(f"  Safety watchdog is '{state}' — expected 'on'.")
                    return STEP_FAIL
    print(f"  No parseable SAFE reply. Raw responses: {resp.get('responses', [])}")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Step 2: TURN ×4 closure
# ---------------------------------------------------------------------------

def step2_turn_closure(proto: NezhaProtocol) -> str:
    """Four sequential TURN 9000 commands; heading must close to ±15°."""
    _banner("Step 2: TURN ×4 closure")
    TURN_HEADING = 9000   # centidegrees = 90°
    TURN_TIMEOUT = 15_000  # ms per turn
    CLOSURE_TOL_DEG = 15.0

    h0 = _otos_heading_deg(proto)
    if h0 is None:
        print("  Cannot read starting OTOS heading — is OTOS enabled?")
        return STEP_FAIL
    print(f"  Starting heading: {h0:.1f}°")

    for i in range(4):
        print(f"  TURN {TURN_HEADING} cdeg (turn {i+1}/4) ...")
        proto.turn(TURN_HEADING, corr_id=str(i + 1))
        outcome = proto.wait_for_evt_done("TURN", timeout_ms=TURN_TIMEOUT,
                                          corr_id=str(i + 1))
        if outcome != "done":
            print(f"  TURN {i+1} outcome: {outcome} (expected 'done')")
            return STEP_FAIL
        time.sleep(0.3)

    h1 = _otos_heading_deg(proto)
    if h1 is None:
        print("  Cannot read final OTOS heading.")
        return STEP_FAIL
    print(f"  Final heading:    {h1:.1f}°")

    delta = abs(_wrap_deg(h1 - h0))
    print(f"  Heading closure error: {delta:.1f}° (tolerance: {CLOSURE_TOL_DEG}°)")
    if delta <= CLOSURE_TOL_DEG:
        return STEP_PASS
    print(f"  FAIL — closure error {delta:.1f}° exceeds {CLOSURE_TOL_DEG}°")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Step 3: G square + Step 4: no double-OK
# ---------------------------------------------------------------------------

# 300×300 mm square corners (relative, starting at origin)
_SQUARE_CORNERS = [
    (150, 150),    # NE
    (-150, 150),   # NW
    (-150, -150),  # SW
    (150, -150),   # SE
    (0, 0),        # back to origin
]

SQUARE_SPEED = 150      # mm/s
SQUARE_TIMEOUT = 20_000  # ms per leg
SQUARE_ARRIVE_MM = 100  # OTOS position error tolerance at origin


def step3_4_g_square(proto: NezhaProtocol) -> tuple[str, str]:
    """Drive G square; return (step3_status, step4_status).

    Step 3: position error at origin < 100 mm.
    Step 4: no '#id' correlation tag repeated in the same reply burst.
    """
    _banner("Step 3+4: G square (300×300 mm) + no double-OK check")

    # Zero OTOS before the run so origin = start.
    proto.zero_otos()
    time.sleep(0.3)
    print("  OTOS zeroed at start position.")

    all_reply_lines: list[str] = []
    ok_ids_per_burst: list[list[str | None]] = []

    for i, (x, y) in enumerate(_SQUARE_CORNERS):
        label = f"({x:+d},{y:+d})" if (x, y) != (0, 0) else "origin"
        print(f"  G {x} {y} {SQUARE_SPEED}  -> {label} ...")
        resp = proto.send(f"G {x} {y} {SQUARE_SPEED}", read_ms=300)
        burst_lines = resp.get("responses", [])
        all_reply_lines.extend(burst_lines)

        # Collect corr_ids from this burst for double-OK check.
        burst_ids: list[str | None] = []
        for line in burst_lines:
            r = parse_response(line)
            if r and r.tag == "OK":
                burst_ids.append(r.corr_id)
        ok_ids_per_burst.append(burst_ids)

        # Wait for EVT done G.
        outcome = proto.wait_for_evt_done("G", timeout_ms=SQUARE_TIMEOUT)
        if outcome != "done":
            print(f"  G to {label}: outcome={outcome} (expected 'done')")
            return STEP_FAIL, STEP_SKIP
        time.sleep(0.3)

    # --- Step 3: check OTOS position at origin ---
    pos = _otos_pos_mm(proto)
    if pos is None:
        print("  Cannot read final OTOS position.")
        step3 = STEP_FAIL
    else:
        err_mm = math.hypot(pos[0], pos[1])
        print(f"  Final OTOS position: x={pos[0]} mm, y={pos[1]} mm")
        print(f"  Position error from origin: {err_mm:.0f} mm"
              f"  (tolerance: {SQUARE_ARRIVE_MM} mm)")
        if err_mm <= SQUARE_ARRIVE_MM:
            step3 = STEP_PASS
        else:
            print(f"  FAIL — position error {err_mm:.0f} mm > {SQUARE_ARRIVE_MM} mm")
            step3 = STEP_FAIL

    # --- Step 4: no double-OK (same #id twice in same burst) ---
    step4 = STEP_PASS
    for burst_i, ids in enumerate(ok_ids_per_burst):
        seen: set[str] = set()
        for cid in ids:
            if cid is None:
                continue
            if cid in seen:
                print(f"  FAIL — duplicate corr_id #{cid} in burst {burst_i}")
                step4 = STEP_FAIL
                break
            seen.add(cid)
    if step4 == STEP_PASS:
        print("  No duplicate OK #id found in any reply burst.")

    return step3, step4


# ---------------------------------------------------------------------------
# Step 5: Stream aliveness during T drive
# ---------------------------------------------------------------------------

def step5_stream_aliveness(proto: NezhaProtocol) -> str:
    """STREAM 40; run T 2000; verify stream continues after EVT done T."""
    _banner("Step 5: Stream aliveness")
    STREAM_PERIOD_MS = 40
    T_DURATION_MS = 2000
    T_SPEED = 150           # mm/s — moderate forward speed
    T_TIMEOUT_MS = 5000     # wait for EVT done T
    POST_T_WAIT_MS = 600    # time to check for continued stream after done T
    STREAM_SILENCE_TOL = 3  # expect at least this many TLM frames post-done

    proto.stream(STREAM_PERIOD_MS)
    time.sleep(0.15)  # let first frames arrive

    print(f"  STREAM {STREAM_PERIOD_MS} ms enabled.")
    print(f"  Sending T {T_SPEED} {T_SPEED} {T_DURATION_MS} ...")
    proto.timed(T_SPEED, T_SPEED, T_DURATION_MS)

    # Wait for EVT done T.
    outcome = proto.wait_for_evt_done("T", timeout_ms=T_TIMEOUT_MS)
    print(f"  EVT done T outcome: {outcome}")
    if outcome != "done":
        proto.stop()
        proto.stream(0)
        print(f"  FAIL — EVT done T not received (outcome={outcome})")
        return STEP_FAIL

    # Check that TLM stream continues after EVT done T.
    post_lines = proto.read_lines(duration_ms=POST_T_WAIT_MS)
    tlm_count = sum(1 for ln in post_lines if parse_tlm(ln) is not None)
    print(f"  TLM frames received in {POST_T_WAIT_MS} ms post-done: {tlm_count}"
          f"  (need >= {STREAM_SILENCE_TOL})")

    proto.stream(0)  # stop streaming

    if tlm_count >= STREAM_SILENCE_TOL:
        return STEP_PASS
    print(f"  FAIL — stream went silent after EVT done T"
          f" ({tlm_count} frames, need {STREAM_SILENCE_TOL})")
    return STEP_FAIL


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="relay serial port (auto-detects if omitted)")
    ap.add_argument("--steps", default="1,2,3,4,5",
                    help="comma-separated step numbers to run (default: all)")
    args = ap.parse_args()

    steps_to_run = {int(s.strip()) for s in args.steps.split(",")}

    # Connect and preflight.
    print("Connecting to robot ...")
    conn = SerialConnection(args.port) if args.port else SerialConnection()
    res = conn.connect()
    if res.get("error"):
        sys.exit(f"Connection failed: {res['error']}")
    proto = NezhaProtocol(conn)

    png = proto.ping()
    if not png:
        proto.stop()
        sys.exit("PING failed — robot not responding. Power-cycle and retry.")
    print(f"Robot alive: t={png[0]} ms, rtt={png[1]:.0f} ms")

    results: dict[int, str] = {}
    step_names = {
        1: "Safety check",
        2: "TURN ×4 closure",
        3: "G square",
        4: "No double-OK",
        5: "Stream aliveness",
    }

    try:
        if 1 in steps_to_run:
            results[1] = step1_safety_check(proto)

        if 2 in steps_to_run:
            results[2] = step2_turn_closure(proto)

        if 3 in steps_to_run or 4 in steps_to_run:
            s3, s4 = step3_4_g_square(proto)
            if 3 in steps_to_run:
                results[3] = s3
            if 4 in steps_to_run:
                results[4] = s4

        if 5 in steps_to_run:
            results[5] = step5_stream_aliveness(proto)

    finally:
        # Always safe-stop the robot on exit (normal or exception).
        print()
        print("[safe-stop] Sending STOP + STREAM 0 ...")
        proto.stop()
        proto.stream(0)
        conn.close()

    # Summary.
    _banner("Smoke Ritual Summary")
    overall = STEP_PASS
    for step_num in sorted(step_names):
        if step_num not in steps_to_run:
            status = STEP_SKIP
        else:
            status = results.get(step_num, STEP_SKIP)
        _result(step_num, step_names[step_num], status)
        if status == STEP_FAIL:
            overall = STEP_FAIL

    print()
    if overall == STEP_PASS:
        print("OVERALL: PASS — all steps passed.")
        sys.exit(0)
    else:
        print("OVERALL: FAIL — one or more steps failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
