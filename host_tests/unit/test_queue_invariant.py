"""
test_queue_invariant.py — N2 regression test (sprint 030-002).

Verifies that the CommandProcessor's queue pointer is non-null after the
sim is created (simulating the boot path) and remains non-null after a
safety-stop and its setQueue(nullptr)/restore dance in loopTickOnce.

Background
----------
main.cpp Phase 3 reassigns the CommandProcessor via move-assign:
    cmd = CommandProcessor(robot.buildCommandTable(&dbgCmd, &sched));
The temporary's _queue is nullptr, so the move-assign wiped the queue wired
in LoopScheduler's constructor (LoopScheduler.cpp:108).  run_blocks() now
re-wires the queue at its entry (mirroring the existing run_test() fix at
LoopScheduler.cpp:139) so the firmware always runs the queue path from boot.

The sim fixture does NOT do a Phase-3-style reassignment, so these tests
function as a structural canary: if a future refactor re-introduces that
pattern and forgets the re-wire, sim_get_queue_wired() will return 0 and
the test will fail before any firmware is flashed.

Why not test firmware main.cpp directly?
-----------------------------------------
main.cpp runs on the micro:bit and is not directly sim-testable.  The
closest testable seam is the SimHandle constructor (sim_api.cpp), which
mirrors LoopScheduler's constructor wiring exactly — cmd.setQueue(&_queue)
and robot.setMotionQueue(&_queue).  Asserting queue non-null here catches:
  1. Any CommandProcessor move-assign that silently clears _queue.
  2. Any future sim_api refactor that drops the queue wire.
Both would also break the firmware path for the same reason.
"""
import ctypes
import pytest


# ---------------------------------------------------------------------------
# 1. test_queue_wired_at_boot
#
# Immediately after sim_create(), before any command or tick, the
# CommandProcessor's queue pointer must be non-null.
# This mirrors the invariant that run_blocks() now establishes at its entry
# (the Phase-3-reassignment fix): the queue is always armed before any
# command is dispatched.
# ---------------------------------------------------------------------------

def test_queue_wired_at_boot(sim):
    """CommandProcessor has its queue attached from the first tick (boot invariant)."""
    assert sim.get_queue_wired(), (
        "cmd._queue is nullptr immediately after sim_create() — "
        "the Phase-3 move-assign bug has regressed (N2): "
        "CommandProcessor was re-assigned and its queue pointer was cleared."
    )


# ---------------------------------------------------------------------------
# 2. test_queue_wired_after_command
#
# After sending a command (which calls cmd.process()), the queue pointer
# must still be non-null.  This verifies that process() itself does not
# accidentally clear the queue.
# ---------------------------------------------------------------------------

def test_queue_wired_after_command(sim):
    """Queue pointer remains non-null after dispatching a command."""
    sim.send_command("PING")
    assert sim.get_queue_wired(), (
        "cmd._queue became nullptr after a PING command — "
        "process() must not clear the queue pointer."
    )


# ---------------------------------------------------------------------------
# 3. test_queue_wired_after_safety_stop
#
# The watchdog path in loopTickOnce does:
#   cmd.setQueue(nullptr); cmd.process("X"); cmd.setQueue(&queue);
# After a safety stop, the restore must put the queue back.  This is
# consistent because run_blocks() now arms the queue first — the restore
# restores an actually-armed pointer, not nullptr.
#
# Pre-fix (N2): the firmware would flip from immediate-dispatch to queue
# mode on the first safety stop, because the restore armed a pointer that
# was previously nullptr.  Post-fix: the queue is always armed, and the
# restore just reinstates the same pointer.
# ---------------------------------------------------------------------------

def test_queue_wired_after_safety_stop(sim):
    """Queue pointer is non-null after a simulated safety-stop (no mode flip)."""
    # Enable a short watchdog timeout so the safety-stop fires quickly.
    sim.send_command("SET sTimeout=200")

    # Issue an open-ended S command (no TIME stop → watchdog fires when silent).
    sim.send_command("S 200 200")

    # Tick for 1 s without keepalives — well past the 200 ms window.
    for _ in range(1000 // 24):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24

    evts = sim.get_async_evts()

    # Confirm the safety-stop actually fired.
    assert "EVT safety_stop" in evts, (
        f"Safety-stop did not fire as expected; got: {repr(evts)}"
    )

    # Queue must still be attached after the setQueue(nullptr)/restore dance.
    assert sim.get_queue_wired(), (
        "cmd._queue is nullptr after safety_stop — "
        "the setQueue(nullptr)/restore dance in loopTickOnce left the queue "
        "detached (or run_blocks() never armed it, so restoring nullptr → nullptr "
        "looks correct but isn't the queue path).  This is the N2 mode-flip bug."
    )


# ---------------------------------------------------------------------------
# 4. test_no_mode_flip_queue_dispatch_consistent
#
# Verify that the dispatch path is the queue path both before and after a
# safety stop: a converter command (D) completes after a safety-stop and
# re-arm, proving that the queue-path converters (push_front VW) work
# throughout the session without a mode flip.
# ---------------------------------------------------------------------------

def test_no_mode_flip_queue_dispatch_consistent(sim):
    """Queue-path dispatch (D command) works before and after a safety-stop."""
    sim.send_command("SET sTimeout=200")

    # Phase A: D command before any safety-stop.
    sim.send_command("D 200 200 100")
    sim.tick_for(3000)
    evts_a = sim.get_async_evts()
    assert "EVT done D" in evts_a, (
        f"Phase A: D command did not complete before safety-stop; "
        f"got: {repr(evts_a)}"
    )

    # Trigger a safety-stop.
    sim.send_command("S 200 200")
    sim.tick_for(600)
    evts_stop = sim.get_async_evts()
    assert "EVT safety_stop" in evts_stop, (
        f"Safety-stop did not fire as expected; got: {repr(evts_stop)}"
    )

    # Re-arm the watchdog and reset timeout to a safe long value.
    sim.send_command("SET sTimeout=60000")
    sim.send_command("PING")  # resets watchdog timestamp

    # Phase B: D command after safety-stop — queue path must still work.
    sim.send_command("D 200 200 100")
    sim.tick_for(3000)
    evts_b = sim.get_async_evts()
    assert "EVT done D" in evts_b, (
        f"Phase B: D command did not complete after safety-stop — "
        f"mode flip may have broken the queue-path converters; "
        f"got: {repr(evts_b)}"
    )
