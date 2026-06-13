"""
test_n7_queue_overflow_err.py — N7 regression tests (sprint 030-005).

N7 finding: CommandProcessor::dispatchTable() silently dropped commands when
the 4-slot CommandQueue was full — no ERR reply, the host just timed out.
Converter handlers (S, T, D, G, R, TURN, RT) also replied OK *before*
pushVW(), so a failed enqueue left the host believing motion had started when
the VW was never queued.

Fixes (030-005):
  1. dispatchTable(): check push_back() return → ERR full on overflow.
  2. Converter handlers: suppress OK until pushVW() succeeds; ERR full on fail.

Tests
-----
test_queue_overflow_produces_err_full
    Fill the 4-slot queue to capacity with dummy entries (sim_fill_queue),
    then send a real command via sim_command_no_drain (no post-process drain).
    The 5th enqueue must return ERR full, not silently succeed.

test_converter_full_queue_no_bare_ok
    Fill the queue to capacity, then send a converter command (D) via
    sim_command_no_drain.  Verify:
      - The reply is ERR, not OK.
      - No bare OK appears in the synchronous reply.
    This proves the converter does NOT emit OK before the failed pushVW.

test_converter_success_replies_ok_after_queue_drain
    Normal path: send a converter command when the queue is NOT full.
    Verify OK is still returned (sanity check that the fix didn't break
    the success path).

test_burst_5_commands_5th_gets_err
    Simulate a host burst by filling the queue to capacity (via
    sim_fill_queue), then sending a 5th command without draining.
    The 5th must get ERR full (not silence or OK).

test_non_motion_command_also_errors_on_full_queue
    A non-converter command (PING) routed through dispatchTable() on a full
    queue must also get ERR full — the fix is in dispatchTable(), not just in
    converter handlers.
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _first_token(reply: str) -> str:
    """Return the first whitespace-delimited token of the first non-empty line."""
    for line in reply.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped.split()[0]
    return ""


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_queue_overflow_produces_err_full(sim):
    """dispatchTable() replies ERR full when push_back() fails (queue at capacity).

    Setup: fill the 4-slot queue with dummy entries, then send one more
    command without draining.  The reply must contain ERR (not OK or silence).
    """
    pushed = sim.fill_queue()
    assert pushed == 4, (
        f"Expected fill_queue() to push 4 dummy items (queue capacity), "
        f"got {pushed}"
    )
    assert sim.queue_size() == 4, (
        f"Queue should be full (size 4) after fill_queue(), "
        f"got {sim.queue_size()}"
    )

    # PING is a non-converter command — it still goes through dispatchTable()
    # and must fail with ERR full.
    reply = sim.send_command_no_drain("PING #1")
    first = _first_token(reply)

    assert first == "ERR", (
        f"Expected ERR full when queue is full, got {repr(reply.strip())}"
    )
    assert "full" in reply, (
        f"Expected 'full' in ERR reply, got {repr(reply.strip())}"
    )


def test_burst_5_commands_5th_gets_err(sim):
    """Simulated host burst: fill queue then send 5th command — 5th gets ERR full.

    This is the exact burst scenario from N7: a host fires 5 commands rapidly;
    on hardware the queue drains 1/tick (~10-25 ms) so a burst overflows.
    In sim we reproduce it by filling the queue then skipping the drain.
    """
    sim.fill_queue()
    reply = sim.send_command_no_drain("SET vel.kP=1.5 #5")
    first = _first_token(reply)

    assert first == "ERR", (
        f"5th command into a full queue must get ERR full, "
        f"got {repr(reply.strip())}"
    )
    assert "full" in reply, (
        f"Expected 'full' token in ERR reply, got {repr(reply.strip())}"
    )


def test_non_motion_command_also_errors_on_full_queue(sim):
    """Non-converter commands (HELLO) also get ERR full when the queue is full.

    The fix is in dispatchTable() itself, which is shared by all command
    descriptors.  Verify it applies to non-motion commands too.
    """
    sim.fill_queue()
    reply = sim.send_command_no_drain("HELLO #9")
    first = _first_token(reply)

    assert first == "ERR", (
        f"HELLO on a full queue must return ERR full, "
        f"got {repr(reply.strip())}"
    )


def test_converter_full_queue_no_bare_ok(sim):
    """Converter (D) on a full queue returns ERR — NOT a bare OK.

    N7 core concern: the converter replied OK *before* pushVW(), so a dropped
    VW left the host believing motion had started.  After the fix, if pushVW()
    fails the converter must NOT emit OK.

    Setup: fill the queue, then send a D command without draining.
    Assert the synchronous reply starts with ERR, not OK.
    """
    sim.fill_queue()
    reply = sim.send_command_no_drain("D 200 200 100 #42")
    first = _first_token(reply)

    assert first == "ERR", (
        f"Converter D on a full queue must return ERR, "
        f"got {repr(reply.strip())}"
    )
    assert "OK" not in reply, (
        f"Converter D must NOT emit OK when pushVW fails (N7), "
        f"got {repr(reply.strip())}"
    )
    assert "full" in reply, (
        f"Expected 'full' in ERR reply from converter D, "
        f"got {repr(reply.strip())}"
    )


def test_converter_success_replies_ok_after_queue_drain(sim):
    """Normal path: converter D replies OK when the queue has room.

    Sanity check that the suppress-OK fix did not break the success path:
    a D command on a non-full queue must still return OK drive ...
    """
    # Queue is empty at fixture start; normal sim_command drains after dispatch.
    reply = sim.send_command("D 200 200 100 #77")
    first = _first_token(reply)

    assert first == "OK", (
        f"Converter D must return OK on success, got {repr(reply.strip())}"
    )
    assert "drive" in reply or "OK" in reply, (
        f"Expected OK drive reply, got {repr(reply.strip())}"
    )


def test_all_converters_no_bare_ok_on_full_queue(sim):
    """All 7 converters (S T D G TURN RT R) return ERR — not OK — when queue is full.

    This is the systematic check: every converter that packs a VW must behave
    the same way.  We re-fill the queue before each command.
    """
    converters = [
        ("S 200 200",     "S"),
        ("T 200 200 500", "T"),
        ("D 200 200 100", "D"),
        ("G 400 300 200", "G"),
        ("TURN 9000",     "TURN"),
        ("RT 9000",       "RT"),
        ("R 200 200",     "R"),
    ]

    for cmd, label in converters:
        # Drain the queue (it may have leftovers from a previous iteration's
        # fill) by sending a no-op that gets dequeued.
        # Actually the fill_queue adds nullptrs that dequeueOne skips safely,
        # so we drain by sending a real command via sim_command (drains 2x).
        # But the queue is full — drain it manually by sending PING via normal
        # send_command which dequeues 2x but will return ERR (queue full, no
        # drain problem since the dummies are there).  Better: just refill.
        #
        # Simplest: use a fresh sim is too expensive; instead, after each
        # fill + no-drain call, the queue still has 4 dummies (the no-drain
        # command was rejected, so nothing was added or removed).  Just re-use
        # the existing fill for the next iteration — but we need to reset
        # between iterations.
        #
        # Approach: use send_command (which drains 2x) to discharge dummy items,
        # then refill.  On first iteration the queue was untouched; just fill it.
        pass

    # Run each converter in its own fresh fill cycle.
    for cmd, label in converters:
        # fill_queue only adds items up to capacity.  If the queue is already
        # partly drained from a prior normal send_command, top it up.
        sim.fill_queue()  # fills remaining slots to capacity

        reply = sim.send_command_no_drain(f"{cmd} #55")
        first = _first_token(reply)

        assert first == "ERR", (
            f"Converter {label} on a full queue must return ERR, "
            f"got {repr(reply.strip())}"
        )
        assert "OK" not in reply, (
            f"Converter {label} must NOT emit OK when pushVW fails (N7), "
            f"reply={repr(reply.strip())}"
        )

        # Drain the queue for the next iteration: send a throwaway normal
        # command that will also dequeue 2 dummy items (dequeueOne skips
        # nullptr desc safely).
        # We need to drain all 4 dummies.  send_command drains 2; do it twice.
        sim.send_command("PING")
        sim.send_command("PING")
