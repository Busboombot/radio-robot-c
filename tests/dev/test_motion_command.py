#!/usr/bin/env python3
"""test_motion_command.py — Unit tests for MotionCommand state machine (017-003).

Pure Python mirror of the MotionCommand lifecycle from:
  source/control/MotionCommand.h / .cpp

Tests verify:
  - SOFT teardown: stop fires → target (0,0) → EVT done when BVC converges.
  - SOFT absolute deadline: EVT emitted after 3000 ms even if BVC never converges.
  - HARD cancel: EVT cancelled on same tick; active() false immediately.
  - active() false after full termination.
  - Recycled command (configure + start twice): baseline resets; no residue.
  - armTime: TIME condition not re-fired within new sTimeoutMs window.
  - Zero-condition command: never self-terminates.
  - OR-across-array stop evaluation.

Implementation note: all BVC interactions are mocked with a minimal Python
class tracking calls made to it. The tests do NOT run C++ code.
"""

from __future__ import annotations

import math
import pytest
from typing import Optional


# ---------------------------------------------------------------------------
# Python mirrors of StopCondition and MotionBaseline
# ---------------------------------------------------------------------------

class MotionBaseline:
    def __init__(self):
        self.t0Ms       = 0
        self.enc0Mm     = 0.0
        self.heading0Rad = 0.0
        self.pose0X     = 0.0
        self.pose0Y     = 0.0


class HardwareState:
    def __init__(self, **kwargs):
        self.encLMm   = kwargs.get('encLMm',   0.0)
        self.encRMm   = kwargs.get('encRMm',   0.0)
        self.poseX    = kwargs.get('poseX',    0.0)
        self.poseY    = kwargs.get('poseY',    0.0)
        self.poseHrad = kwargs.get('poseHrad', 0.0)
        self.line     = kwargs.get('line',     [0, 0, 0, 0])
        self.colorR   = kwargs.get('colorR',   0)
        self.colorG   = kwargs.get('colorG',   0)
        self.colorB   = kwargs.get('colorB',   0)
        self.colorC   = kwargs.get('colorC',   0)
        self.analogIn = kwargs.get('analogIn', [0, 0, 0, 0])


# ---------------------------------------------------------------------------
# Mock BVC
# ---------------------------------------------------------------------------

class MockBVC:
    """Minimal mock of BodyVelocityController."""

    def __init__(self, at_target_val: bool = False):
        self._at_target_val = at_target_val
        self.last_v         = 0.0
        self.last_omega     = 0.0
        self.set_target_calls: list[tuple[float, float]] = []
        self.advance_calls: list[float] = []
        self.reset_calls: int = 0

    def set_target(self, v: float, omega: float) -> None:
        self.last_v     = v
        self.last_omega = omega
        self.set_target_calls.append((v, omega))

    def advance(self, dt_s: float) -> bool:
        self.advance_calls.append(dt_s)
        return not self._at_target_val

    def at_target(self) -> bool:
        return self._at_target_val

    def reset(self) -> None:
        self.reset_calls += 1
        self.last_v     = 0.0
        self.last_omega = 0.0

    # Convenience: make the mock converge after N advance() calls.
    def converge_after(self, n: int) -> None:
        """Set up: at_target returns False for first n-1 calls, True after."""
        # Replace at_target with a counter-based version.
        count = [0]
        target_n = n

        def at_target_counted() -> bool:
            return count[0] >= target_n

        def advance_counted(dt_s: float) -> bool:
            count[0] += 1
            self.advance_calls.append(dt_s)
            return not at_target_counted()

        self._at_target_fn = at_target_counted
        self._advance_fn   = advance_counted

    def _at_target_default(self) -> bool:
        return self._at_target_val


# ---------------------------------------------------------------------------
# Python mirror of MotionCommand
# ---------------------------------------------------------------------------

# Re-use evaluate() from test_stop_condition
import math

def wrap_angle(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))


def _evaluate_condition(cond: dict, s: HardwareState, now_ms: int,
                        base: MotionBaseline) -> bool:
    kind = cond.get('kind', 'NONE')
    a    = cond.get('a', 0.0)
    b    = cond.get('b', 0.0)
    ax   = cond.get('ax', 0.0)

    if kind == 'NONE':
        return False
    elif kind == 'TIME':
        elapsed = now_ms - base.t0Ms
        return elapsed >= int(a)
    elif kind == 'DISTANCE':
        enc_avg = (s.encLMm + s.encRMm) * 0.5
        return abs(enc_avg - base.enc0Mm) >= a
    elif kind == 'HEADING':
        delta = wrap_angle(s.poseHrad - base.heading0Rad)
        return abs(wrap_angle(delta - a)) < b
    elif kind == 'POSITION':
        dx = s.poseX - ax
        dy = s.poseY - a
        return (dx * dx + dy * dy) < (b * b)
    elif kind == 'SENSOR':
        return False  # not exercised in these tests
    return False


class MotionCommand:
    """
    Python mirror of source/control/MotionCommand.cpp.

    Simulates MotorController interaction via a MockBVC.
    EVT messages are collected in self.emitted_evts for inspection.
    """

    K_MAX_STOP_CONDS   = 4
    K_SOFT_DEADLINE_MS = 3000

    def __init__(self):
        self._bvc:      Optional[MockBVC] = None
        self._vTgt      = 0.0
        self._omegaTgt  = 0.0
        self._stops:    list[dict] = []
        self._baseline  = MotionBaseline()
        self._reply_fn  = None
        self._corr_id   = ''
        self._stop_style = 'SOFT'
        self._active    = False
        self._stopping  = False
        self._soft_deadline_ms = 0
        self._now_ms    = 0
        # Mirrors C++ MotionCommand::setDoneEvt(); default "EVT done".
        self._done_evt_label = 'EVT done'

        # Captured EVT messages.
        self.emitted_evts: list[str] = []

    # ----------------------------------------------------------------
    # Configuration
    # ----------------------------------------------------------------

    def configure(self, v_mms: float, omega_rads: float, bvc: MockBVC) -> None:
        self._bvc       = bvc
        self._vTgt      = v_mms
        self._omegaTgt  = omega_rads
        self._stops     = []
        self._baseline  = MotionBaseline()
        self._reply_fn  = None
        self._corr_id   = ''
        self._stop_style = 'SOFT'
        self._active    = False
        self._stopping  = False
        self._soft_deadline_ms = 0
        self._now_ms    = 0
        self._done_evt_label = 'EVT done'
        self.emitted_evts = []

    def add_stop(self, cond: dict) -> bool:
        if len(self._stops) >= self.K_MAX_STOP_CONDS:
            return False
        self._stops.append(cond)
        return True

    def set_reply_sink(self, fn, ctx, corr_id: str) -> None:
        self._reply_fn = fn
        self._corr_id  = corr_id or ''

    def set_stop_style(self, style: str) -> None:
        self._stop_style = style

    def set_done_evt(self, label: str) -> None:
        """Mirror of MotionCommand::setDoneEvt. Override the EVT label on completion."""
        self._done_evt_label = label

    def arm_time(self, now_ms: int) -> None:
        """Re-arm t0Ms in the first TIME condition baseline."""
        for c in self._stops:
            if c.get('kind') == 'TIME':
                self._baseline.t0Ms = now_ms
                return

    # ----------------------------------------------------------------
    # Execution
    # ----------------------------------------------------------------

    def start(self, inputs: HardwareState, now_ms: int) -> None:
        self._baseline.t0Ms        = now_ms
        self._baseline.enc0Mm      = (inputs.encLMm + inputs.encRMm) * 0.5
        self._baseline.heading0Rad = inputs.poseHrad
        self._baseline.pose0X      = inputs.poseX
        self._baseline.pose0Y      = inputs.poseY
        self._now_ms               = now_ms
        self._active               = True
        self._stopping             = False
        if self._bvc:
            self._bvc.set_target(self._vTgt, self._omegaTgt)

    def set_target(self, v_mms: float, omega_rads: float) -> None:
        self._vTgt    = v_mms
        self._omegaTgt = omega_rads
        if self._bvc:
            self._bvc.set_target(v_mms, omega_rads)
        self.arm_time(self._now_ms)

    def tick(self, inputs: HardwareState, now_ms: int, dt_s: float) -> bool:
        if not self._active:
            return False

        self._now_ms = now_ms

        # -- SOFT ramp-down sub-phase.
        if self._stopping:
            if self._bvc:
                self._bvc.advance(dt_s)
            converged     = self._bvc.at_target() if self._bvc else True
            deadline_hit  = (now_ms - self._soft_deadline_ms) >= 0
            if converged or deadline_hit:
                self._active   = False
                self._stopping = False
                self._emit_evt(self._done_evt_label)
            return self._active

        # -- Normal running sub-phase.
        if self._bvc:
            self._bvc.advance(dt_s)

        # Evaluate conditions (OR-combined).
        stopped = False
        for c in self._stops:
            if _evaluate_condition(c, inputs, now_ms, self._baseline):
                stopped = True
                break

        if stopped:
            if self._stop_style == 'HARD':
                if self._bvc: self._bvc.reset()
                self._active   = False
                self._stopping = False
                self._emit_evt(self._done_evt_label)
            else:  # SOFT
                self._stopping        = True
                self._soft_deadline_ms = now_ms + self.K_SOFT_DEADLINE_MS
                if self._bvc:
                    self._bvc.set_target(0.0, 0.0)

        return self._active

    def cancel(self, style: str = 'HARD') -> None:
        if not self._active:
            return
        if self._bvc: self._bvc.reset()
        self._active   = False
        self._stopping = False
        self._emit_evt('EVT cancelled')

    def active(self) -> bool:
        return self._active

    # ----------------------------------------------------------------
    # Private
    # ----------------------------------------------------------------

    def _emit_evt(self, base: str) -> None:
        if self._corr_id:
            msg = f"{base} #{self._corr_id}"
        else:
            msg = base
        self.emitted_evts.append(msg)
        if self._reply_fn:
            self._reply_fn(msg, None)


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------

def make_mc() -> MotionCommand:
    return MotionCommand()


def make_bvc(at_target: bool = False) -> MockBVC:
    return MockBVC(at_target_val=at_target)


# ---------------------------------------------------------------------------
# Tests — SOFT teardown
# ---------------------------------------------------------------------------

class TestSoftTeardown:
    """SOFT stop: target (0,0); active stays true during ramp; EVT done on convergence."""

    def test_soft_stop_sets_zero_target_on_fire(self):
        """When stop fires with SOFT style, BVC is commanded to (0, 0)."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})  # 100 ms
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        # Advance to just before stop, then trigger.
        mc.tick(HardwareState(), now_ms=99,  dt_s=0.01)   # not yet
        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)   # TIME fires → SOFT start

        # Last set_target should have been (0, 0).
        assert bvc.last_v     == pytest.approx(0.0)
        assert bvc.last_omega == pytest.approx(0.0)

    def test_soft_active_stays_true_while_ramping(self):
        """active() is True during the SOFT ramp-down sub-phase."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)  # never converges

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)   # stop fires
        assert mc.active() is True, "active() should be True during SOFT ramp"

    def test_soft_evt_done_emitted_when_bvc_converges(self):
        """EVT done is emitted when BVC mock reports at_target() = True."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        # Fire the stop.
        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)
        assert mc.active() is True
        assert len(mc.emitted_evts) == 0

        # BVC now reports at target.
        bvc._at_target_val = True
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)   # should terminate

        assert mc.active() is False
        assert len(mc.emitted_evts) == 1
        assert mc.emitted_evts[0] == 'EVT done'

    def test_soft_active_false_after_evt_done(self):
        """active() is False once EVT done is emitted.

        SOFT teardown requires a two-tick sequence:
          Tick 1 (t=100): TIME fires → _stopping=True, target (0,0).
          Tick 2 (t=110): BVC reports atTarget() → emit EVT done → IDLE.
        """
        mc  = make_mc()
        bvc = make_bvc(at_target=True)  # converges on first check

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)   # TIME fires → _stopping
        assert mc.active() is True, "Still in SOFT ramp after first tick"

        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)   # BVC at target → done
        assert mc.active() is False


# ---------------------------------------------------------------------------
# Tests — SOFT absolute deadline
# ---------------------------------------------------------------------------

class TestSoftDeadline:
    """SOFT absolute deadline: EVT done emitted after 3000 ms even if BVC never converges."""

    def test_soft_deadline_fires_after_3000ms(self):
        """If BVC never reaches zero, EVT done emits after kSoftDeadlineMs=3000."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        # Fire the stop at 100 ms.
        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)
        assert mc.active() is True

        # Tick at 3099 ms (just before deadline): still active.
        mc.tick(HardwareState(), now_ms=3099, dt_s=0.01)
        assert mc.active() is True, "Should still be active before deadline"

        # Tick at 3100 ms (deadline = 100 + 3000 = 3100 ms): fires.
        mc.tick(HardwareState(), now_ms=3100, dt_s=0.01)
        assert mc.active() is False
        assert any('EVT done' in e for e in mc.emitted_evts)

    def test_soft_deadline_earlier_than_3000ms_if_bvc_converges(self):
        """If BVC converges early, EVT done fires before 3000 ms deadline."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)  # stop fires
        bvc._at_target_val = True
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)  # converged → done

        assert mc.active() is False
        assert mc.emitted_evts[-1] == 'EVT done'


# ---------------------------------------------------------------------------
# Tests — HARD cancel
# ---------------------------------------------------------------------------

class TestHardCancel:
    """HARD cancel: EVT cancelled on same tick; active() false immediately."""

    def test_hard_cancel_emits_evt_cancelled(self):
        """cancel(HARD) emits EVT cancelled."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        mc.cancel('HARD')

        assert any('EVT cancelled' in e for e in mc.emitted_evts)

    def test_hard_cancel_active_false_immediately(self):
        """active() is False immediately after cancel()."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()  # default HARD

        assert mc.active() is False

    def test_hard_cancel_calls_bvc_reset(self):
        """cancel() calls bvc.reset() to zero the profiler."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        assert bvc.reset_calls == 1

    def test_hard_cancel_noop_when_not_active(self):
        """cancel() is a no-op when the command is not running."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        # Do NOT call start — command is idle.
        mc.cancel()

        assert mc.active() is False
        assert len(mc.emitted_evts) == 0  # no EVT emitted


# ---------------------------------------------------------------------------
# Tests — active() semantics
# ---------------------------------------------------------------------------

class TestActiveSemantics:
    """active() is false before start, true while running, false after termination."""

    def test_active_false_before_start(self):
        mc  = make_mc()
        bvc = make_bvc()
        mc.configure(200.0, 0.0, bvc)
        assert mc.active() is False

    def test_active_true_after_start(self):
        mc  = make_mc()
        bvc = make_bvc()
        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        assert mc.active() is True

    def test_active_false_after_hard_stop(self):
        """HARD stop via a stop condition: active() false on same tick."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('HARD')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)
        assert mc.active() is False

    def test_tick_returns_false_when_idle(self):
        """tick() returns False when called on an idle (not started) command."""
        mc  = make_mc()
        bvc = make_bvc()
        mc.configure(200.0, 0.0, bvc)

        result = mc.tick(HardwareState(), now_ms=0, dt_s=0.01)
        assert result is False


# ---------------------------------------------------------------------------
# Tests — Recycled command (no residue)
# ---------------------------------------------------------------------------

class TestRecycledCommand:
    """configure + start called twice: baseline resets; no residue from prior run."""

    def test_recycled_baseline_resets(self):
        """A second configure+start uses fresh baseline from the new start inputs."""
        mc  = make_mc()
        bvc = make_bvc()

        # First run: start at enc=0.
        mc.configure(100.0, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': 200.0})
        mc.set_stop_style('HARD')  # immediate stop so active() goes False on fire tick
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)
        mc.cancel()  # abort first run

        # Second run: start at enc=500mm.
        mc.configure(100.0, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': 200.0})
        mc.set_stop_style('HARD')
        mc.start(HardwareState(encLMm=500.0, encRMm=500.0), now_ms=0)

        # enc0 should be 500, not 0.  Only 50mm traveled → not yet fired.
        mc.tick(HardwareState(encLMm=550.0, encRMm=550.0), now_ms=10, dt_s=0.01)
        assert mc.active() is True, "Should not fire (only 50mm since new baseline)"

        # 200mm from new enc0=500 → enc_avg=700 → fires (HARD → immediate).
        mc.tick(HardwareState(encLMm=700.0, encRMm=700.0), now_ms=20, dt_s=0.01)
        assert mc.active() is False

    def test_recycled_stop_conditions_cleared(self):
        """configure() clears stop conditions from the previous run."""
        mc  = make_mc()
        bvc = make_bvc()

        # First run with a TIME stop.
        mc.configure(100.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        # Second run: no stop conditions (fresh configure).
        mc.configure(100.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)

        # Tick far past the old TIME threshold — should NOT fire.
        mc.tick(HardwareState(), now_ms=5000, dt_s=0.01)
        assert mc.active() is True

    def test_recycled_evt_log_cleared(self):
        """configure() clears the emitted_evts list from the previous run."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        prev_evts = list(mc.emitted_evts)  # copy before configure

        mc.configure(100.0, 0.0, bvc)
        assert mc.emitted_evts == [], (
            f"configure() should clear emitted_evts, but got {mc.emitted_evts!r}"
        )
        # Previous run's events still in the copy.
        assert prev_evts == ['EVT cancelled']

    def test_recycled_corr_id_used(self):
        """Second run uses its own corrId in the EVT, not the first run's."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        mc.set_reply_sink(None, None, 'first-id')
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        # Second run.
        mc.configure(100.0, 0.0, bvc)
        mc.set_reply_sink(None, None, 'second-id')
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        # Last EVT should reference second-id.
        assert 'second-id' in mc.emitted_evts[-1]
        assert 'first-id'  not in mc.emitted_evts[-1]


# ---------------------------------------------------------------------------
# Tests — armTime (VW keepalive)
# ---------------------------------------------------------------------------

class TestArmTime:
    """armTime: TIME condition not re-fired within new sTimeoutMs window."""

    def test_arm_time_resets_t0(self):
        """After armTime(500), TIME(a=200) requires 200 ms from t=500, not t=0."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 200.0})   # 200 ms timeout
        mc.set_stop_style('HARD')  # HARD so active() goes False on the same tick
        mc.start(HardwareState(), now_ms=0)

        # At t=150: 150ms elapsed. Would fire at t=200.
        mc.tick(HardwareState(), now_ms=150, dt_s=0.01)
        assert mc.active() is True

        # armTime at t=150: reset t0 to 150.
        mc.arm_time(150)

        # At t=200: only 50ms since re-arm → should NOT fire.
        mc.tick(HardwareState(), now_ms=200, dt_s=0.01)
        assert mc.active() is True, "TIME should not fire 50ms after re-arm"

        # At t=350: 200ms since re-arm (150+200=350) → fires (HARD → immediate).
        mc.tick(HardwareState(), now_ms=350, dt_s=0.01)
        assert mc.active() is False

    def test_set_target_calls_arm_time(self):
        """setTarget() internally calls armTime to reset the TIME baseline."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 300.0})   # 300 ms timeout
        mc.set_stop_style('HARD')  # HARD so active() goes False on fire tick
        mc.start(HardwareState(), now_ms=0)

        # At t=250: 250ms elapsed, would fire at 300.
        mc.tick(HardwareState(), now_ms=250, dt_s=0.01)
        assert mc.active() is True

        # set_target acts as a keepalive — re-arms TIME.
        # The tick() method updates _now_ms at the top of each call,
        # so we emulate that here before calling set_target directly.
        mc._now_ms = 250  # emulate the now_ms update inside tick
        mc.set_target(100.0, 0.0)

        # At t=400: 150ms since re-arm (250+300=550, not 400) → NOT fired.
        mc.tick(HardwareState(), now_ms=400, dt_s=0.01)
        assert mc.active() is True, "Should not fire 150ms after re-arm"

        # At t=550: exactly 300ms since re-arm → fires (HARD → immediate).
        mc.tick(HardwareState(), now_ms=550, dt_s=0.01)
        assert mc.active() is False


# ---------------------------------------------------------------------------
# Tests — zero-condition command
# ---------------------------------------------------------------------------

class TestZeroConditions:
    """Zero-condition command: no self-termination."""

    def test_no_conditions_never_terminates(self):
        """A command with no stop conditions runs indefinitely (must be cancelled)."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        # No addStop calls.
        mc.start(HardwareState(), now_ms=0)

        # Tick many times — should never self-terminate.
        for t in range(0, 100000, 1000):
            result = mc.tick(HardwareState(encLMm=float(t), encRMm=float(t)),
                             now_ms=t, dt_s=0.01)
            assert result is True, f"Command terminated unexpectedly at t={t}"

    def test_no_conditions_terminates_on_cancel(self):
        """A zero-condition command still responds to cancel()."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        mc.tick(HardwareState(), now_ms=1000, dt_s=0.01)

        mc.cancel()
        assert mc.active() is False


# ---------------------------------------------------------------------------
# Tests — addStop overflow
# ---------------------------------------------------------------------------

class TestAddStopOverflow:
    """addStop returns false (and does not crash) when full."""

    def test_addstop_returns_false_when_full(self):
        """Adding more than kMaxStopConds conditions returns False."""
        mc  = make_mc()
        bvc = make_bvc()
        mc.configure(100.0, 0.0, bvc)

        results = []
        for i in range(MotionCommand.K_MAX_STOP_CONDS + 2):
            r = mc.add_stop({'kind': 'NONE'})
            results.append(r)

        # First K_MAX_STOP_CONDS should succeed.
        assert all(results[:MotionCommand.K_MAX_STOP_CONDS])
        # Overflow calls should return False.
        assert not results[MotionCommand.K_MAX_STOP_CONDS]
        assert not results[MotionCommand.K_MAX_STOP_CONDS + 1]

    def test_addstop_overflow_does_not_corrupt(self):
        """Overflow addStop calls do not corrupt existing conditions."""
        mc  = make_mc()
        bvc = make_bvc()
        mc.configure(100.0, 0.0, bvc)

        # Add max conditions, all NONE.
        for _ in range(MotionCommand.K_MAX_STOP_CONDS):
            mc.add_stop({'kind': 'NONE'})

        # Try to add one more (should be rejected).
        mc.add_stop({'kind': 'TIME', 'a': 100.0})

        mc.start(HardwareState(), now_ms=0)

        # Tick at 100ms — TIME should NOT have been added (overflow rejected).
        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)
        assert mc.active() is True, "Overflow condition should not have been added"


# ---------------------------------------------------------------------------
# Tests — corrId in EVT messages
# ---------------------------------------------------------------------------

class TestCorrId:
    """EVT messages include corrId when set."""

    def test_evt_done_includes_corr_id(self):
        """EVT done includes corrId.

        SOFT teardown: two ticks needed (fire → _stopping; then at_target → done).
        """
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(100.0, 0.0, bvc)
        mc.set_reply_sink(None, None, 'abc123')
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)   # TIME fires → _stopping
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)   # BVC at target → EVT done
        assert mc.emitted_evts[-1] == 'EVT done #abc123'

    def test_evt_cancelled_includes_corr_id(self):
        """EVT cancelled includes corrId."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(100.0, 0.0, bvc)
        mc.set_reply_sink(None, None, 'xyz789')
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        assert mc.emitted_evts[-1] == 'EVT cancelled #xyz789'

    def test_evt_no_corr_id(self):
        """EVT without corrId omits the # suffix."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(100.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)   # TIME fires → _stopping
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)   # BVC at target → EVT done
        assert mc.emitted_evts[-1] == 'EVT done'


# ---------------------------------------------------------------------------
# Tests — BVC advance called exactly once per tick
# ---------------------------------------------------------------------------

class TestBvcAdvance:
    """BVC advance is called exactly once per tick, not double-advanced."""

    def test_advance_once_per_tick_running(self):
        """During normal running, advance() is called once per tick."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)

        for i in range(5):
            mc.tick(HardwareState(), now_ms=(i + 1) * 10, dt_s=0.01)

        assert len(bvc.advance_calls) == 5

    def test_advance_once_per_tick_soft_ramp(self):
        """During SOFT ramp-down, advance() is called once per tick."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        # Fire the stop.
        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)

        # 3 more ticks during SOFT ramp.
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)
        mc.tick(HardwareState(), now_ms=120, dt_s=0.01)
        mc.tick(HardwareState(), now_ms=130, dt_s=0.01)

        # Total: 4 ticks → 4 advance calls.
        assert len(bvc.advance_calls) == 4


# ---------------------------------------------------------------------------
# Tests — D-command migration (018-004)
# ---------------------------------------------------------------------------
#
# These tests validate the key behavioral properties of the D-command
# migration onto MotionCommand:
#   1. DISTANCE stop fires at the target; EVT done D is emitted.
#   2. Terminal decel cap clamps commanded speed downward as d_remaining shrinks.
#   3. Safety TIME net (2× nominal + 2 s) does not trip on a normal ramped drive.
#   4. DISTANCE stop fires before safety TIME net in a normal drive.
#
# Uses Python mirrors from this module; does not exercise C++ code directly.
# ---------------------------------------------------------------------------

import math as _math


def _decel_cap(a_decel: float, d_remaining: float) -> float:
    """Python mirror of the D decel hook: v_cap = sqrt(2 * aDecel * d_remaining)."""
    if d_remaining <= 0.0:
        return 0.0
    return _math.sqrt(2.0 * a_decel * d_remaining)


class TestDCommandDistanceStop:
    """DISTANCE stop terminates the command at the target and emits EVT done D."""

    def test_distance_stop_fires_at_target(self):
        """DISTANCE(300) fires when enc_avg - enc0 >= 300."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': 300.0})
        mc.set_stop_style('SOFT')
        # Inform the mock about done evt name (for inspection).
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)

        # 299 mm traveled — not yet.
        mc.tick(HardwareState(encLMm=299.0, encRMm=299.0), now_ms=1000, dt_s=0.01)
        assert mc.active() is True, "DISTANCE should not fire at 299 mm"

        # 300 mm traveled — DISTANCE fires → SOFT ramp starts (still active).
        mc.tick(HardwareState(encLMm=300.0, encRMm=300.0), now_ms=1010, dt_s=0.01)
        assert mc.active() is True, "Still active during SOFT ramp-down"

        # BVC now at target → EVT done emitted.
        bvc._at_target_val = True
        mc.tick(HardwareState(encLMm=300.0, encRMm=300.0), now_ms=1020, dt_s=0.01)
        assert mc.active() is False
        assert len(mc.emitted_evts) == 1

    def test_distance_stop_evt_done_label(self):
        """When setDoneEvt('EVT done D') is used, the emitted label is 'EVT done D'."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': 200.0})
        mc.set_stop_style('SOFT')
        mc.set_done_evt('EVT done D')
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)

        # Fire the DISTANCE condition.
        mc.tick(HardwareState(encLMm=200.0, encRMm=200.0), now_ms=1000, dt_s=0.01)
        # SOFT ramp-down tick — BVC at target so EVT fires immediately.
        mc.tick(HardwareState(encLMm=200.0, encRMm=200.0), now_ms=1010, dt_s=0.01)

        # EVT uses the overridden label.
        assert any('EVT done D' in e for e in mc.emitted_evts)

    def test_distance_stop_does_not_fire_before_target(self):
        """DISTANCE stop does not fire until enc_avg delta reaches threshold."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': 500.0})
        mc.set_stop_style('HARD')
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)

        # Accumulate 499 mm — must still be active.
        for step in range(1, 10):
            enc = float(step * 49)  # max 441 mm
            mc.tick(HardwareState(encLMm=enc, encRMm=enc), now_ms=step * 100, dt_s=0.1)
        assert mc.active() is True, "Should still be running at < 500 mm"


class TestDCommandDecelCap:
    """Terminal decel cap clamps commanded speed downward as d_remaining shrinks."""

    def test_decel_cap_formula(self):
        """v_cap = sqrt(2 * aDecel * d_remaining) — formula verification."""
        # aDecel = 600 mm/s^2 (typical config value)
        a_decel = 600.0

        # At 100 mm remaining: v_cap = sqrt(2 * 600 * 100) = sqrt(120000) ≈ 346.4
        cap = _decel_cap(a_decel, 100.0)
        assert abs(cap - _math.sqrt(120000.0)) < 0.01

        # At 10 mm remaining: v_cap = sqrt(2 * 600 * 10) = sqrt(12000) ≈ 109.5
        cap = _decel_cap(a_decel, 10.0)
        assert abs(cap - _math.sqrt(12000.0)) < 0.01

    def test_decel_cap_clamps_downward_only(self):
        """decel cap only clamps; it does NOT increase speed beyond commanded v."""
        a_decel = 600.0
        v_commanded = 200.0

        # Far from target (d_remaining = 5000 mm): cap = sqrt(6000000) >> 200
        cap_far = _decel_cap(a_decel, 5000.0)
        v_applied = min(v_commanded, cap_far)
        assert v_applied == pytest.approx(v_commanded), (
            "Far from target, cap should not reduce speed"
        )

        # Near target (d_remaining = 10 mm): cap ≈ 109.5 < 200 → clamp
        cap_near = _decel_cap(a_decel, 10.0)
        v_applied = min(v_commanded, cap_near)
        assert v_applied < v_commanded, "Near target, cap must reduce speed"
        assert v_applied == pytest.approx(cap_near)

    def test_decel_cap_zero_at_zero_remaining(self):
        """decel cap returns 0 when d_remaining <= 0."""
        assert _decel_cap(600.0, 0.0) == 0.0
        assert _decel_cap(600.0, -5.0) == 0.0

    def test_setTarget_clamps_speed_near_end(self):
        """MotionCommand.setTarget called with v_cap when d_remaining is small."""
        mc  = make_mc()
        bvc = make_bvc(at_target=False)

        v_cmd   = 200.0
        a_decel = 600.0
        target_mm = 300.0

        mc.configure(v_cmd, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': target_mm})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)

        # Simulate the decel hook calling setTarget with a capped speed
        # when 10 mm remain.  In production this is done by DriveController::driveAdvance.
        d_remaining = 10.0  # mm
        v_cap = _decel_cap(a_decel, d_remaining)
        assert v_cap < v_cmd, "Sanity: v_cap < v_cmd at 10 mm remaining"

        # Mimic DriveController hook: only call setTarget when v_cap < current target v.
        if v_cap < bvc.last_v if bvc.last_v > 0 else v_cmd:
            mc.set_target(v_cap, 0.0)

        # After setTarget, BVC's last commanded speed should be the capped value.
        mc.set_target(v_cap, 0.0)
        assert bvc.last_v == pytest.approx(v_cap)
        assert bvc.last_v < v_cmd


class TestDCommandSafetyTimeNet:
    """Safety TIME net: generous enough to not trip on a normal ramped drive."""

    @staticmethod
    def _compute_timeout_ms(target_mm: float, speed_mms: float) -> float:
        """
        Mirror of the D-command timeout formula in beginDistance:
          nominalMs = (targetMm / max(|vL|, |vR|)) * 1000
          timeoutMs = nominalMs * 2.0 + 2000
        """
        spd_max = max(abs(speed_mms), 1.0)
        nominal_ms = (target_mm / spd_max) * 1000.0
        return nominal_ms * 2.0 + 2000.0

    def test_timeout_formula_200_200_400(self):
        """D 200 200 400: nominal = 2000 ms; timeout = 6000 ms."""
        # From the ticket: 400/200 * 1000 = 2000 ms nominal; 2*2000+2000 = 6000
        timeout = self._compute_timeout_ms(400.0, 200.0)
        assert timeout == pytest.approx(6000.0)

    def test_timeout_generous_with_ramp_up(self):
        """Timeout must be well above actual travel time including ramp-up overhead.

        For D 200 200 400: at 200 mm/s full speed it takes 2000 ms.
        With a ~200 ms ramp-up at aMax=600 mm/s^2 the robot covers ~12 mm during ramp.
        Actual travel time is slightly longer than nominal.  The 2× factor (6000 ms)
        must comfortably exceed the actual time.
        """
        target_mm   = 400.0
        speed_mms   = 200.0
        a_max       = 600.0   # mm/s^2 typical

        # Ramp-up time at constant acceleration to reach v_cmd from 0.
        ramp_up_time_s = speed_mms / a_max   # 200/600 ≈ 0.333 s
        dist_ramp_up   = 0.5 * a_max * ramp_up_time_s ** 2  # ≈ 33 mm

        # Remaining distance at full speed.
        dist_cruise    = target_mm - dist_ramp_up  # ≈ 367 mm
        cruise_time_s  = dist_cruise / speed_mms   # ≈ 1.83 s

        # Total estimated travel time in ms.
        total_time_ms  = (ramp_up_time_s + cruise_time_s) * 1000.0  # ≈ 2167 ms

        timeout_ms = self._compute_timeout_ms(target_mm, speed_mms)

        # Timeout must be substantially above the actual travel time.
        assert timeout_ms > total_time_ms * 1.5, (
            f"Timeout {timeout_ms:.0f} ms too close to estimated travel time "
            f"{total_time_ms:.0f} ms — would risk early trip during ramp-up"
        )

    def test_timeout_scales_with_distance(self):
        """Longer distances get proportionally longer timeouts."""
        t_short = self._compute_timeout_ms(200.0, 200.0)   # 1000 + 2000 = 3000
        t_long  = self._compute_timeout_ms(1000.0, 200.0)  # 5000 + 2000 = 7000
        assert t_long > t_short

    def test_timeout_scales_with_speed(self):
        """Faster drives get shorter timeouts (closer to actual travel time)."""
        t_slow = self._compute_timeout_ms(400.0, 100.0)   # 4000 + 2000 = 6000 → 10000
        t_fast = self._compute_timeout_ms(400.0, 400.0)   # 1000 + 2000 = 3000 → 4000
        assert t_fast < t_slow

    def test_time_stop_does_not_fire_before_distance_stop(self):
        """In a normal drive, DISTANCE stop fires well before the safety TIME stop.

        Simulates D 200 200 400: target 400 mm, 200 mm/s.
        Uses HARD stop style so active() goes False on the fire tick.
        """
        mc  = make_mc()
        bvc = make_bvc()

        target_mm  = 400.0
        speed_mms  = 200.0
        timeout_ms = self._compute_timeout_ms(target_mm, speed_mms)  # 6000 ms

        mc.configure(speed_mms, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': target_mm})
        mc.add_stop({'kind': 'TIME',     'a': timeout_ms})
        mc.set_stop_style('HARD')
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)

        # Simulate arriving at target_mm at t=2200 ms (slightly above nominal 2000 ms
        # to model ramp-up overhead — DISTANCE fires, TIME does NOT).
        mc.tick(HardwareState(encLMm=target_mm, encRMm=target_mm),
                now_ms=2200, dt_s=0.01)

        # Command should have terminated via DISTANCE (2200 ms << 6000 ms timeout).
        assert mc.active() is False, "Should have stopped at distance target"
        # Confirm it fired well before the timeout — time budget check.
        assert 2200 < timeout_ms, (
            f"Simulated arrival time {2200} ms should be < timeout {timeout_ms} ms"
        )
