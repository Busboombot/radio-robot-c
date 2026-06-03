#!/usr/bin/env python3
"""test_pursuit_arc_steering.py — Unit tests for pursuit-arc steering law (011-002).

Pure Python implementation of the receding-horizon curvature steering law from:
  docs/kinematics-model.md §1.5
  source/control/DriveController.cpp (PURSUE branch)

Tests verify:
  - World-to-robot-frame goal transform
  - Curvature law: κ = 2·dy/(dx²+dy²)
  - Zero-distance guard: d² ≤ 0.1 → κ = 0 (no divide-by-zero)
  - Straight-ahead: κ = 0, ω = 0, vL = vR = _gSpeed
  - 45° left offset: κ = 0.01, ω = _gSpeed · 0.01
  - 90° left: κ = 0.02
  - Goal at origin: d² guard fires, κ = 0
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirrors of the C++ pursuit-arc steering law
# ---------------------------------------------------------------------------

def world_to_robot_goal(
    gx_world: float,
    gy_world: float,
    robot_x: float,
    robot_y: float,
    robot_h_rad: float,
) -> tuple[float, float]:
    """Transform a world-frame goal into robot frame.

    C++ equivalent in DriveController::tick() PURSUE branch:
        dxW = _gTargetXWorld - x
        dyW = _gTargetYWorld - y
        dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad)
        dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad)
    """
    dxW = gx_world - robot_x
    dyW = gy_world - robot_y
    dx =  dxW * math.cos(robot_h_rad) + dyW * math.sin(robot_h_rad)
    dy = -dxW * math.sin(robot_h_rad) + dyW * math.cos(robot_h_rad)
    return dx, dy


def compute_kappa(dx: float, dy: float) -> float:
    """Pursuit-arc curvature: κ = 2·dy/(dx²+dy²), with d²≤0.1 guard → 0.

    C++ equivalent in DriveController::tick() PURSUE branch:
        float d2    = dx * dx + dy * dy;
        float kappa = (d2 > 0.1f) ? (2.0f * dy / d2) : 0.0f;
    """
    d2 = dx * dx + dy * dy
    if d2 > 0.1:
        return 2.0 * dy / d2
    return 0.0


def beginGoTo_world_goal(
    tx: float,
    ty: float,
    robot_x: float,
    robot_y: float,
    robot_h_rad: float,
) -> tuple[float, float]:
    """Transform robot-relative (tx, ty) goal to world frame at beginGoTo() time.

    C++ equivalent in DriveController::beginGoTo():
        _gTargetXWorld = x + tx * cosf(h_rad) - ty * sinf(h_rad)
        _gTargetYWorld = y + tx * sinf(h_rad) + ty * cosf(h_rad)
    """
    gx = robot_x + tx * math.cos(robot_h_rad) - ty * math.sin(robot_h_rad)
    gy = robot_y + tx * math.sin(robot_h_rad) + ty * math.cos(robot_h_rad)
    return gx, gy


def bk_inverse(v: float, omega: float, b: float) -> tuple[float, float]:
    """vL = v - omega*(b/2), vR = v + omega*(b/2)."""
    half_b = b / 2.0
    vL = v - omega * half_b
    vR = v + omega * half_b
    return vL, vR


# ---------------------------------------------------------------------------
# Tests — curvature formula κ = 2·dy/(dx²+dy²)
# ---------------------------------------------------------------------------

class TestCurvatureLaw:
    """Verify κ = 2·dy/(dx²+dy²) covers all acceptance-criteria cases."""

    def test_straight_ahead_kappa_zero(self):
        """AC: goal (dx=300, dy=0) → κ = 0."""
        kappa = compute_kappa(dx=300.0, dy=0.0)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_45_deg_left(self):
        """AC: goal (dx=100, dy=100) → κ = 2·100/(100²+100²) = 0.01."""
        # d² = 10000 + 10000 = 20000; κ = 200/20000 = 0.01
        kappa = compute_kappa(dx=100.0, dy=100.0)
        assert kappa == pytest.approx(0.01, rel=1e-6)

    def test_90_deg_left(self):
        """AC: goal (dx=0, dy=100) → κ = 2·100/(0+10000) = 0.02."""
        kappa = compute_kappa(dx=0.0, dy=100.0)
        assert kappa == pytest.approx(0.02, rel=1e-6)

    def test_zero_distance_guard(self):
        """AC: goal (dx=0, dy=0) → d²=0 ≤ 0.1 guard fires, κ = 0."""
        kappa = compute_kappa(dx=0.0, dy=0.0)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_guard_threshold_boundary(self):
        """d² exactly at boundary (0.1): guard fires, κ = 0."""
        # dx=sqrt(0.05), dy=sqrt(0.05): d²=0.1 (not > 0.1)
        dx = math.sqrt(0.05)
        dy = math.sqrt(0.05)
        kappa = compute_kappa(dx=dx, dy=dy)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_guard_just_above_threshold(self):
        """d² just above 0.1: guard does NOT fire, κ = 2·dy/d²."""
        dx = 0.0
        dy = math.sqrt(0.1001)  # d² ≈ 0.1001 > 0.1
        kappa = compute_kappa(dx=dx, dy=dy)
        d2 = dx * dx + dy * dy
        expected = 2.0 * dy / d2
        assert kappa == pytest.approx(expected, rel=1e-5)

    def test_right_turn_negative_kappa(self):
        """Negative dy (goal to right) → negative κ → right turn."""
        kappa = compute_kappa(dx=100.0, dy=-100.0)
        assert kappa == pytest.approx(-0.01, rel=1e-6)

    def test_behind_robot(self):
        """Goal directly behind (dx<0, dy=0): κ = 0 (no lateral offset)."""
        kappa = compute_kappa(dx=-200.0, dy=0.0)
        assert kappa == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Tests — straight-ahead steering (vL = vR = v)
# ---------------------------------------------------------------------------

class TestStraightAheadSteering:
    """Verify full steering chain for goal directly ahead."""

    TRACK_WIDTH = 120.0   # mm
    SPEED       = 200.0   # mm/s

    def test_straight_ahead_no_steering(self):
        """AC: goal (300, 0), robot at origin facing +x → κ=0, ω=0, vL=vR=speed."""
        # beginGoTo(): robot at (0,0), heading=0; goal (300,0) robot-relative
        #   → world goal = (300, 0)
        # tick(): robot at (0,0), heading=0
        #   dx=300, dy=0 in robot frame
        dx, dy = 300.0, 0.0
        kappa = compute_kappa(dx, dy)
        omega = self.SPEED * kappa
        vL, vR = bk_inverse(self.SPEED, omega, self.TRACK_WIDTH)

        assert kappa == pytest.approx(0.0, abs=1e-9)
        assert omega == pytest.approx(0.0, abs=1e-9)
        assert vL == pytest.approx(self.SPEED)
        assert vR == pytest.approx(self.SPEED)

    def test_omega_proportional_to_kappa(self):
        """ω = v · κ: for goal (100, 100), ω = 200 · 0.01 = 2.0 rad/s."""
        dx, dy = 100.0, 100.0
        kappa = compute_kappa(dx, dy)
        omega = self.SPEED * kappa
        assert kappa == pytest.approx(0.01, rel=1e-6)
        assert omega == pytest.approx(self.SPEED * 0.01, rel=1e-6)


# ---------------------------------------------------------------------------
# Tests — world-to-robot-frame transform
# ---------------------------------------------------------------------------

class TestWorldToRobotTransform:
    """Verify world→robot-frame goal projection used in each PURSUE tick."""

    def test_robot_at_origin_facing_right(self):
        """Robot at (0,0,0): world goal = robot goal (identity transform)."""
        dx, dy = world_to_robot_goal(
            gx_world=300.0, gy_world=0.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        assert dx == pytest.approx(300.0, rel=1e-6)
        assert dy == pytest.approx(0.0, abs=1e-6)

    def test_robot_facing_90deg_ccw(self):
        """Robot at (0,0) facing 90° CCW (+y): world goal (300,0) → robot frame (0,-300).

        Robot's +x axis points in world +y direction.
        World goal at (300, 0) is therefore behind and right of robot:
          dx = 300*cos(90°) + 0*sin(90°) = 0
          dy = -300*sin(90°) + 0*cos(90°) = -300
        """
        h_rad = math.pi / 2.0
        dx, dy = world_to_robot_goal(
            gx_world=300.0, gy_world=0.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=h_rad
        )
        # world goal is at 90° right of robot's forward direction
        assert dx == pytest.approx(0.0, abs=1e-5)
        assert dy == pytest.approx(-300.0, rel=1e-5)

    def test_robot_translated(self):
        """Robot at (100,200,0): world goal (400,200) → robot frame (300, 0)."""
        dx, dy = world_to_robot_goal(
            gx_world=400.0, gy_world=200.0,
            robot_x=100.0, robot_y=200.0, robot_h_rad=0.0
        )
        assert dx == pytest.approx(300.0, rel=1e-6)
        assert dy == pytest.approx(0.0, abs=1e-6)

    def test_beginGoTo_roundtrip_zero_pose(self):
        """beginGoTo() world transform then tick() inverse → original robot-relative goal.

        Robot at (0,0,0): beginGoTo with (tx=300, ty=100) stores world goal.
        On first tick (robot still at 0,0,0), world_to_robot recovers (300, 100).
        """
        tx, ty = 300.0, 100.0
        gx, gy = beginGoTo_world_goal(tx, ty, robot_x=0.0, robot_y=0.0, robot_h_rad=0.0)
        dx, dy = world_to_robot_goal(gx, gy, robot_x=0.0, robot_y=0.0, robot_h_rad=0.0)
        assert dx == pytest.approx(tx, rel=1e-6)
        assert dy == pytest.approx(ty, rel=1e-6)

    def test_beginGoTo_roundtrip_nonzero_pose(self):
        """beginGoTo() + tick() at same pose recovers original goal: rotated pose."""
        tx, ty = 200.0, 150.0
        h_rad  = math.pi / 6.0   # 30° CCW
        rx, ry = 50.0, 75.0      # robot position

        gx, gy = beginGoTo_world_goal(tx, ty, robot_x=rx, robot_y=ry, robot_h_rad=h_rad)
        dx, dy = world_to_robot_goal(gx, gy, robot_x=rx, robot_y=ry, robot_h_rad=h_rad)
        assert dx == pytest.approx(tx, abs=1e-4)
        assert dy == pytest.approx(ty, abs=1e-4)

    def test_goal_directly_ahead_kappa_zero(self):
        """After transform: goal directly ahead in robot frame → κ = 0."""
        # Robot at (100, 200) facing 0: goal at world (400, 200)
        dx, dy = world_to_robot_goal(
            gx_world=400.0, gy_world=200.0,
            robot_x=100.0, robot_y=200.0, robot_h_rad=0.0
        )
        kappa = compute_kappa(dx, dy)
        assert kappa == pytest.approx(0.0, abs=1e-9)

    def test_goal_to_left_positive_kappa(self):
        """Goal to the left of robot's direction → positive κ (CCW turn)."""
        # Robot at (0,0) facing 0; goal at world (0, 300) → 90° to the left
        dx, dy = world_to_robot_goal(
            gx_world=0.0, gy_world=300.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        kappa = compute_kappa(dx, dy)
        # dy=300 > 0, so kappa > 0
        assert kappa > 0.0

    def test_goal_to_right_negative_kappa(self):
        """Goal to the right of robot's direction → negative κ (CW turn)."""
        # Robot at (0,0) facing 0; goal at world (0, -300) → 90° to the right
        dx, dy = world_to_robot_goal(
            gx_world=0.0, gy_world=-300.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        kappa = compute_kappa(dx, dy)
        assert kappa < 0.0


# ---------------------------------------------------------------------------
# Tests — turn-in-place gate (011-003)
# ---------------------------------------------------------------------------
#
# Pure Python mirror of the gate logic in DriveController::beginGoTo()
# and DriveController::tick() PRE_ROTATE branch.
#
# beginGoTo() gate:
#   bearing = abs(atan2(ty, tx))  — in robot frame at command time
#   gateRad = turnInPlaceGate * (pi/180)
#   if bearing > gateRad: PRE_ROTATE; else: PURSUE
#
# tick() PRE_ROTATE continuous check:
#   dx_rf =  dxW*cos(h) + dyW*sin(h)
#   dy_rf = -dxW*sin(h) + dyW*cos(h)
#   bearing = abs(atan2(dy_rf, dx_rf))
#   if bearing <= gateRad: transition to PURSUE
# ---------------------------------------------------------------------------

DEFAULT_GATE_DEG = 45.0  # default turnInPlaceGate in degrees


def begin_goto_bearing(tx: float, ty: float) -> float:
    """Robot-frame bearing to the goal at command time (radians, always ≥ 0)."""
    return abs(math.atan2(ty, tx))


def gate_fires(tx: float, ty: float, gate_deg: float = DEFAULT_GATE_DEG) -> bool:
    """Return True when PRE_ROTATE fires (bearing > gate), False for PURSUE."""
    bearing = begin_goto_bearing(tx, ty)
    gate_rad = gate_deg * (math.pi / 180.0)
    return bearing > gate_rad


def turn_sign(ty: float) -> float:
    """Sign of the in-place rotation: +1 CCW (ty ≥ 0), -1 CW (ty < 0)."""
    return 1.0 if ty >= 0.0 else -1.0


def tick_pre_rotate_bearing(
    gx_world: float,
    gy_world: float,
    robot_x: float,
    robot_y: float,
    robot_h_rad: float,
) -> float:
    """Compute the bearing used by the PRE_ROTATE tick to check exit condition."""
    dxW  = gx_world - robot_x
    dyW  = gy_world - robot_y
    dx_rf =  dxW * math.cos(robot_h_rad) + dyW * math.sin(robot_h_rad)
    dy_rf = -dxW * math.sin(robot_h_rad) + dyW * math.cos(robot_h_rad)
    return abs(math.atan2(dy_rf, dx_rf))


class TestTurnInPlaceGate:
    """AC tests for the turn-in-place gate decision (011-003)."""

    def test_target_behind_gate_fires(self):
        """AC: target at (tx=-300, ty=0) → bearing = π > 45° gate → PRE_ROTATE."""
        tx, ty = -300.0, 0.0
        bearing = begin_goto_bearing(tx, ty)
        assert bearing == pytest.approx(math.pi, rel=1e-6)
        assert gate_fires(tx, ty), "Bearing π should exceed 45° gate"

    def test_target_ahead_slight_left_no_gate(self):
        """AC: target at (tx=300, ty=10) → bearing ≈ 1.9° < 45° gate → PURSUE."""
        tx, ty = 300.0, 10.0
        bearing_deg = math.degrees(begin_goto_bearing(tx, ty))
        assert bearing_deg == pytest.approx(math.degrees(math.atan2(10.0, 300.0)), rel=1e-4)
        assert bearing_deg < 45.0, f"Bearing {bearing_deg:.2f}° should be below 45° gate"
        assert not gate_fires(tx, ty), "Small bearing should NOT trigger gate"

    def test_target_90deg_left_gate_fires(self):
        """AC: target at (tx=0, ty=300) → bearing = 90° > 45° gate → PRE_ROTATE."""
        tx, ty = 0.0, 300.0
        bearing = begin_goto_bearing(tx, ty)
        assert bearing == pytest.approx(math.pi / 2.0, rel=1e-6)
        assert gate_fires(tx, ty), "90° bearing should exceed 45° gate"

    def test_custom_gate_threshold(self):
        """AC: gate=30° makes target at (tx=200, ty=150) ≈ 36.87° trigger PRE_ROTATE."""
        tx, ty = 200.0, 150.0
        bearing_deg = math.degrees(begin_goto_bearing(tx, ty))
        # atan2(150, 200) ≈ 36.87°
        assert bearing_deg == pytest.approx(36.87, abs=0.02)
        # With default 45° gate: does NOT fire
        assert not gate_fires(tx, ty, gate_deg=45.0), "36.87° should be below 45° gate"
        # With 30° gate: fires
        assert gate_fires(tx, ty, gate_deg=30.0), "36.87° should exceed 30° gate"

    def test_rotation_direction_left_target(self):
        """Target at (tx=0, ty=300) → ty ≥ 0 → CCW rotation (turnSign = +1)."""
        assert turn_sign(ty=300.0) == 1.0

    def test_rotation_direction_right_target(self):
        """Target at (tx=0, ty=-300) → ty < 0 → CW rotation (turnSign = -1)."""
        assert turn_sign(ty=-300.0) == -1.0

    def test_rotation_direction_target_behind_left(self):
        """Target at (tx=-300, ty=1) → ty ≥ 0 → CCW rotation."""
        assert turn_sign(ty=1.0) == 1.0

    def test_rotation_direction_target_behind_right(self):
        """Target at (tx=-300, ty=-1) → ty < 0 → CW rotation."""
        assert turn_sign(ty=-1.0) == -1.0

    def test_rotation_direction_ty_zero(self):
        """ty = 0 (target directly ahead or behind, no lateral offset): CCW by convention."""
        assert turn_sign(ty=0.0) == 1.0

    def test_gate_boundary_exactly_at_gate(self):
        """Bearing exactly at gate (45°) does NOT fire PRE_ROTATE (> not >=)."""
        # atan2(1, 1) = 45°: bearing == gate, so NOT > gate → PURSUE
        tx, ty = 100.0, 100.0
        bearing_deg = math.degrees(begin_goto_bearing(tx, ty))
        assert bearing_deg == pytest.approx(45.0, abs=1e-6)
        assert not gate_fires(tx, ty, gate_deg=45.0), "Bearing == gate should NOT fire"

    def test_gate_boundary_just_above(self):
        """Bearing just above 45° fires PRE_ROTATE."""
        tx, ty = 100.0, 100.01   # slightly more than 45°
        assert gate_fires(tx, ty, gate_deg=45.0)


class TestTurnInPlaceTickBearing:
    """Tests for the PRE_ROTATE continuous bearing check (tick logic, 011-003)."""

    def test_bearing_within_gate_exits_to_pursue(self):
        """When robot has rotated enough, bearing ≤ gate → would transition to PURSUE."""
        gate_deg = 45.0
        gate_rad = gate_deg * (math.pi / 180.0)
        # World goal is at (300, 0), robot at (0,0) facing 0 → bearing = 0 → within gate
        bearing = tick_pre_rotate_bearing(
            gx_world=300.0, gy_world=0.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=0.0
        )
        assert bearing <= gate_rad, "Bearing 0 should be within gate"

    def test_bearing_beyond_gate_stays_pre_rotate(self):
        """When robot still faces away, bearing > gate → stays in PRE_ROTATE."""
        gate_deg = 45.0
        gate_rad = gate_deg * (math.pi / 180.0)
        # Robot at origin facing +y (π/2 CCW); world goal at (300, 0)
        # Robot frame: dx=0, dy=-300 → bearing = π/2 > 45° gate
        bearing = tick_pre_rotate_bearing(
            gx_world=300.0, gy_world=0.0,
            robot_x=0.0, robot_y=0.0, robot_h_rad=math.pi / 2.0
        )
        assert bearing > gate_rad, f"Bearing {math.degrees(bearing):.1f}° should exceed gate"

    def test_tick_bearing_equals_begin_goto_bearing_at_start(self):
        """At command time, tick bearing = beginGoTo bearing (same geometry)."""
        tx, ty = 0.0, 300.0      # 90° left target
        robot_x, robot_y, h_rad = 0.0, 0.0, 0.0

        # beginGoTo bearing (robot frame at command time)
        begin_bearing = begin_goto_bearing(tx, ty)

        # World goal
        gx = robot_x + tx * math.cos(h_rad) - ty * math.sin(h_rad)
        gy = robot_y + tx * math.sin(h_rad) + ty * math.cos(h_rad)

        # tick bearing at same pose
        tick_bearing = tick_pre_rotate_bearing(gx, gy, robot_x, robot_y, h_rad)

        assert tick_bearing == pytest.approx(begin_bearing, rel=1e-6)

    def test_tick_bearing_decreases_as_robot_rotates_ccw(self):
        """As the robot rotates CCW toward a 90°-left goal, bearing decreases."""
        # World goal: (0, 300) (directly +y in world, 90° left when robot faces +x)
        gx_world, gy_world = 0.0, 300.0

        # Robot rotates CCW: h_rad increases from 0 toward π/2
        bearing_0   = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, 0.0)
        bearing_30  = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, math.pi / 6.0)
        bearing_45  = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, math.pi / 4.0)
        bearing_90  = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, math.pi / 2.0)

        assert bearing_0  == pytest.approx(math.pi / 2.0, rel=1e-6)
        assert bearing_30  < bearing_0,  "Bearing should decrease as robot rotates CCW"
        assert bearing_45  < bearing_30, "Bearing should decrease as robot rotates CCW"
        assert bearing_90  == pytest.approx(0.0, abs=1e-6)  # robot now faces the goal

    def test_target_directly_behind_both_rotation_paths(self):
        """For target directly behind (tx=-300, ty=0), rotation CCW or CW both reduce bearing."""
        # World goal: (-300, 0) (directly behind robot facing +x)
        gx_world, gy_world = -300.0, 0.0

        bearing_at_0   = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, 0.0)
        assert bearing_at_0 == pytest.approx(math.pi, rel=1e-6)

        # After 90° CCW rotation: goal is 90° to the right of robot forward
        bearing_ccw90  = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, math.pi / 2.0)
        assert bearing_ccw90 == pytest.approx(math.pi / 2.0, rel=1e-5)

        # After 180° rotation: goal is directly ahead
        bearing_180    = tick_pre_rotate_bearing(gx_world, gy_world, 0.0, 0.0, math.pi)
        assert bearing_180 == pytest.approx(0.0, abs=1e-5)
