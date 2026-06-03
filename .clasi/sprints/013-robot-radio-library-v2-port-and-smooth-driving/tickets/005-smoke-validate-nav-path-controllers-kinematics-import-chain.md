---
id: '005'
title: Smoke-validate nav/path/controllers/kinematics import chain
status: done
use-cases:
  - SUC-001
depends-on:
  - '004'
github-issue: ''
issue: ''
completes_issue: false
---

# Smoke-validate nav/path/controllers/kinematics import chain

## Description

The nav/path/controllers/kinematics modules sit above the wire layer and depend only on the `Nezha`/pose API. These modules are already present in `host/robot_radio/` (confirmed in T001). This ticket adds a minimal smoke-test layer to confirm the full import chain works and that each module constructs without error using basic inputs.

Deep v2 integration testing (e.g., `PurePursuitController` calling `stream_drive` on a real robot) is explicitly deferred to a follow-up sprint. This ticket only validates that nothing is broken at the import and construction level.

## Acceptance Criteria

- [x] All nav/path/controllers/kinematics modules import without error or fail with a CLEAR ImportError for missing optional deps: `nav.pose`, `nav.pose_align`, `nav.nav_params`, `path.arc`, `path.bezier` (numpy OK), `path.builder`, `path.catmull_rom`, `path.obstacle`, `path.patterns`, `path.sampled_path`, `path.path_helper`, `controllers.base`, `controllers.pid`, `controllers.pure_pursuit`, `controllers.stanley` all import cleanly; `controllers.ltv` and `kinematics.differential_drive` raise `ImportError: No module named 'wpimath'` (wpimath is not installed — clear error, not AttributeError); `nav.navigator` (lazy-guarded; imports aprilcam which IS available — verified structurally, not imported in the test suite to avoid sys.modules contamination with cv2).
- [x] A minimal smoke test constructs one instance from each available module: `PID`, `PurePursuitTracker`, `StanleyController`, `SampledPath`, `BezierPathBuilder`, `Pose`, `Waypoint`, `NavParams`, `compute_arc`, `catmull_rom` — all using representative dummy parameters, no exception raised. `DifferentialDriveKinematics` and `LTVController` require wpimath (not installed); their lazy `__getattr__` raises clear `ImportError`.
- [x] No nav/path/controller module imports from `robot_radio.robot.protocol` directly (confirmed by grep — all are pure math or Nezha-level API).
- [x] `uv run --with pytest python -m pytest host/tests` — 395 tests pass, 0 fail, ~0.95s.
- [x] Deep v2 validation is explicitly deferred: documented in the sprint architecture (Section 7, "Deep nav/path validation"), the module docstrings in `test_imports_smoke.py`, and the ticket description.

## Implementation Plan

**Approach**: Attempt imports; fix any broken `__init__.py` or missing dependency. Create `host/tests/test_nav_smoke.py`.

**Files to modify** (only if broken):
- Any `__init__.py` in `nav/`, `path/`, `controllers/`, `kinematics/` that has broken imports.
- Individual module files only if import fails due to removed v1 symbols (e.g., a v1 protocol constant that no longer exists).

**Files to create**:
- `host/tests/test_nav_smoke.py` — import smoke and construction tests.

**New test cases in `test_nav_smoke.py`**:
- `test_nav_imports` — `import robot_radio.nav.navigator` etc.; assert no exception.
- `test_path_imports` — `import robot_radio.path.arc` etc.; assert no exception.
- `test_controllers_imports` — import all controller submodules; assert no exception.
- `test_kinematics_imports` — `from robot_radio.kinematics.differential_drive import DifferentialDrive`; assert no exception.
- `test_pid_construct` — `PidController(kp=1.0, ki=0.0, kd=0.0)`; no exception.
- `test_differential_drive_construct` — `DifferentialDrive(trackwidth_mm=126.0, mm_per_deg=0.484)`; no exception.

**Testing plan**: Run `uv run --with pytest python -m pytest host/tests/test_nav_smoke.py -v` then full suite.
