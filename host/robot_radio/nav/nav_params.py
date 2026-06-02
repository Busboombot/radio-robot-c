"""Navigation parameters as a structured dataclass.

Stanley-specific fields (tunable at runtime via the ``tune`` MCP tool):

- ``stanley_k`` — cross-track error gain (default 0.8).
- ``stanley_v_soft`` — denominator regularisation in motor command units
  (default 0.1).  Prevents division-by-zero when ``base_speed`` is zero.
- ``stanley_omega_gain`` — steering-angle to angular-velocity scale factor
  (default 2.0).
"""

import dataclasses
from dataclasses import dataclass, asdict


@dataclass
class NavParams:
    """Tunable navigation parameters. All values are floats."""

    # Speed PID
    speed_kp: float = 1.5
    speed_ki: float = 0.8
    speed_kd: float = 0.3
    # Steering PID
    steer_kp: float = 20.0
    steer_ki: float = 0.0
    steer_kd: float = 0.0
    # Limits
    max_speed: float = 50
    stop_dist: float = 9.0
    ff_overshoot: float = 0.0
    # Turn phase
    turn_threshold: float = 15
    # Crawl mode
    crawl_dist: float = 9.0
    crawl_cm_per_pulse: float = 0.3
    crawl_deg_per_pulse: float = 0.8
    crawl_speed: float = 50
    crawl_ms: float = 20
    crawl_spin_max: float = 2
    crawl_fwd_max: float = 15
    crawl_angle: float = 25
    # Navigate loop
    cmd_duration_ms: float = 150
    nav_loop_hz: float = 15
    frames_before_stop: float = 5
    crawl_turns: float = 0
    # Path planning
    bezier_tangent_frac: float = 0.33
    path_spacing_cm: float = 1.0
    # Spin-align phase (follow_pose_path)
    spin_align_threshold_deg: float = 90.0
    spin_align_tolerance_deg: float = 15.0
    spin_align_max_frames: float = 60.0
    # Final-turn phase (follow_pose_path)
    final_turn_tolerance_deg: float = 5.0
    final_turn_speed: float = 45.0
    final_turn_max_frames: float = 60.0
    # Shared spin speed used by both phases (when not overridden)
    spin_speed: float = 45.0
    # OTOS fallback
    otos_fallback_enabled: bool = False
    otos_fallback_max_age_s: float = 2.0
    # Stanley controller
    stanley_k: float = 0.8
    stanley_v_soft: float = 0.1
    stanley_omega_gain: float = 2.0

    def as_dict(self) -> dict[str, float]:
        return asdict(self)

    def update(self, **kwargs) -> dict:
        """Update parameters by name. Returns dict of changed values.

        Bool fields are coerced to bool; all other fields are coerced to float.
        """
        # Build a lookup from field name → field type
        field_types = {f.name: f.type for f in dataclasses.fields(self)}
        updated = {}
        for k, v in kwargs.items():
            if hasattr(self, k):
                ftype = field_types.get(k, "float")
                if ftype is bool or ftype == "bool":
                    coerced = bool(v)
                else:
                    coerced = float(v)
                setattr(self, k, coerced)
                updated[k] = coerced
        return updated
