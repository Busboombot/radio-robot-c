"""robot_radio.kinematics — kinematics subpackage.

Optional exports (require wpimath):
  DifferentialDriveKinematics — lazy import.
"""


def __getattr__(name: str):
    """Lazy import for wpimath-dependent kinematics."""
    if name == "DifferentialDriveKinematics":
        from robot_radio.kinematics.differential_drive import DifferentialDriveKinematics
        return DifferentialDriveKinematics
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "DifferentialDriveKinematics",  # lazy
]
