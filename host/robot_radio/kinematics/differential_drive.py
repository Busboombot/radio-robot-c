from wpimath.kinematics import DifferentialDriveKinematics as _WpiDDK
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds


class DifferentialDriveKinematics:
    """Wraps wpimath.kinematics.DifferentialDriveKinematics.

    All inputs/outputs use project conventions:
    - distances in mm/s (not m/s)
    - yaw/omega: CW-positive radians (not CCW)
    """

    def __init__(self, trackwidth_mm: float):
        self._tw_m = trackwidth_mm / 1000.0
        self._kinematics = _WpiDDK(self._tw_m)

    def inverse(self, vx_ms: float, omega_rads: float) -> tuple[float, float]:
        """(vx m/s, omega CW rad/s) -> (v_left mm/s, v_right mm/s)"""
        # WPILib uses CCW-positive omega; negate for our CW convention
        speeds = ChassisSpeeds(vx_ms, 0.0, -omega_rads)
        ws = self._kinematics.toWheelSpeeds(speeds)
        return ws.left * 1000.0, ws.right * 1000.0

    def forward(self, v_left_mms: float, v_right_mms: float) -> tuple[float, float]:
        """(v_left mm/s, v_right mm/s) -> (vx m/s, omega CW rad/s)"""
        ws = DifferentialDriveWheelSpeeds(v_left_mms / 1000.0, v_right_mms / 1000.0)
        chassis = self._kinematics.toChassisSpeeds(ws)
        # Negate omega back to CW-positive
        return chassis.vx, -chassis.omega
