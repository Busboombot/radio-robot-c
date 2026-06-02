"""Discrete PID controller and angle utilities."""

import math


class PID:
    """Discrete PID controller with output clamping and integral windup guard."""

    def __init__(self, kp, ki, kd, out_min=-100, out_max=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def update(self, error, now):
        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = error
            return max(self.out_min, min(self.out_max, self.kp * error))

        dt = now - self.prev_time
        if dt <= 0:
            return max(self.out_min, min(self.out_max, self.kp * error))

        self.integral += error * dt
        self.integral = max(-50 / max(self.ki, 0.001),
                            min(50 / max(self.ki, 0.001), self.integral))

        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.out_min, min(self.out_max, output))

        self.prev_error = error
        self.prev_time = now
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None


def normalize_angle(a):
    """Normalize angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a
