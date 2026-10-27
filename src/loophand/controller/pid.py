from __future__ import annotations

import numpy as np


class BoundedPID:
    """
    Simple PID controller with integral windup clamp and derivative on measurement.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.integral_limit = abs(float(integral_limit))
        self._integral = np.zeros(2, dtype=float)
        self._prev_error = np.zeros(2, dtype=float)

    def reset(self) -> None:
        self._integral[:] = 0.0
        self._prev_error[:] = 0.0

    def update(self, error: np.ndarray, dt: float) -> np.ndarray:
        if error.shape != (2,):
            raise ValueError("PID error vector must be length-2 (Fx, Fy)")
        self._integral += error * dt #误差积分
        self._integral = np.clip(self._integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self._prev_error) / max(dt, 1e-6) #误差微分
        self._prev_error = error.copy()
        return (
            self.kp * error
            + self.ki * self._integral
            + self.kd * derivative
        )


__all__ = ["BoundedPID"]
