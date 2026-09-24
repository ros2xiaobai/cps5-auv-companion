"""Hardware-independent PID controller."""

from __future__ import annotations

import time


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
        integral_min: float = -20.0,
        integral_max: float = 20.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None

    def compute(
        self,
        setpoint: float,
        current_value: float,
        *,
        now: float | None = None,
    ) -> float:
        current_time = time.monotonic() if now is None else now
        error = setpoint - current_value

        if self.last_time is None:
            dt = 0.0
            derivative = 0.0
        else:
            dt = max(current_time - self.last_time, 1e-9)
            derivative = self.kd * (error - self.previous_error) / dt

        candidate_integral = self.integral + error * dt
        if self.ki != 0:
            lower = self.integral_min / self.ki
            upper = self.integral_max / self.ki
            if lower > upper:
                lower, upper = upper, lower
            self.integral = max(lower, min(candidate_integral, upper))
        else:
            self.integral = 0.0

        output = self.kp * error + self.ki * self.integral + derivative
        self.previous_error = error
        self.last_time = current_time
        return max(self.output_min, min(output, self.output_max))
