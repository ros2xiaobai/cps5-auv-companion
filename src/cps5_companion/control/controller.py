"""Hardware-independent mission state machine and control calculations."""

from __future__ import annotations

from dataclasses import dataclass

from ..config import ControlConfig
from .mission import MissionSegment, SegmentState
from .pid import PIDController


@dataclass(frozen=True)
class SensorSnapshot:
    now: float
    heading_deg: float | None
    heading_received_at: float | None
    depth_m: float | None
    depth_received_at: float | None
    position_x_m: float | None
    position_received_at: float | None


@dataclass(frozen=True)
class ControlSetpoint:
    state: SegmentState
    segment_index: int
    throttle_pwm: int
    forward_pwm: int
    target_yaw_deg: float | None
    progress_m: float | None = None
    remaining_m: float | None = None
    complete: bool = False
    fault: str | None = None


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180)."""

    return (angle_deg + 180.0) % 360.0 - 180.0


class MissionController:
    """Deterministic mission controller with no network or hardware access."""

    def __init__(self, segments: list[MissionSegment], config: ControlConfig) -> None:
        if not segments:
            raise ValueError("at least one mission segment is required")
        if any(segment.tasks for segment in segments):
            raise ValueError("automatic task actions are not enabled in the safe controller")
        self.segments = tuple(segments)
        self.config = config
        self.state = SegmentState.ADJUST_DEPTH
        self.segment_index = 0
        self.state_started_at: float | None = None
        self.position_origin_m: float | None = None
        self.fault: str | None = None
        self.depth_pid = PIDController(
            config.depth_kp,
            config.depth_ki,
            config.depth_kd,
            -float(config.depth_max_pwm_delta),
            float(config.depth_max_pwm_delta),
        )

    @property
    def segment(self) -> MissionSegment:
        return self.segments[self.segment_index]

    def step(self, sensors: SensorSnapshot) -> ControlSetpoint:
        if self.state == SegmentState.COMPLETE:
            return self._terminal_setpoint(complete=True)
        if self.state == SegmentState.FAILED:
            return self._terminal_setpoint(fault=self.fault or "mission failed")
        if self.state_started_at is None:
            self.state_started_at = sensors.now

        if not self._fresh(
            sensors.depth_m,
            sensors.depth_received_at,
            sensors.now,
        ):
            return self._fail("depth telemetry is missing or stale")

        segment = self.segment
        throttle_pwm = self._depth_pwm(segment.depth_m, float(sensors.depth_m), sensors.now)

        if self.state == SegmentState.ADJUST_DEPTH:
            if abs(segment.depth_m - float(sensors.depth_m)) <= self.config.depth_threshold_m:
                self._enter(SegmentState.ADJUST_HEADING, sensors.now)
            return self._setpoint(throttle_pwm=throttle_pwm)

        if self.state == SegmentState.ADJUST_HEADING:
            if not self._fresh(
                sensors.heading_deg,
                sensors.heading_received_at,
                sensors.now,
            ):
                return self._fail("heading telemetry is missing or stale")
            heading_error = abs(
                normalize_angle(segment.heading_deg - float(sensors.heading_deg))
            )
            if heading_error <= self.config.heading_threshold_deg:
                self._enter(SegmentState.CAPTURE_DVL_ORIGIN, sensors.now)
            elif sensors.now - float(self.state_started_at) > self.config.heading_timeout_s:
                return self._fail("heading adjustment timed out")
            return self._setpoint(
                throttle_pwm=throttle_pwm,
                target_yaw_deg=segment.heading_deg,
            )

        if self.state == SegmentState.CAPTURE_DVL_ORIGIN:
            if not self._fresh(
                sensors.position_x_m,
                sensors.position_received_at,
                sensors.now,
            ):
                return self._fail("DVL position is missing or stale")
            self.position_origin_m = float(sensors.position_x_m)
            self._enter(SegmentState.MOVE_FORWARD, sensors.now)
            return self._setpoint(
                throttle_pwm=throttle_pwm,
                target_yaw_deg=segment.heading_deg,
            )

        if self.state == SegmentState.MOVE_FORWARD:
            if not self._fresh(
                sensors.heading_deg,
                sensors.heading_received_at,
                sensors.now,
            ):
                return self._fail("heading telemetry is missing or stale")
            if not self._fresh(
                sensors.position_x_m,
                sensors.position_received_at,
                sensors.now,
            ):
                return self._fail("DVL position is missing or stale")
            if self.position_origin_m is None:
                return self._fail("DVL origin was not captured")

            progress = float(sensors.position_x_m) - self.position_origin_m
            remaining = segment.distance_m - progress
            if abs(remaining) <= self.config.distance_threshold_m:
                self._advance(sensors.now)
                if self.state == SegmentState.COMPLETE:
                    return self._terminal_setpoint(complete=True)
                return self._setpoint(
                    progress_m=progress,
                    remaining_m=remaining,
                )
            if sensors.now - float(self.state_started_at) > self.config.move_timeout_s:
                return self._fail("forward movement timed out")

            delta = self.config.forward_kp * remaining
            limit = float(self.config.forward_max_pwm_delta)
            delta = max(-limit, min(delta, limit))
            forward_pwm = self._clamp_pwm(self.config.rc_neutral + delta)
            return self._setpoint(
                throttle_pwm=throttle_pwm,
                forward_pwm=forward_pwm,
                target_yaw_deg=segment.heading_deg,
                progress_m=progress,
                remaining_m=remaining,
            )

        return self._fail(f"unsupported mission state: {self.state}")

    def _fresh(
        self,
        value: float | None,
        received_at: float | None,
        now: float,
    ) -> bool:
        return (
            value is not None
            and received_at is not None
            and 0.0 <= now - received_at <= self.config.sensor_timeout_s
        )

    def _depth_pwm(self, target_depth: float, current_depth: float, now: float) -> int:
        delta = self.depth_pid.compute(target_depth, current_depth, now=now)
        return self._clamp_pwm(self.config.rc_neutral + delta)

    def _clamp_pwm(self, value: float) -> int:
        return round(max(self.config.rc_min, min(value, self.config.rc_max)))

    def _enter(self, state: SegmentState, now: float) -> None:
        self.state = state
        self.state_started_at = now

    def _advance(self, now: float) -> None:
        self.segment_index += 1
        self.position_origin_m = None
        self.depth_pid.reset()
        if self.segment_index >= len(self.segments):
            self.state = SegmentState.COMPLETE
            self.state_started_at = now
        else:
            self._enter(SegmentState.ADJUST_DEPTH, now)

    def _fail(self, reason: str) -> ControlSetpoint:
        self.state = SegmentState.FAILED
        self.fault = reason
        return self._terminal_setpoint(fault=reason)

    def _setpoint(
        self,
        *,
        throttle_pwm: int | None = None,
        forward_pwm: int | None = None,
        target_yaw_deg: float | None = None,
        progress_m: float | None = None,
        remaining_m: float | None = None,
    ) -> ControlSetpoint:
        return ControlSetpoint(
            state=self.state,
            segment_index=self.segment_index,
            throttle_pwm=self.config.rc_neutral if throttle_pwm is None else throttle_pwm,
            forward_pwm=self.config.rc_neutral if forward_pwm is None else forward_pwm,
            target_yaw_deg=target_yaw_deg,
            progress_m=progress_m,
            remaining_m=remaining_m,
        )

    def _terminal_setpoint(
        self,
        *,
        complete: bool = False,
        fault: str | None = None,
    ) -> ControlSetpoint:
        return ControlSetpoint(
            state=self.state,
            segment_index=min(self.segment_index, len(self.segments) - 1),
            throttle_pwm=self.config.rc_neutral,
            forward_pwm=self.config.rc_neutral,
            target_yaw_deg=None,
            complete=complete,
            fault=fault,
        )
