"""Explicit MAVLink actuation adapter; never constructed by monitor mode."""

from __future__ import annotations

import math
import time
from typing import Any

from .config import ControlConfig
from .control.controller import ControlSetpoint
from .mavlink import MavlinkMonitor


class MavlinkActuator:
    def __init__(self, monitor: MavlinkMonitor, config: ControlConfig) -> None:
        self.monitor = monitor
        self.config = config
        self.boot_time = time.monotonic()

    @property
    def connection(self) -> Any:
        connection = self.monitor.connection
        if connection is None:
            raise RuntimeError("MAVLink is not connected")
        return connection

    def set_mode(self) -> None:
        mode_id = self.connection.mode_mapping().get(self.config.mode)
        if mode_id is None:
            raise RuntimeError(f"flight mode is unavailable: {self.config.mode}")
        self.connection.set_mode(mode_id)
        deadline = time.monotonic() + self.config.mode_timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            heartbeat = self.connection.wait_heartbeat(timeout=min(1.0, remaining))
            if heartbeat is not None and getattr(heartbeat, "custom_mode", None) == mode_id:
                return
        raise TimeoutError(f"vehicle did not confirm flight mode: {self.config.mode}")

    def arm(self) -> None:
        self._send_arm_command(armed=True)
        self._wait_for_armed_state(armed=True)

    def disarm(self) -> None:
        self._send_arm_command(armed=False)
        self._wait_for_armed_state(armed=False)

    def _send_arm_command(self, *, armed: bool) -> None:
        from pymavlink import mavutil

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if armed else 0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _wait_for_armed_state(self, *, armed: bool) -> None:
        deadline = time.monotonic() + self.config.arm_timeout_s
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.connection.wait_heartbeat(timeout=min(1.0, remaining))
            if bool(self.connection.motors_armed()) is armed:
                return
        state = "armed" if armed else "disarmed"
        raise TimeoutError(f"vehicle did not report {state} state")

    def send_setpoint(self, setpoint: ControlSetpoint) -> None:
        if setpoint.target_yaw_deg is not None:
            self._send_attitude_target(setpoint.target_yaw_deg)
        self._send_rc_override(setpoint.throttle_pwm, setpoint.forward_pwm)

    def neutralize(self, repeats: int = 5) -> None:
        for _ in range(repeats):
            self._send_rc_override(self.config.rc_neutral, self.config.rc_neutral)
            time.sleep(0.05)

    def release_override(self) -> None:
        """Release every RC override channel back to the flight controller/GCS."""

        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            *([65535] * 8),
        )

    def _send_attitude_target(self, yaw_deg: float) -> None:
        from pymavlink import mavutil
        from pymavlink.quaternion import QuaternionBase

        quaternion = QuaternionBase([0.0, 0.0, math.radians(yaw_deg)])
        time_boot_ms = int((time.monotonic() - self.boot_time) * 1000) & 0xFFFFFFFF
        self.connection.mav.set_attitude_target_send(
            time_boot_ms,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            quaternion,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _send_rc_override(self, throttle_pwm: int, forward_pwm: int) -> None:
        channels = [65535] * 8
        channels[self.config.throttle_channel - 1] = throttle_pwm
        channels[self.config.yaw_channel - 1] = self.config.rc_neutral
        channels[self.config.forward_channel - 1] = forward_pwm
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            *channels,
        )
