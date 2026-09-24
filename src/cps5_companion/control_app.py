"""Explicitly gated integration of sensors, mission control and MAVLink output."""

from __future__ import annotations

import logging
import time

from .actuation import MavlinkActuator
from .config import VehicleConfig
from .control.controller import MissionController, SensorSnapshot
from .control.mission import MissionSegment, SegmentState
from .dvl import DvlClient
from .mavlink import MavlinkMonitor
from .models import DvlPosition

LOGGER = logging.getLogger(__name__)


class ControlRuntimeError(RuntimeError):
    """Raised when the active control session cannot continue safely."""


class CompanionControlApp:
    def __init__(self, config: VehicleConfig, segments: list[MissionSegment]) -> None:
        if not config.dvl.enabled:
            raise ValueError("mission control requires dvl.enabled=true")
        self.config = config
        self.controller = MissionController(segments, config.control)
        self.mavlink = MavlinkMonitor(config.mavlink)
        self.dvl = DvlClient(config.dvl)
        self.actuator = MavlinkActuator(self.mavlink, config.control)
        self.last_position: DvlPosition | None = None
        self.last_position_received_at: float | None = None

    def run(self) -> None:
        arm_attempted = False
        try:
            self.mavlink.connect()
            self.dvl.connect()
            initial = self._wait_for_fresh_sensors()
            LOGGER.info(
                "Pre-arm sensors ready: heading=%s depth=%s position_x=%s",
                initial.heading_deg,
                initial.depth_m,
                initial.position_x_m,
            )

            self.actuator.set_mode()
            arm_attempted = True
            self.actuator.arm()
            LOGGER.warning("Vehicle armed; mission control is active")

            previous_state: SegmentState | None = None
            while True:
                snapshot = self._read_snapshot()
                setpoint = self.controller.step(snapshot)
                self.actuator.send_setpoint(setpoint)
                if setpoint.state != previous_state:
                    LOGGER.info(
                        "Mission state=%s segment=%s",
                        setpoint.state.value,
                        setpoint.segment_index,
                    )
                    previous_state = setpoint.state
                if setpoint.fault is not None:
                    raise ControlRuntimeError(setpoint.fault)
                if setpoint.complete:
                    LOGGER.info("Mission completed")
                    return
                time.sleep(self.config.monitoring.poll_interval_s)
        finally:
            if arm_attempted:
                try:
                    self.actuator.neutralize()
                except Exception:
                    LOGGER.exception("Failed to send neutral outputs")
                try:
                    self.actuator.disarm()
                except Exception:
                    LOGGER.exception("Failed to confirm disarm")
                try:
                    self.actuator.release_override()
                except Exception:
                    LOGGER.exception("Failed to release RC override")
            self.dvl.close()
            self.mavlink.close()

    def _wait_for_fresh_sensors(self) -> SensorSnapshot:
        deadline = time.monotonic() + self.config.control.startup_timeout_s
        last_snapshot: SensorSnapshot | None = None
        while time.monotonic() < deadline:
            last_snapshot = self._read_snapshot()
            if self._snapshot_is_fresh(last_snapshot):
                return last_snapshot
            time.sleep(self.config.monitoring.poll_interval_s)
        raise TimeoutError(f"pre-arm sensor readiness timed out; last snapshot={last_snapshot}")

    def _snapshot_is_fresh(self, snapshot: SensorSnapshot) -> bool:
        timeout = self.config.control.sensor_timeout_s
        pairs = (
            (snapshot.heading_deg, snapshot.heading_received_at),
            (snapshot.depth_m, snapshot.depth_received_at),
            (snapshot.position_x_m, snapshot.position_received_at),
        )
        return all(
            value is not None
            and received_at is not None
            and 0.0 <= snapshot.now - received_at <= timeout
            for value, received_at in pairs
        )

    def _read_snapshot(self) -> SensorSnapshot:
        telemetry = self.mavlink.poll()
        for report in self.dvl.poll():
            position = DvlPosition.from_report(report)
            if position is not None:
                self.last_position = position
                self.last_position_received_at = time.monotonic()

        now = time.monotonic()
        depth_m = None
        if telemetry.altitude_m is not None:
            depth_m = (
                self.config.depth.scale * telemetry.altitude_m
                + self.config.depth.offset_m
            )
        position_x_m = None
        if self.last_position is not None:
            position_x_m = self.config.dvl.forward_sign * self.last_position.x_m

        return SensorSnapshot(
            now=now,
            heading_deg=telemetry.heading_deg,
            heading_received_at=telemetry.heading_received_at,
            depth_m=depth_m,
            depth_received_at=telemetry.altitude_received_at,
            position_x_m=position_x_m,
            position_received_at=self.last_position_received_at,
        )
