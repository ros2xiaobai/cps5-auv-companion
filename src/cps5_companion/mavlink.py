"""Read-only MAVLink telemetry connection used by the safe monitor mode."""

from __future__ import annotations

import time
from typing import Any

from .config import MavlinkConfig
from .models import VehicleTelemetry


class MavlinkMonitor:
    """Receive telemetry without exposing actuation methods."""

    def __init__(self, config: MavlinkConfig) -> None:
        self.config = config
        self.connection: Any | None = None
        self.telemetry = VehicleTelemetry()

    def connect(self) -> None:
        if self.connection is not None:
            return
        from pymavlink import mavutil

        connection = mavutil.mavlink_connection(self.config.endpoint)
        connection.wait_heartbeat(timeout=self.config.heartbeat_timeout_s)
        self.connection = connection
        self.telemetry.heartbeat_seen = True

    def poll(self) -> VehicleTelemetry:
        if self.connection is None:
            return self.telemetry

        while True:
            message = self.connection.recv_match(
                type=["HEARTBEAT", "VFR_HUD", "AHRS2"],
                blocking=False,
            )
            if message is None:
                break

            message_type = message.get_type()
            if message_type == "HEARTBEAT":
                self.telemetry.heartbeat_seen = True
            elif message_type == "VFR_HUD":
                self.telemetry.heading_deg = float(message.heading)
                self.telemetry.heading_received_at = time.monotonic()
            elif message_type == "AHRS2":
                # Keep the MAVLink field's real name. Its relationship to water
                # depth and sign convention still requires vehicle validation.
                self.telemetry.altitude_m = float(message.altitude)
                self.telemetry.altitude_received_at = time.monotonic()

        return self.telemetry

    def close(self) -> None:
        connection = self.connection
        self.connection = None
        if connection is not None and hasattr(connection, "close"):
            connection.close()
