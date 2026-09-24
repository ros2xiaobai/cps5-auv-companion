"""Safe read-only monitor application."""

from __future__ import annotations

import logging
import time

from .config import VehicleConfig
from .dvl import DvlClient
from .mavlink import MavlinkMonitor
from .models import DvlPosition

LOGGER = logging.getLogger(__name__)


class CompanionMonitorApp:
    """Monitor MAVLink and DVL data without sending vehicle commands."""

    def __init__(self, config: VehicleConfig) -> None:
        if config.safety.allow_actuation:
            raise ValueError("monitor mode requires safety.allow_actuation=false")
        self.config = config
        self.mavlink = MavlinkMonitor(config.mavlink)
        self.dvl = DvlClient(config.dvl) if config.dvl.enabled else None
        self.last_position: DvlPosition | None = None

    def connect(self) -> None:
        LOGGER.info("Connecting to MAVLink at %s", self.config.mavlink.endpoint)
        self.mavlink.connect()
        LOGGER.info("MAVLink heartbeat received; monitor mode sends no commands")

        if self.dvl is not None:
            try:
                self.dvl.connect()
                LOGGER.info("Connected to DVL at %s:%s", self.config.dvl.host, self.config.dvl.port)
            except OSError:
                if self.config.dvl.required:
                    raise
                LOGGER.exception("DVL connection failed; continuing with MAVLink monitoring")
                self.dvl = None

    def run(self) -> None:
        self.connect()
        next_status = 0.0
        try:
            while True:
                telemetry = self.mavlink.poll()
                if self.dvl is not None:
                    for report in self.dvl.poll():
                        position = DvlPosition.from_report(report)
                        if position is not None:
                            self.last_position = position

                now = time.monotonic()
                if now >= next_status:
                    LOGGER.info(
                        "heading_deg=%s altitude_m=%s dvl_position=%s",
                        telemetry.heading_deg,
                        telemetry.altitude_m,
                        self.last_position,
                    )
                    next_status = now + self.config.monitoring.status_interval_s
                time.sleep(self.config.monitoring.poll_interval_s)
        finally:
            self.close()

    def close(self) -> None:
        if self.dvl is not None:
            self.dvl.close()
        self.mavlink.close()
