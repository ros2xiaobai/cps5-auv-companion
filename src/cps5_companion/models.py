"""Data models shared by communication and monitoring modules."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass
class VehicleTelemetry:
    heartbeat_seen: bool = False
    heading_deg: float | None = None
    altitude_m: float | None = None
    heading_received_at: float | None = None
    altitude_received_at: float | None = None


@dataclass(frozen=True)
class DvlPosition:
    x_m: float
    y_m: float

    @classmethod
    def from_report(cls, report: Mapping[str, Any]) -> DvlPosition | None:
        if report.get("type") != "position_local":
            return None
        try:
            return cls(x_m=float(report["x"]), y_m=float(report["y"]))
        except (KeyError, TypeError, ValueError):
            return None
