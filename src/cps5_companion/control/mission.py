"""Mission data structures separated from hardware control."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml

from ..config import ConfigError


class SegmentState(str, Enum):
    ADJUST_DEPTH = "adjust_depth"
    ADJUST_HEADING = "adjust_heading"
    CAPTURE_DVL_ORIGIN = "capture_dvl_origin"
    MOVE_FORWARD = "move_forward"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True)
class MissionSegment:
    heading_deg: float
    distance_m: float
    depth_m: float
    tasks: tuple[str, ...] = ()


def load_mission(path: str | Path) -> list[MissionSegment]:
    mission_path = Path(path)
    if not mission_path.is_file():
        raise ConfigError(f"mission file not found: {mission_path}")
    try:
        loaded = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid mission YAML: {exc}") from exc
    if not isinstance(loaded, Mapping):
        raise ConfigError("mission root must be a mapping")

    raw_segments = loaded.get("segments")
    if not isinstance(raw_segments, list) or not raw_segments:
        raise ConfigError("mission must contain at least one segment")

    segments: list[MissionSegment] = []
    for index, raw in enumerate(raw_segments):
        if not isinstance(raw, Mapping):
            raise ConfigError(f"mission segment {index} must be a mapping")
        try:
            tasks: Any = raw.get("tasks", [])
            if not isinstance(tasks, list) or not all(isinstance(task, str) for task in tasks):
                raise ConfigError(f"mission segment {index} tasks must be a string list")
            segment = MissionSegment(
                heading_deg=float(raw["heading_deg"]),
                distance_m=float(raw["distance_m"]),
                depth_m=float(raw["depth_m"]),
                tasks=tuple(tasks),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ConfigError(f"invalid mission segment {index}") from exc
        if segment.distance_m < 0 or segment.depth_m < 0:
            raise ConfigError(f"mission segment {index} distance/depth cannot be negative")
        segments.append(segment)

    return segments
