"""Validated configuration loading for the companion application."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Raised when a configuration file is missing or invalid."""


@dataclass(frozen=True)
class MavlinkConfig:
    endpoint: str
    heartbeat_timeout_s: float = 10.0


@dataclass(frozen=True)
class DvlConfig:
    enabled: bool
    required: bool
    host: str
    port: int
    connect_timeout_s: float = 3.0
    receive_timeout_s: float = 0.1
    forward_sign: float = 1.0


@dataclass(frozen=True)
class MonitoringConfig:
    poll_interval_s: float = 0.05
    status_interval_s: float = 1.0


@dataclass(frozen=True)
class ControlConfig:
    mode: str = "STABILIZE"
    throttle_channel: int = 3
    yaw_channel: int = 4
    forward_channel: int = 5
    rc_min: int = 1100
    rc_neutral: int = 1500
    rc_max: int = 1900
    heading_threshold_deg: float = 3.0
    depth_threshold_m: float = 0.1
    distance_threshold_m: float = 0.1
    heading_timeout_s: float = 8.0
    move_timeout_s: float = 20.0
    sensor_timeout_s: float = 1.0
    startup_timeout_s: float = 10.0
    mode_timeout_s: float = 5.0
    arm_timeout_s: float = 10.0
    depth_kp: float = 500.0
    depth_ki: float = 5.0
    depth_kd: float = 0.0
    depth_max_pwm_delta: int = 150
    forward_kp: float = 230.0
    forward_max_pwm_delta: int = 60


@dataclass(frozen=True)
class DepthConfig:
    scale: float = -1.0
    offset_m: float = 0.0


@dataclass(frozen=True)
class CameraConfig:
    index: int = 0
    output_dir: str = "photos"


@dataclass(frozen=True)
class SafetyConfig:
    allow_actuation: bool = False
    rc_mapping_validated: bool = False
    depth_mapping_validated: bool = False
    dvl_axis_validated: bool = False


@dataclass(frozen=True)
class VehicleConfig:
    mavlink: MavlinkConfig
    dvl: DvlConfig
    monitoring: MonitoringConfig
    control: ControlConfig
    depth: DepthConfig
    camera: CameraConfig
    safety: SafetyConfig


def _section(data: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = data.get(name, {})
    if not isinstance(value, Mapping):
        raise ConfigError(f"'{name}' must be a mapping")
    return value


def _positive(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"'{name}' must be a number") from exc
    if number <= 0:
        raise ConfigError(f"'{name}' must be greater than zero")
    return number


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"'{name}' must be true or false")
    return value


def _channel(value: Any, name: str) -> int:
    try:
        channel = int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"'{name}' must be an integer") from exc
    if not 1 <= channel <= 8:
        raise ConfigError(f"'{name}' must be between 1 and 8")
    return channel


def load_vehicle_config(path: str | Path) -> VehicleConfig:
    """Load and validate a vehicle YAML configuration file."""

    config_path = Path(path)
    if not config_path.is_file():
        raise ConfigError(f"configuration file not found: {config_path}")
    try:
        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {config_path}: {exc}") from exc
    if not isinstance(loaded, Mapping):
        raise ConfigError("configuration root must be a mapping")

    mavlink = _section(loaded, "mavlink")
    endpoint = str(mavlink.get("endpoint", "")).strip()
    if not endpoint:
        raise ConfigError("'mavlink.endpoint' is required")

    dvl = _section(loaded, "dvl")
    dvl_enabled = _boolean(dvl.get("enabled", True), "dvl.enabled")
    dvl_required = _boolean(dvl.get("required", False), "dvl.required")
    dvl_host = str(dvl.get("host", "")).strip()
    if dvl_enabled and not dvl_host:
        raise ConfigError("'dvl.host' is required when DVL is enabled")
    try:
        dvl_port = int(dvl.get("port", 16171))
    except (TypeError, ValueError) as exc:
        raise ConfigError("'dvl.port' must be an integer") from exc
    if not 1 <= dvl_port <= 65535:
        raise ConfigError("'dvl.port' must be between 1 and 65535")
    forward_sign = float(dvl.get("forward_sign", 1.0))
    if forward_sign == 0:
        raise ConfigError("'dvl.forward_sign' cannot be zero")

    monitoring = _section(loaded, "monitoring")
    control = _section(loaded, "control")
    depth = _section(loaded, "depth")
    camera = _section(loaded, "camera")
    safety = _section(loaded, "safety")

    rc_min = int(control.get("rc_min", 1100))
    rc_neutral = int(control.get("rc_neutral", 1500))
    rc_max = int(control.get("rc_max", 1900))
    if not rc_min < rc_neutral < rc_max:
        raise ConfigError("'control.rc_neutral' must lie between rc_min and rc_max")
    channels = (
        _channel(control.get("throttle_channel", 3), "control.throttle_channel"),
        _channel(control.get("yaw_channel", 4), "control.yaw_channel"),
        _channel(control.get("forward_channel", 5), "control.forward_channel"),
    )
    if len(set(channels)) != len(channels):
        raise ConfigError("control channel assignments must be unique")

    depth_max_delta = int(control.get("depth_max_pwm_delta", 150))
    forward_max_delta = int(control.get("forward_max_pwm_delta", 60))
    available_delta = min(rc_neutral - rc_min, rc_max - rc_neutral)
    if not 0 < depth_max_delta <= available_delta:
        raise ConfigError("'control.depth_max_pwm_delta' exceeds the RC range")
    if not 0 < forward_max_delta <= available_delta:
        raise ConfigError("'control.forward_max_pwm_delta' exceeds the RC range")

    return VehicleConfig(
        mavlink=MavlinkConfig(
            endpoint=endpoint,
            heartbeat_timeout_s=_positive(
                mavlink.get("heartbeat_timeout_s", 10.0), "mavlink.heartbeat_timeout_s"
            ),
        ),
        dvl=DvlConfig(
            enabled=dvl_enabled,
            required=dvl_required,
            host=dvl_host,
            port=dvl_port,
            connect_timeout_s=_positive(
                dvl.get("connect_timeout_s", 3.0), "dvl.connect_timeout_s"
            ),
            receive_timeout_s=_positive(
                dvl.get("receive_timeout_s", 0.1), "dvl.receive_timeout_s"
            ),
            forward_sign=forward_sign,
        ),
        monitoring=MonitoringConfig(
            poll_interval_s=_positive(
                monitoring.get("poll_interval_s", 0.05), "monitoring.poll_interval_s"
            ),
            status_interval_s=_positive(
                monitoring.get("status_interval_s", 1.0), "monitoring.status_interval_s"
            ),
        ),
        control=ControlConfig(
            mode=str(control.get("mode", "STABILIZE")).strip().upper(),
            throttle_channel=channels[0],
            yaw_channel=channels[1],
            forward_channel=channels[2],
            rc_min=rc_min,
            rc_neutral=rc_neutral,
            rc_max=rc_max,
            heading_threshold_deg=_positive(
                control.get("heading_threshold_deg", 3.0), "control.heading_threshold_deg"
            ),
            depth_threshold_m=_positive(
                control.get("depth_threshold_m", 0.1), "control.depth_threshold_m"
            ),
            distance_threshold_m=_positive(
                control.get("distance_threshold_m", 0.1), "control.distance_threshold_m"
            ),
            heading_timeout_s=_positive(
                control.get("heading_timeout_s", 8.0), "control.heading_timeout_s"
            ),
            move_timeout_s=_positive(
                control.get("move_timeout_s", 20.0), "control.move_timeout_s"
            ),
            sensor_timeout_s=_positive(
                control.get("sensor_timeout_s", 1.0), "control.sensor_timeout_s"
            ),
            startup_timeout_s=_positive(
                control.get("startup_timeout_s", 10.0), "control.startup_timeout_s"
            ),
            mode_timeout_s=_positive(
                control.get("mode_timeout_s", 5.0), "control.mode_timeout_s"
            ),
            arm_timeout_s=_positive(
                control.get("arm_timeout_s", 10.0), "control.arm_timeout_s"
            ),
            depth_kp=float(control.get("depth_kp", 500.0)),
            depth_ki=float(control.get("depth_ki", 5.0)),
            depth_kd=float(control.get("depth_kd", 0.0)),
            depth_max_pwm_delta=depth_max_delta,
            forward_kp=float(control.get("forward_kp", 230.0)),
            forward_max_pwm_delta=forward_max_delta,
        ),
        depth=DepthConfig(
            scale=float(depth.get("scale", -1.0)),
            offset_m=float(depth.get("offset_m", 0.0)),
        ),
        camera=CameraConfig(
            index=int(camera.get("index", 0)),
            output_dir=str(camera.get("output_dir", "photos")),
        ),
        safety=SafetyConfig(
            allow_actuation=_boolean(
                safety.get("allow_actuation", False), "safety.allow_actuation"
            ),
            rc_mapping_validated=_boolean(
                safety.get("rc_mapping_validated", False),
                "safety.rc_mapping_validated",
            ),
            depth_mapping_validated=_boolean(
                safety.get("depth_mapping_validated", False),
                "safety.depth_mapping_validated",
            ),
            dvl_axis_validated=_boolean(
                safety.get("dvl_axis_validated", False), "safety.dvl_axis_validated"
            ),
        ),
    )
