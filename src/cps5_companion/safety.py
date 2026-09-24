"""Independent safety gate for commands that can move the vehicle."""

from __future__ import annotations

from dataclasses import dataclass

from .config import VehicleConfig


class SafetyError(RuntimeError):
    """Raised when an actuation prerequisite has not been confirmed."""


@dataclass(frozen=True)
class OperatorApproval:
    propellers_clear: bool
    controlled_environment: bool
    emergency_stop_ready: bool


def validate_control_execution(
    config: VehicleConfig,
    approval: OperatorApproval,
) -> None:
    missing: list[str] = []
    if not config.safety.allow_actuation:
        missing.append("safety.allow_actuation")
    if not config.safety.rc_mapping_validated:
        missing.append("safety.rc_mapping_validated")
    if not config.safety.depth_mapping_validated:
        missing.append("safety.depth_mapping_validated")
    if not config.safety.dvl_axis_validated:
        missing.append("safety.dvl_axis_validated")
    if not config.dvl.enabled:
        missing.append("dvl.enabled")
    if not approval.propellers_clear:
        missing.append("--confirm-propellers-clear")
    if not approval.controlled_environment:
        missing.append("--confirm-controlled-environment")
    if not approval.emergency_stop_ready:
        missing.append("--confirm-emergency-stop-ready")
    if missing:
        raise SafetyError("actuation blocked; missing confirmation: " + ", ".join(missing))
