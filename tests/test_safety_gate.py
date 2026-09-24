from dataclasses import replace
from pathlib import Path

import pytest

from cps5_companion.config import SafetyConfig, load_vehicle_config
from cps5_companion.safety import (
    OperatorApproval,
    SafetyError,
    validate_control_execution,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FULL_APPROVAL = OperatorApproval(True, True, True)


def test_example_configuration_blocks_actuation() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    with pytest.raises(SafetyError, match="allow_actuation"):
        validate_control_execution(config, FULL_APPROVAL)


def test_all_configuration_and_operator_checks_are_required() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    validated = replace(
        config,
        safety=SafetyConfig(
            allow_actuation=True,
            rc_mapping_validated=True,
            depth_mapping_validated=True,
            dvl_axis_validated=True,
        ),
    )
    validate_control_execution(validated, FULL_APPROVAL)
    with pytest.raises(SafetyError, match="emergency-stop"):
        validate_control_execution(validated, OperatorApproval(True, True, False))
