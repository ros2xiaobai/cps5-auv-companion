from dataclasses import replace
from pathlib import Path

import pytest

from cps5_companion.app import CompanionMonitorApp
from cps5_companion.config import SafetyConfig, load_vehicle_config

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_monitor_rejects_actuation_enabled_configuration() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    unsafe = replace(config, safety=SafetyConfig(allow_actuation=True))
    with pytest.raises(ValueError, match="allow_actuation=false"):
        CompanionMonitorApp(unsafe)
