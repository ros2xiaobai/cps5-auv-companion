from pathlib import Path

import pytest

from cps5_companion.config import ConfigError, load_vehicle_config

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_example_vehicle_config_is_valid() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    assert config.mavlink.endpoint == "udpin:0.0.0.0:14553"
    assert config.dvl.port == 16171
    assert config.safety.allow_actuation is False


def test_invalid_rc_neutral_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "invalid.yaml"
    path.write_text(
        "mavlink:\n  endpoint: test\ndvl:\n  enabled: false\n  port: 16171\n"
        "control:\n  rc_neutral: 999\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="rc_neutral"):
        load_vehicle_config(path)
