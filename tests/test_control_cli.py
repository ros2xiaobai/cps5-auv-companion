from pathlib import Path

from cps5_companion.control_cli import main

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_default_control_command_is_offline_dry_run(capsys) -> None:
    result = main(
        [
            "--config",
            str(PROJECT_ROOT / "config" / "vehicle.example.yaml"),
            "--mission",
            str(PROJECT_ROOT / "config" / "mission.example.yaml"),
        ]
    )
    assert result == 0
    output = capsys.readouterr().out
    assert "Dry-run passed" in output
    assert "No network connection" in output


def test_example_configuration_blocks_execute_before_network_access() -> None:
    result = main(
        [
            "--config",
            str(PROJECT_ROOT / "config" / "vehicle.example.yaml"),
            "--mission",
            str(PROJECT_ROOT / "config" / "mission.example.yaml"),
            "--execute",
            "--confirm-propellers-clear",
            "--confirm-controlled-environment",
            "--confirm-emergency-stop-ready",
        ]
    )
    assert result == 3
