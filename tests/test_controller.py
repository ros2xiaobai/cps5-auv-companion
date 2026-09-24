from pathlib import Path

import pytest

from cps5_companion.config import load_vehicle_config
from cps5_companion.control.controller import MissionController, SensorSnapshot
from cps5_companion.control.mission import MissionSegment, SegmentState

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _config():
    return load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml").control


def _snapshot(
    now: float,
    *,
    heading: float = 0.0,
    depth: float = 0.2,
    position: float = 10.0,
) -> SensorSnapshot:
    return SensorSnapshot(
        now=now,
        heading_deg=heading,
        heading_received_at=now,
        depth_m=depth,
        depth_received_at=now,
        position_x_m=position,
        position_received_at=now,
    )


def test_nominal_segment_reaches_complete_without_dvl_reset() -> None:
    controller = MissionController([MissionSegment(0.0, 0.5, 0.2)], _config())

    first = controller.step(_snapshot(0.0, depth=0.0))
    assert first.state == SegmentState.ADJUST_DEPTH
    assert first.throttle_pwm > 1500

    controller.step(_snapshot(0.1))
    assert controller.state == SegmentState.ADJUST_HEADING
    controller.step(_snapshot(0.2))
    assert controller.state == SegmentState.CAPTURE_DVL_ORIGIN
    controller.step(_snapshot(0.3, position=10.0))
    assert controller.state == SegmentState.MOVE_FORWARD

    complete = controller.step(_snapshot(0.4, position=10.5))
    assert complete.complete is True
    assert complete.state == SegmentState.COMPLETE
    assert complete.throttle_pwm == 1500
    assert complete.forward_pwm == 1500


def test_stale_depth_fails_to_neutral_output() -> None:
    controller = MissionController([MissionSegment(0.0, 0.5, 0.2)], _config())
    stale = SensorSnapshot(
        now=2.0,
        heading_deg=0.0,
        heading_received_at=2.0,
        depth_m=0.2,
        depth_received_at=0.0,
        position_x_m=0.0,
        position_received_at=2.0,
    )
    output = controller.step(stale)
    assert output.state == SegmentState.FAILED
    assert output.fault == "depth telemetry is missing or stale"
    assert output.throttle_pwm == 1500
    assert output.forward_pwm == 1500


def test_heading_timeout_fails_instead_of_advancing() -> None:
    controller = MissionController([MissionSegment(0.0, 0.5, 0.2)], _config())
    controller.step(_snapshot(0.0))
    controller.step(_snapshot(0.1, heading=90.0))
    output = controller.step(_snapshot(8.2, heading=90.0))
    assert output.state == SegmentState.FAILED
    assert output.fault == "heading adjustment timed out"


def test_legacy_automatic_tasks_are_rejected() -> None:
    with pytest.raises(ValueError, match="task actions"):
        MissionController(
            [MissionSegment(0.0, 0.5, 0.2, tasks=("roll",))],
            _config(),
        )
