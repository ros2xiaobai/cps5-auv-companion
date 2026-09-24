from pathlib import Path

from cps5_companion.control.mission import load_mission

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_example_mission_is_valid() -> None:
    segments = load_mission(PROJECT_ROOT / "config" / "mission.example.yaml")
    assert len(segments) == 1
    assert segments[0].distance_m == 0.5
    assert segments[0].tasks == ()
