from pathlib import Path

from cps5_companion.actuation import MavlinkActuator
from cps5_companion.config import load_vehicle_config
from cps5_companion.control.controller import ControlSetpoint
from cps5_companion.control.mission import SegmentState

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeMav:
    def __init__(self) -> None:
        self.rc_calls: list[tuple[int, ...]] = []

    def rc_channels_override_send(self, *args: int) -> None:
        self.rc_calls.append(args)


class FakeConnection:
    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav()
        self.selected_mode: int | None = None

    class Heartbeat:
        custom_mode = 0

    def mode_mapping(self) -> dict[str, int]:
        return {"STABILIZE": 0}

    def set_mode(self, mode_id: int) -> None:
        self.selected_mode = mode_id

    def wait_heartbeat(self, timeout: float):
        del timeout
        return self.Heartbeat()


class FakeMonitor:
    def __init__(self) -> None:
        self.connection = FakeConnection()


def test_rc_mapping_uses_one_based_configuration_channels() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    monitor = FakeMonitor()
    actuator = MavlinkActuator(monitor, config.control)  # type: ignore[arg-type]
    actuator.send_setpoint(
        ControlSetpoint(
            state=SegmentState.MOVE_FORWARD,
            segment_index=0,
            throttle_pwm=1510,
            forward_pwm=1560,
            target_yaw_deg=None,
        )
    )
    call = monitor.connection.mav.rc_calls[-1]
    assert call[:2] == (1, 1)
    channels = call[2:]
    assert channels[2] == 1510
    assert channels[3] == 1500
    assert channels[4] == 1560
    assert all(value == 65535 for value in channels[:2] + channels[5:])


def test_configured_mode_is_resolved_before_selection() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    monitor = FakeMonitor()
    actuator = MavlinkActuator(monitor, config.control)  # type: ignore[arg-type]
    actuator.set_mode()
    assert monitor.connection.selected_mode == 0


def test_release_override_releases_all_channels() -> None:
    config = load_vehicle_config(PROJECT_ROOT / "config" / "vehicle.example.yaml")
    monitor = FakeMonitor()
    actuator = MavlinkActuator(monitor, config.control)  # type: ignore[arg-type]
    actuator.release_override()
    call = monitor.connection.mav.rc_calls[-1]
    assert call[:2] == (1, 1)
    assert call[2:] == (65535,) * 8
