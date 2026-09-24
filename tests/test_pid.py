from cps5_companion.control.pid import PIDController


def test_pid_first_sample_has_no_derivative_spike() -> None:
    pid = PIDController(1.0, 0.0, 10.0, -100.0, 100.0)
    assert pid.compute(1.0, 0.0, now=10.0) == 1.0


def test_pid_output_is_limited() -> None:
    pid = PIDController(100.0, 0.0, 0.0, -20.0, 20.0)
    assert pid.compute(1.0, 0.0, now=1.0) == 20.0
    assert pid.compute(0.0, 1.0, now=2.0) == -20.0
