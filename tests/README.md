# Tests

The current hardware-independent tests cover configuration validation, DVL stream framing and JSON parsing, PID edge cases, mission-file loading, state transitions, stale-sensor/timeout behavior, the actuation gate, and the default offline control dry-run.

Run them after installing the development dependencies:

```bash
python -m pip install -e ".[dev]"
python -m pytest
```
