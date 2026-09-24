# CPS5 AUV Companion

A companion-computer project for a small hybrid autonomous/remotely operated underwater vehicle based on the CPS5ROV platform.

The current repository focuses on the Raspberry Pi Python application, MAVLink communication with Pixhawk/ArduSub, DVL data acquisition over TCP, and deployment/integration notes.

The new command-line entry point is read-only by default: it receives MAVLink telemetry and DVL reports but does not send mode changes, arming, RC override, or attitude-target commands.

Validate the example files without opening any network connection:

```bash
python -m pip install -e ".[dev]"
cps5-monitor --config config/vehicle.example.yaml \
  --mission config/mission.example.yaml --check-config
```

The retained `dvltest1.py` is an engineering prototype that still contains vehicle-control and automatic-arming behavior. Do not use it as the safe public entry point.

Mission-control validation is also offline by default:

```bash
cps5-control --config config/vehicle.example.yaml \
  --mission config/mission.example.yaml
```

The example configuration blocks hardware execution. The gated execution path additionally requires validated RC/depth/DVL mappings, three operator flags, an interactive terminal, fresh pre-arm sensor data, and an exact final confirmation phrase. See `docs/control-safety.md`.
