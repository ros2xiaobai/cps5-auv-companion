"""Command-line entry point for the safe monitor application."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from .app import CompanionMonitorApp
from .config import ConfigError, load_vehicle_config
from .control.mission import load_mission

LOGGER = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only CPS5 companion-computer monitor",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/vehicle.example.yaml"),
        help="vehicle YAML file (default: config/vehicle.example.yaml)",
    )
    parser.add_argument(
        "--mission",
        type=Path,
        default=None,
        help="optional mission YAML to validate; it is not executed in monitor mode",
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate configuration and exit without opening network connections",
    )
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        config = load_vehicle_config(args.config)
        mission = load_mission(args.mission) if args.mission is not None else None
    except ConfigError as exc:
        LOGGER.error("Configuration error: %s", exc)
        return 2

    if args.check_config:
        print("Configuration is valid.")
        if mission is not None:
            print(f"Mission is valid: {len(mission)} segment(s).")
        print("No network connection or vehicle command was attempted.")
        return 0

    app = CompanionMonitorApp(config)
    try:
        app.run()
    except KeyboardInterrupt:
        LOGGER.info("Monitor stopped by user")
        return 0
    except (OSError, TimeoutError) as exc:
        LOGGER.error("Connection failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
