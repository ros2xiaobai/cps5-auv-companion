"""Dry-run-first command-line entry point for mission control."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from .config import ConfigError, load_vehicle_config
from .control.controller import MissionController
from .control.mission import load_mission
from .control_app import CompanionControlApp, ControlRuntimeError
from .safety import OperatorApproval, SafetyError, validate_control_execution

CONFIRMATION_PHRASE = "ARM CPS5"
LOGGER = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CPS5 mission controller; dry-run unless --execute is supplied",
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="allow the gated hardware-control path; omitted means dry-run",
    )
    parser.add_argument("--confirm-propellers-clear", action="store_true")
    parser.add_argument("--confirm-controlled-environment", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
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
        segments = load_mission(args.mission)
        # Construction validates that no unsupported automatic task action is present.
        MissionController(segments, config.control)
    except (ConfigError, ValueError) as exc:
        LOGGER.error("Configuration error: %s", exc)
        return 2

    if not args.execute:
        print(f"Dry-run passed: {len(segments)} mission segment(s).")
        print("No network connection or vehicle command was attempted.")
        print("Hardware execution remains blocked unless --execute and all safety checks are used.")
        return 0

    approval = OperatorApproval(
        propellers_clear=args.confirm_propellers_clear,
        controlled_environment=args.confirm_controlled_environment,
        emergency_stop_ready=args.confirm_emergency_stop_ready,
    )
    try:
        validate_control_execution(config, approval)
    except SafetyError as exc:
        LOGGER.error("%s", exc)
        return 3

    if not sys.stdin.isatty():
        LOGGER.error("hardware execution requires an interactive terminal")
        return 3
    typed = input(f"Type {CONFIRMATION_PHRASE!r} to permit arming: ").strip()
    if typed != CONFIRMATION_PHRASE:
        LOGGER.error("confirmation phrase did not match; actuation cancelled")
        return 3

    try:
        CompanionControlApp(config, segments).run()
    except KeyboardInterrupt:
        LOGGER.warning("Control interrupted; neutral/disarm cleanup requested")
        return 130
    except (OSError, TimeoutError, ControlRuntimeError, RuntimeError) as exc:
        LOGGER.error("Control stopped: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
