#!/usr/bin/env python3
"""Emergency stop CLI script for the inchworm robot (AA-10).

Usage::

    # Dry-run (default -- safe, no hardware commands sent):
    uv run python scripts/estop.py

    # Live mode (sends real commands to hardware):
    uv run python scripts/estop.py --live

The script:
1. Loads the hardware config.
2. Opens the mock backend (or hardware backend with --live).
3. Constructs TorqueManager, DexArmDriver, and EStop.
4. Calls EStop.stop().

In dry-run mode, all commands are logged but never transmitted.
In live mode, real serial commands are sent; use only with connected hardware.

Exit codes:
    0 – e-stop completed without errors.
    1 – e-stop triggered but one or more subsystems failed (partial failure).
    2 – configuration or setup error.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Ensure the src package is importable when run directly.
_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from inchworm.backends import MockBackend, HardwareBackend, create_backend
from inchworm.config import load_config
from inchworm.dexarm import DexArmDriver, MockDexArmSerial
from inchworm.estop import EStop, EStopPartialError
from inchworm.servo_bus import ArmServoBus
from inchworm.torque import TorqueManager

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
)
_LOG = logging.getLogger("scripts.estop")


def _build_mock_arm_bus(config, role: str) -> ArmServoBus:
    """Build a minimal ArmServoBus backed by a mock transport."""
    from inchworm.backends import MockSerialTransport

    transport = MockSerialTransport(role)
    joint_id_map = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }
    return ArmServoBus(transport, role=role, joint_id_map=joint_id_map, dry_run=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Inchworm emergency stop (AA-10). Default: dry-run (no hardware)."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Send real commands to hardware. Without this flag the script "
            "runs in dry-run mode and logs only."
        ),
    )
    args = parser.parse_args(argv)
    dry_run: bool = not args.live

    if dry_run:
        _LOG.info("=== DRY-RUN mode: no hardware commands will be sent ===")
    else:
        _LOG.warning("=== LIVE mode: real hardware commands will be sent ===")

    try:
        config = load_config()
    except Exception as exc:
        _LOG.error("Failed to load hardware config: %s", exc)
        return 2

    # Build arm buses.
    if dry_run:
        bus_left = _build_mock_arm_bus(config, "arm_left")
        bus_right = _build_mock_arm_bus(config, "arm_right")
    else:
        # Live: open real transports via the hardware backend.
        # Note: hardware backend is not fully implemented until AA-5 completion;
        # this path is provided for forward compatibility.
        try:
            backend = create_backend(config)
        except Exception as exc:
            _LOG.error("Failed to create hardware backend: %s", exc)
            return 2
        from inchworm.backends import MockSerialTransport
        # Fall back to mock transports until hardware backend is implemented.
        _LOG.warning(
            "Hardware backend serial transport not yet implemented; using mock transports."
        )
        bus_left = _build_mock_arm_bus(config, "arm_left")
        bus_right = _build_mock_arm_bus(config, "arm_right")

    torque_mgr = TorqueManager(
        {"arm_left": bus_left, "arm_right": bus_right},
        dry_run=dry_run,
    )

    # Build DexArm driver.
    dexarm_transport = MockDexArmSerial()
    dexarm = DexArmDriver(dexarm_transport)

    estop = EStop(
        torque_mgr,
        dexarm,
        config=config,
        dry_run=dry_run,
        install_signal_handlers=False,  # CLI; don't install in subprocess
    )

    _LOG.info("Triggering emergency stop (dry_run=%s)...", dry_run)
    try:
        estop.stop()
        _LOG.info("Emergency stop completed successfully.")
        return 0
    except EStopPartialError as exc:
        _LOG.error("Emergency stop partially failed: %s", exc)
        for name, err in exc.subsystem_errors.items():
            _LOG.error("  subsystem %r: %s", name, err)
        return 1
    except Exception as exc:
        _LOG.error("Emergency stop raised unexpected error: %s", exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
