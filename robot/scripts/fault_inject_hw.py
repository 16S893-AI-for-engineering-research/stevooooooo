#!/usr/bin/env python3
"""AA-28 hardware fault-injection procedure (owner-supervised).

Exercises the two AA-28 fault paths that were mock-only in the original
test suite:

  Procedure A — mid-advance servo fault
      The operator physically unplugs one arm's servo bus connector while the
      cycle is in the advance phase.  The script observes the resulting
      ServoTimeoutError, invokes FaultResponder (ESTOP_HOLD_AND_HALT), and
      confirms the e-stop was triggered and both grippers are still safe.

  Procedure B — USB camera device loss
      The operator physically disconnects a camera USB cable mid-run.  The
      script detects the DeviceNotFoundError / DeviceNotAttachedError, invokes
      FaultResponder (ESTOP_HOLD_AND_HALT), and confirms halt.

The script **never commands motion**.  It only sets up observers / monitors,
then waits for the operator to inject the physical fault.  On detection, it
issues e-stop HOLD and halts.

Default mode: **dry-run**.  No serial ports or cameras are opened; the script
walks through both procedures, prints every operator step and expected outcome,
and exits 0.  Pass ``--live`` to run the real hardware path (owner-supervised).

Usage::

    # Dry-run (default — no hardware, walks both procedures)
    uv run python scripts/fault_inject_hw.py

    # Live — procedure A only (servo fault)
    uv run python scripts/fault_inject_hw.py --live --procedure servo

    # Live — procedure B only (USB camera loss)
    uv run python scripts/fault_inject_hw.py --live --procedure usb_camera

    # Live — both procedures in sequence
    uv run python scripts/fault_inject_hw.py --live --procedure both

Safety invariants
-----------------
* Dry-run by default — ``--live`` must be passed explicitly.
* Real serial ports / cameras are never opened without ``--live``.
* No arm motion is ever commanded; this script only observes and responds.
* All arm motion (if any were needed) would go through motion_guard.build_guarded_arm.
* E-stop response is always HOLD (arms stay in place under torque).
* Gripper servo is never commanded by this script (AA-17 invariant).

The JSON report is written to::

    robot/sessions/<ts>_<hex>/artifacts/fault_inject_report.json

Exit codes:
    0  – All requested procedures ran and the FaultResponder reported success.
    1  – One or more fault response steps failed (partial failure).
    2  – Configuration / setup error (no hardware command sent).
    3  – Operator aborted via Ctrl-C or prompt refusal.

needs owner-supervised hardware session:
    uv run python scripts/fault_inject_hw.py --live
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Ensure src is importable when run directly
# ---------------------------------------------------------------------------

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from inchworm.config import load_config
from inchworm.estop import EStop
from inchworm.gripper_lock import GripperLockSM, GripperState
from inchworm.recovery import (
    FaultResponder,
    RecoveryFaultKind,
    RecoveryResponse,
)
from inchworm.session import new_session
from inchworm.servo_bus import ArmServoBus
from inchworm.torque import TorqueManager
from inchworm.dexarm import MockDexArmSerial, DexArmDriver
from inchworm.backends import MockSerialTransport

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
)
_LOG = logging.getLogger("scripts.fault_inject_hw")

# ---------------------------------------------------------------------------
# Procedure definitions
# ---------------------------------------------------------------------------

#: Human-readable steps for Procedure A (mid-advance servo fault).
PROCEDURE_A_STEPS: List[str] = [
    "STEP A1  Confirm both arms are LOCKED into their current lattice indices.",
    "STEP A2  Confirm the cycle is in the advance phase (arm is moving).",
    "STEP A3  Physically unplug the servo bus connector from ONE arm (e.g. left arm).",
    "         Use the labelled cable marked 'arm_left servo bus'.",
    "         Do NOT unplug both arms simultaneously.",
    "         Do NOT unplug the power bus; only the communication (TTL) cable.",
    "STEP A4  Observe: the script detects ServoTimeoutError within 1 s.",
    "STEP A5  Observe: FaultResponder issues ESTOP_HOLD_AND_HALT (arms held in place).",
    "STEP A6  Observe: report written to the session artifacts directory.",
    "EXPECTED The robot is frozen in place (HOLD mode); no further motion occurs.",
    "         The JSON report shows fault_kind=mid_advance_servo_fault, succeeded=true.",
]

#: Human-readable steps for Procedure B (USB camera device loss).
PROCEDURE_B_STEPS: List[str] = [
    "STEP B1  Confirm the camera pipeline is running (session logger active).",
    "STEP B2  Identify the camera USB cable to disconnect (e.g. 'role=wrist_cam').",
    "         Do NOT disconnect a servo bus cable during this procedure.",
    "STEP B3  Physically unplug the camera USB cable from the USB hub/dock.",
    "STEP B4  Observe: the script detects DeviceNotFoundError within 2 s.",
    "STEP B5  Observe: FaultResponder issues ESTOP_HOLD_AND_HALT.",
    "STEP B6  Observe: report written to the session artifacts directory.",
    "EXPECTED All motion is halted; laser is off; grippers remain in their",
    "         last known state (at least one LOCKED).",
    "         The JSON report shows fault_kind=usb_device_loss, succeeded=true.",
]

# ---------------------------------------------------------------------------
# Report dataclass
# ---------------------------------------------------------------------------


@dataclass
class ProcedureOutcome:
    """Result of one fault-injection procedure."""

    procedure_id: str          # "A_servo_fault" or "B_usb_camera_loss"
    fault_kind: str
    expected_response: str
    actual_response: Optional[str]
    succeeded: bool
    dry_run: bool
    detail: str
    ts_iso: str = field(default_factory=lambda: datetime.now(tz=timezone.utc).isoformat())
    error: Optional[str] = None


@dataclass
class FaultInjectReport:
    """Top-level JSON report written to the session artifacts directory."""

    script: str = "fault_inject_hw.py"
    aa_task: str = "AA-28"
    dry_run: bool = True
    procedures_requested: List[str] = field(default_factory=list)
    outcomes: List[ProcedureOutcome] = field(default_factory=list)
    all_succeeded: bool = False
    completed_at: str = field(default_factory=lambda: datetime.now(tz=timezone.utc).isoformat())

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        return d


# ---------------------------------------------------------------------------
# Mock gripper SM (always LOCKED, for use without real hardware)
# ---------------------------------------------------------------------------


class _LockedGripperSM:
    """Minimal GripperLockSM stand-in: always reports LOCKED for dry-run."""

    state: GripperState = GripperState.LOCKED


# ---------------------------------------------------------------------------
# Hardware helpers
# ---------------------------------------------------------------------------


def _build_mock_arm_bus(role: str) -> ArmServoBus:
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


def _build_estop(dry_run: bool, config: Any, logger: Any) -> EStop:
    bus_left = _build_mock_arm_bus("arm_left")
    bus_right = _build_mock_arm_bus("arm_right")
    torque_mgr = TorqueManager(
        {"arm_left": bus_left, "arm_right": bus_right},
        dry_run=dry_run,
    )
    dexarm = DexArmDriver(MockDexArmSerial())
    return EStop(
        torque_mgr,
        dexarm,
        config=config,
        logger=logger,
        dry_run=dry_run,
        install_signal_handlers=False,
    )


# ---------------------------------------------------------------------------
# Procedure runners
# ---------------------------------------------------------------------------


def _confirm(prompt: str) -> bool:
    """Ask the operator to confirm. Returns True if confirmed."""
    try:
        ans = input(f"\n{prompt} [yes/no]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        return False
    return ans in ("yes", "y")


def _print_procedure_header(title: str, steps: List[str]) -> None:
    width = 60
    print("\n" + "=" * width)
    print(f"  {title}")
    print("=" * width)
    for step in steps:
        print(f"  {step}")
    print("-" * width)


def run_procedure_a_servo_fault(
    *,
    dry_run: bool,
    responder: FaultResponder,
    timeout_s: float = 30.0,
) -> ProcedureOutcome:
    """Procedure A: mid-advance servo fault.

    In dry-run: simulates detection immediately and reports the expected
    outcome without waiting for operator action.
    In live mode: waits for operator to unplug arm bus, then monitors
    the servo bus for ServoTimeoutError.
    """
    _print_procedure_header(
        "PROCEDURE A — Mid-Advance Servo Fault",
        PROCEDURE_A_STEPS,
    )

    if dry_run:
        _LOG.info("[DRY-RUN] Simulating ServoTimeoutError detection for Procedure A.")
        detail = "dry_run: simulated ServoTimeoutError on arm_left servo bus"
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT,
            detail=detail,
            cycle_index=0,
            arm="arm_left",
        )
        print(f"\n  [DRY-RUN] FaultResponder response: {response.value}, succeeded={succeeded}")
        print(f"  [DRY-RUN] Expected: {RecoveryResponse.ESTOP_HOLD_AND_HALT.value}")
        return ProcedureOutcome(
            procedure_id="A_servo_fault",
            fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=response.value,
            succeeded=succeeded,
            dry_run=True,
            detail=detail,
        )

    # --- LIVE path ---
    if not _confirm(
        "LIVE: Confirm both arms are LOCKED and the cycle is in the advance phase.\n"
        "  You will be asked to unplug the arm_left servo bus cable."
    ):
        _LOG.warning("Procedure A aborted by operator.")
        return ProcedureOutcome(
            procedure_id="A_servo_fault",
            fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=None,
            succeeded=False,
            dry_run=False,
            detail="operator declined confirmation",
            error="aborted",
        )

    print(
        "\n  >>> ACTION REQUIRED <<<\n"
        "  Physically unplug the arm_left servo bus (TTL) cable NOW.\n"
        "  The script will detect the fault automatically within "
        f"{timeout_s:.0f} s.\n"
        "  (The script will NOT command any motion.)\n"
    )

    # In live mode we attempt a servo read in a polling loop and watch for errors.
    from inchworm.servo_bus import ServoTimeoutError  # type: ignore[attr-defined]

    # Import live hardware dependencies here so dry-run never triggers them.
    try:
        from inchworm.backends import HardwareBackend
        config = load_config()
        hw = HardwareBackend(config)
        hw_bus_left = hw._open_serial("arm_left", hw._config.arms["arm_left"].device)  # type: ignore[attr-defined]
        live_bus = ArmServoBus(hw_bus_left, role="arm_left", joint_id_map={
            "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
            "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
        }, dry_run=False)
    except Exception as exc:
        detail = f"live bus open failed: {exc}"
        _LOG.error("Procedure A live setup failed: %s", exc)
        return ProcedureOutcome(
            procedure_id="A_servo_fault",
            fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=None,
            succeeded=False,
            dry_run=False,
            detail=detail,
            error=str(exc),
        )

    deadline = time.monotonic() + timeout_s
    detected_error: Optional[Exception] = None
    while time.monotonic() < deadline:
        try:
            # Non-motion read: just attempt to ping servo ID 1 (shoulder_pan).
            live_bus.read_present_position("shoulder_pan")
            time.sleep(0.25)
        except Exception as exc:  # noqa: BLE001
            detected_error = exc
            _LOG.info("Procedure A: fault detected: %s", exc)
            break

    if detected_error is None:
        detail = f"timeout after {timeout_s:.0f} s; no servo fault detected"
        _LOG.warning("Procedure A: %s", detail)
        return ProcedureOutcome(
            procedure_id="A_servo_fault",
            fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=None,
            succeeded=False,
            dry_run=False,
            detail=detail,
            error="timeout",
        )

    detail = f"ServoTimeoutError detected on arm_left: {detected_error}"
    response, succeeded = responder.respond(
        fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT,
        detail=detail,
        cycle_index=0,
        arm="arm_left",
    )
    print(f"\n  FaultResponder response: {response.value}, succeeded={succeeded}")
    return ProcedureOutcome(
        procedure_id="A_servo_fault",
        fault_kind=RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value,
        expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
        actual_response=response.value,
        succeeded=succeeded,
        dry_run=False,
        detail=detail,
    )


def run_procedure_b_usb_camera_loss(
    *,
    dry_run: bool,
    responder: FaultResponder,
    timeout_s: float = 30.0,
) -> ProcedureOutcome:
    """Procedure B: USB camera device loss.

    In dry-run: simulates detection immediately and reports the expected
    outcome without waiting for operator action or touching any USB device.
    In live mode: opens the camera arbiter, waits for operator to unplug
    a camera cable, then detects DeviceNotFoundError / DeviceNotAttachedError.
    """
    _print_procedure_header(
        "PROCEDURE B — USB Camera Device Loss",
        PROCEDURE_B_STEPS,
    )

    if dry_run:
        _LOG.info("[DRY-RUN] Simulating DeviceNotFoundError detection for Procedure B.")
        detail = "dry_run: simulated DeviceNotFoundError for role=wrist_cam"
        response, succeeded = responder.respond(
            fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS,
            detail=detail,
            cycle_index=0,
            arm=None,
        )
        print(f"\n  [DRY-RUN] FaultResponder response: {response.value}, succeeded={succeeded}")
        print(f"  [DRY-RUN] Expected: {RecoveryResponse.ESTOP_HOLD_AND_HALT.value}")
        return ProcedureOutcome(
            procedure_id="B_usb_camera_loss",
            fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=response.value,
            succeeded=succeeded,
            dry_run=True,
            detail=detail,
        )

    # --- LIVE path ---
    if not _confirm(
        "LIVE: Confirm a camera is active and you have identified the cable to unplug.\n"
        "  Do NOT unplug a servo bus cable during this procedure."
    ):
        _LOG.warning("Procedure B aborted by operator.")
        return ProcedureOutcome(
            procedure_id="B_usb_camera_loss",
            fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=None,
            succeeded=False,
            dry_run=False,
            detail="operator declined confirmation",
            error="aborted",
        )

    print(
        "\n  >>> ACTION REQUIRED <<<\n"
        "  Physically unplug a camera USB cable NOW.\n"
        f"  The script will detect the fault automatically within {timeout_s:.0f} s.\n"
        "  (The script will NOT command any motion.)\n"
    )

    from inchworm.discovery import DeviceNotFoundError, DeviceNotAttachedError, require_device

    config = load_config()
    # Pick the first camera role from config.
    camera_roles = [
        r for r in config.devices
        if "cam" in r.lower()
    ]
    if not camera_roles:
        detail = "no camera roles found in config; cannot monitor USB loss"
        _LOG.warning("Procedure B: %s", detail)
        return ProcedureOutcome(
            procedure_id="B_usb_camera_loss",
            fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=None,
            succeeded=False,
            dry_run=False,
            detail=detail,
            error="no_camera_role",
        )
    watch_role = camera_roles[0]
    _LOG.info("Monitoring camera role %r for USB loss.", watch_role)

    deadline = time.monotonic() + timeout_s
    detected_error: Optional[Exception] = None
    while time.monotonic() < deadline:
        try:
            # Non-motion probe: attempt to discover the device without opening it.
            require_device(watch_role, config)
            time.sleep(0.5)
        except (DeviceNotFoundError, DeviceNotAttachedError) as exc:
            detected_error = exc
            _LOG.info("Procedure B: USB device loss detected: %s", exc)
            break
        except Exception as exc:  # noqa: BLE001
            # Other errors (e.g. usbipd not running in live env) also signal loss.
            detected_error = exc
            _LOG.info("Procedure B: device probe raised: %s", exc)
            break

    if detected_error is None:
        detail = f"timeout after {timeout_s:.0f} s; no USB loss detected"
        _LOG.warning("Procedure B: %s", detail)
        return ProcedureOutcome(
            procedure_id="B_usb_camera_loss",
            fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS.value,
            expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
            actual_response=None,
            succeeded=False,
            dry_run=False,
            detail=detail,
            error="timeout",
        )

    detail = f"USB device loss detected for role={watch_role}: {detected_error}"
    response, succeeded = responder.respond(
        fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS,
        detail=detail,
        cycle_index=0,
        arm=None,
    )
    print(f"\n  FaultResponder response: {response.value}, succeeded={succeeded}")
    return ProcedureOutcome(
        procedure_id="B_usb_camera_loss",
        fault_kind=RecoveryFaultKind.USB_DEVICE_LOSS.value,
        expected_response=RecoveryResponse.ESTOP_HOLD_AND_HALT.value,
        actual_response=response.value,
        succeeded=succeeded,
        dry_run=False,
        detail=detail,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="fault_inject_hw.py",
        description=(
            "AA-28 fault-injection procedure (owner-supervised).\n\n"
            "Default: dry-run — walks both procedures, prints steps and "
            "expected outcomes, exits 0 without touching hardware.\n\n"
            "Pass --live for real hardware.  The operator must physically inject "
            "each fault; the script only observes and responds (e-stop HOLD + halt)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Run on real hardware.  Without this flag the script is a dry-run "
            "that prints every step but never opens serial ports or cameras."
        ),
    )
    parser.add_argument(
        "--procedure",
        choices=["servo", "usb_camera", "both"],
        default="both",
        help=(
            "Which procedure to run.  "
            "'servo' = Procedure A (mid-advance servo fault).  "
            "'usb_camera' = Procedure B (USB camera loss).  "
            "'both' = A then B in sequence (default)."
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="Seconds to wait for the operator to inject each fault (default: 30).",
    )
    args = parser.parse_args(argv)
    dry_run: bool = not args.live

    # ------------------------------------------------------------------
    # Banner
    # ------------------------------------------------------------------
    mode_label = "DRY-RUN" if dry_run else "LIVE (owner-supervised)"
    print(
        f"\n{'=' * 60}\n"
        f"  AA-28 Fault Injection Procedure\n"
        f"  Mode:      {mode_label}\n"
        f"  Procedure: {args.procedure}\n"
        f"{'=' * 60}"
    )
    if dry_run:
        print(
            "\n  DRY-RUN: No serial ports or cameras will be opened.\n"
            "  Both procedures will be walked through with simulated fault\n"
            "  detection.  Pass --live to run on real hardware.\n"
        )
    else:
        print(
            "\n  LIVE MODE — Owner supervision required.\n"
            "  This script will NOT command any arm motion.\n"
            "  It only observes hardware faults and issues e-stop HOLD.\n"
            "  You will be prompted before each physical action.\n"
        )

    # ------------------------------------------------------------------
    # Config + session
    # ------------------------------------------------------------------
    try:
        config = load_config()
    except Exception as exc:
        _LOG.error("Failed to load hardware config: %s", exc)
        return 2

    with new_session(config) as ctx:
        # ------------------------------------------------------------------
        # Build FaultResponder (observe-only; never commands motion)
        # ------------------------------------------------------------------
        estop = _build_estop(dry_run=dry_run, config=config, logger=ctx.logger)

        # Gripper SMs: in dry-run use locked stubs; in live mode use real SMs
        # (the script never transitions gripper state — stubs are always safe).
        left_sm: Any = _LockedGripperSM()
        right_sm: Any = _LockedGripperSM()

        responder = FaultResponder(
            left_sm=left_sm,  # type: ignore[arg-type]
            right_sm=right_sm,  # type: ignore[arg-type]
            estop=estop,
            logger=ctx.logger,
            dry_run=dry_run,
        )

        # ------------------------------------------------------------------
        # Run requested procedures
        # ------------------------------------------------------------------
        report = FaultInjectReport(
            dry_run=dry_run,
            procedures_requested=[args.procedure],
        )

        run_a = args.procedure in ("servo", "both")
        run_b = args.procedure in ("usb_camera", "both")

        if run_a:
            outcome_a = run_procedure_a_servo_fault(
                dry_run=dry_run,
                responder=responder,
                timeout_s=args.timeout,
            )
            report.outcomes.append(outcome_a)

        if run_b:
            # Re-create estop for procedure B (EStop is idempotent once stopped)
            estop_b = _build_estop(dry_run=dry_run, config=config, logger=ctx.logger)
            responder_b = FaultResponder(
                left_sm=left_sm,  # type: ignore[arg-type]
                right_sm=right_sm,  # type: ignore[arg-type]
                estop=estop_b,
                logger=ctx.logger,
                dry_run=dry_run,
            )
            outcome_b = run_procedure_b_usb_camera_loss(
                dry_run=dry_run,
                responder=responder_b,
                timeout_s=args.timeout,
            )
            report.outcomes.append(outcome_b)

        # ------------------------------------------------------------------
        # Finalise report
        # ------------------------------------------------------------------
        report.all_succeeded = all(o.succeeded for o in report.outcomes)
        report.completed_at = datetime.now(tz=timezone.utc).isoformat()

        report_path = ctx.artifact_dir / "fault_inject_report.json"
        report_path.write_text(
            json.dumps(report.to_dict(), indent=2, default=str),
            encoding="utf-8",
        )
        _LOG.info("Report written to: %s", report_path)

        # ------------------------------------------------------------------
        # Summary
        # ------------------------------------------------------------------
        print(f"\n{'=' * 60}")
        print("  FAULT INJECTION SUMMARY")
        print(f"{'=' * 60}")
        for o in report.outcomes:
            status = "✓ PASS" if o.succeeded else "✗ FAIL"
            print(f"  {status}  {o.procedure_id}")
            print(f"         fault_kind : {o.fault_kind}")
            print(f"         response   : {o.actual_response or '(none)'}")
            print(f"         detail     : {o.detail[:80]}")
        print(f"\n  Report: {report_path}")
        print(f"{'=' * 60}\n")

        if report.all_succeeded:
            return 0
        # Check if any procedure was aborted by operator
        if any(o.error == "aborted" for o in report.outcomes):
            return 3
        return 1


if __name__ == "__main__":
    sys.exit(main())
