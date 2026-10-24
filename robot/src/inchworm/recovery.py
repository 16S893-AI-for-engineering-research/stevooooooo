"""AA-28: Crawl-cycle fault recovery and tube-state verification.

This module implements:

1. **TubeStateVerifier** – checks, before and after each cycle, that each arm
   holding a lattice index has its AA-17 state machine in LOCKED, the gripper
   servo position in the locked band, and the arm FK TCP within tolerance of
   the AA-15 attachment-point pose in the chassis frame.

2. **Fault catalogue** – defines :class:`RecoveryFaultKind` and the automatic
   responses for every enumerated fault (AC#2).

3. **Safety invariant guard** – :func:`assert_not_both_unlocked` raises
   immediately if both grippers would be unlocked (AC#3 enforcement).

4. **CycleStateCheckpoint** – atomic JSON persistence of the FrameModel dump
   + cycle phase + gripper states after every phase; resume validates freshly
   verified state and either continues or refuses with a reason (AC#4).

5. **PostRunReport** – builds a structured text + dict report from the AA-7
   event log (AC#6).

Fault catalogue summary (AC#2)
-------------------------------
+---------------------------------------+-------------------------------+
| Fault                                 | Response                      |
+---------------------------------------+-------------------------------+
| Insertion failure after retries       | Safe halt (holder stays       |
|                                       | LOCKED)                       |
+---------------------------------------+-------------------------------+
| Lock verification failure             | Safe halt                     |
+---------------------------------------+-------------------------------+
| Mid-advance servo fault               | E-stop HOLD (AA-10 default) + |
| (ServoTimeoutError during             | halt                          |
| approach/retract)                     |                               |
+---------------------------------------+-------------------------------+
| Unexpected tube displacement          | Halt + require operator       |
| (verification mismatch)               | re-calibration                |
+---------------------------------------+-------------------------------+
| Loss of USB device                    | E-stop HOLD + halt            |
| (AA-5 DeviceNotFoundError /           |                               |
| DeviceNotAttachedError mid-run)       |                               |
+---------------------------------------+-------------------------------+

Safety invariants (non-negotiable, AGENTS.md)
---------------------------------------------
* :func:`assert_not_both_unlocked` is called at every state-transition that
  could lead to a double-unlock; it raises :class:`BothGrippersUnlockedError`
  before any command is issued.
* Recovery *never* commands motion from an unverified state; every function
  that would issue motion first calls :func:`assert_state_verified`.

Live mode vs dry-run
--------------------
When *any* threshold in :class:`VerificationThresholds` is ``None`` the
verifier returns :class:`VerificationResult` with ``passed=False`` and
``refused=True``.  Live mode must see ``passed=True`` before issuing motion
(enforced by :func:`assert_state_verified`).  Dry-run may call with
``require_pass=False`` to get a diagnostic result without raising.

Persist / resume (AC#4)
-----------------------
:class:`CycleStateCheckpoint` writes one JSON file per phase transition.  The
write is atomic (write to a tmp file, fsync, rename).  On resume, the verifier
re-checks gripper states and FK positions; if they differ from the persisted
state the resume is refused with a clear human-readable reason.  Stale or
corrupt state files are always refused.
"""

from __future__ import annotations

import enum
import json
import logging
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from inchworm.frames import FrameModel, attachment_point_in_chassis_frame
from inchworm.gripper_lock import GripperLockSM, GripperState, _bands_configured

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Module-version stamp (increment if checkpoint schema changes)
# ---------------------------------------------------------------------------

_CHECKPOINT_SCHEMA_VERSION: int = 1

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class RecoveryError(RuntimeError):
    """Base class for all AA-28 recovery errors."""


class BothGrippersUnlockedError(RecoveryError):
    """Raised when an operation would leave both grippers unlocked.

    This is a hard stop: the calling layer must NOT proceed with any motion.
    The tube must have at least one locked grip at all times (AGENTS.md).
    """

    def __init__(self, detail: str) -> None:
        self.detail = detail
        super().__init__(
            f"AA-28 SAFETY VIOLATION: both grippers would be unlocked — {detail}. "
            "Halting immediately."
        )


class UnverifiedStateError(RecoveryError):
    """Raised when motion is requested from an unverified tube state.

    Recovery never commands motion without first verifying tube state (AC#3).
    Live mode must verify; dry-run may skip with ``require_pass=False``.
    """

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(
            f"AA-28: motion refused from unverified state — {reason}. "
            "Run TubeStateVerifier.verify() and confirm passed=True before motion."
        )


class ResumeRefusedError(RecoveryError):
    """Raised when a resume is refused because persisted state is inconsistent.

    Attributes
    ----------
    reason:
        Human-readable explanation of why the resume was refused.
    """

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(f"AA-28 resume refused: {reason}")


# ---------------------------------------------------------------------------
# Fault catalogue
# ---------------------------------------------------------------------------


class RecoveryFaultKind(str, enum.Enum):
    """Enumeration of every fault in the AA-28 fault catalogue (AC#2).

    Values are used as the ``fault_kind`` field in AA-7 fault events emitted
    by :class:`FaultResponder`.
    """

    INSERTION_FAILURE = "insertion_failure_after_retries"
    LOCK_VERIFICATION_FAILURE = "lock_verification_failure"
    MID_ADVANCE_SERVO_FAULT = "mid_advance_servo_fault"
    UNEXPECTED_TUBE_DISPLACEMENT = "unexpected_tube_displacement"
    USB_DEVICE_LOSS = "usb_device_loss"


class RecoveryResponse(str, enum.Enum):
    """Automatic response taken for each fault.

    SAFE_HALT:
        Stop the cycle cleanly. All arms remain in their current locked state.
        No further motion is commanded. Holder stays LOCKED.

    ESTOP_HOLD_AND_HALT:
        Trigger AA-10 EStop (HOLD mode — arms stay in place under torque) then
        halt. Used for faults where the robot state is uncertain (servo fault,
        USB loss).

    HALT_REQUIRE_RECALIBRATION:
        Stop cycle, log a message requiring the operator to re-run calibration
        (AA-16) before resuming. Used when the measured tube pose differs
        significantly from the model.
    """

    SAFE_HALT = "safe_halt"
    ESTOP_HOLD_AND_HALT = "estop_hold_and_halt"
    HALT_REQUIRE_RECALIBRATION = "halt_require_recalibration"


# Fault catalogue: mapping from fault kind → defined response (AC#2).
FAULT_CATALOGUE: Dict[RecoveryFaultKind, RecoveryResponse] = {
    RecoveryFaultKind.INSERTION_FAILURE: RecoveryResponse.SAFE_HALT,
    RecoveryFaultKind.LOCK_VERIFICATION_FAILURE: RecoveryResponse.SAFE_HALT,
    RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT: RecoveryResponse.ESTOP_HOLD_AND_HALT,
    RecoveryFaultKind.UNEXPECTED_TUBE_DISPLACEMENT: RecoveryResponse.HALT_REQUIRE_RECALIBRATION,
    RecoveryFaultKind.USB_DEVICE_LOSS: RecoveryResponse.ESTOP_HOLD_AND_HALT,
}

# ---------------------------------------------------------------------------
# Typed AA-7 fault event constructors (AC#2)
# ---------------------------------------------------------------------------


def fault_event(
    *,
    fault_kind: RecoveryFaultKind,
    response: RecoveryResponse,
    detail: str,
    cycle_index: int = 0,
    arm: Optional[str] = None,
) -> Dict[str, Any]:
    """Return AA-7 emit-kwargs for a recovery fault event.

    The returned dict is passed directly to ``session_logger.emit(**...)``.
    """
    return {
        "role": arm or "orchestrator",
        "subsystem": "recovery",
        "event_type": "recovery_fault",
        "severity": "CRITICAL",
        "payload": {
            "fault_kind": fault_kind.value,
            "response": response.value,
            "detail": detail,
            "cycle_index": cycle_index,
            "arm": arm,
        },
    }


def recovery_response_event(
    *,
    fault_kind: RecoveryFaultKind,
    response: RecoveryResponse,
    succeeded: bool,
    detail: str,
    cycle_index: int = 0,
) -> Dict[str, Any]:
    """Return AA-7 emit-kwargs for a recovery response outcome event."""
    return {
        "role": "orchestrator",
        "subsystem": "recovery",
        "event_type": "recovery_response",
        "severity": "ERROR" if not succeeded else "WARNING",
        "payload": {
            "fault_kind": fault_kind.value,
            "response": response.value,
            "succeeded": succeeded,
            "detail": detail,
            "cycle_index": cycle_index,
        },
    }


# ---------------------------------------------------------------------------
# Verification config
# ---------------------------------------------------------------------------


@dataclass
class VerificationThresholds:
    """Thresholds used by :class:`TubeStateVerifier`.

    All values are in SI units (metres) or degrees as noted.  A ``None``
    threshold means "not yet measured"; the verifier returns
    ``VerificationResult(passed=False, refused=True)`` in live mode.

    pos_tolerance_m:
        Maximum allowed distance between FK TCP translation and the
        attachment-point chassis-frame translation.  ``None`` → unverified.

    locked_band_tolerance_deg:
        How far outside the configured locked band the servo position may be
        and still be counted as "in band" (a small fudge for servo deadband).
        Defaults to 0.0 (exact band check).

    Thresholds are unverified until an owner-supervised hardware session
    measures them; until then live mode refuses to proceed.
    """

    pos_tolerance_m: Optional[float] = None
    locked_band_tolerance_deg: float = 0.0
    # Set True when owner confirms thresholds are from hardware measurement.
    verified: bool = False


# ---------------------------------------------------------------------------
# Verification result
# ---------------------------------------------------------------------------


@dataclass
class ArmVerificationResult:
    """Result of verifying one arm's state.

    Attributes
    ----------
    arm:
        ``"left"`` or ``"right"``.
    index:
        The lattice index the model believes this arm holds (``None`` =
        model says not engaged; this is always OK).
    sm_state:
        The AA-17 :class:`~inchworm.gripper_lock.GripperState` at check time.
    gripper_pos_deg:
        Last commanded (or read, if available) gripper position (degrees).
        ``None`` if not readable in the current mode.
    fk_tcp_chassis_m:
        FK TCP translation in chassis frame (metres).  ``None`` if not
        computable (no joint readings in dry-run).
    expected_tcp_chassis_m:
        Attachment-point translation from the FrameModel (metres).
        ``None`` if the model says arm is not engaged.
    passed:
        ``True`` iff all applicable checks pass.
    refused:
        ``True`` iff a live-mode required threshold is ``None`` (cannot verify).
    reasons:
        Human-readable list of every failing check.
    """

    arm: str
    index: Optional[int]
    sm_state: GripperState
    gripper_pos_deg: Optional[float]
    fk_tcp_chassis_m: Optional[List[float]]
    expected_tcp_chassis_m: Optional[List[float]]
    passed: bool
    refused: bool
    reasons: List[str] = field(default_factory=list)


@dataclass
class VerificationResult:
    """Combined verification result for both arms.

    Attributes
    ----------
    left:
        Per-arm result for the left arm.
    right:
        Per-arm result for the right arm.
    passed:
        ``True`` iff both arms pass.
    refused:
        ``True`` iff any threshold is ``None`` in live mode (cannot verify).
    reasons:
        All failing reasons across both arms.
    """

    left: ArmVerificationResult
    right: ArmVerificationResult

    @property
    def passed(self) -> bool:
        return self.left.passed and self.right.passed

    @property
    def refused(self) -> bool:
        return self.left.refused or self.right.refused

    @property
    def reasons(self) -> List[str]:
        return self.left.reasons + self.right.reasons


# ---------------------------------------------------------------------------
# TubeStateVerifier (AC#1)
# ---------------------------------------------------------------------------


class TubeStateVerifier:
    """Verifies tube attachment state before and after each crawl cycle (AC#1).

    For each arm the verifier checks:

    1. If the :class:`~inchworm.frames.FrameModel` says the arm holds index k,
       the AA-17 :class:`~inchworm.gripper_lock.GripperLockSM` must be in LOCKED.
    2. The last commanded gripper position must be inside the configured locked
       band (``GripperBandConfig.locked_low_deg`` .. ``locked_high_deg``), within
       ``thresholds.locked_band_tolerance_deg`` of fuzz.
    3. If ``joint_q`` is provided, the FK TCP translation must be within
       ``thresholds.pos_tolerance_m`` metres of the attachment-point
       chassis-frame position from the FrameModel.

    When ``thresholds.pos_tolerance_m`` is ``None`` (unverified) check #3 is
    skipped *and* the result is marked ``refused=True`` in live mode.  In
    dry-run mode, ``refused`` does not block execution (the caller decides).

    Parameters
    ----------
    frame_model:
        Live :class:`~inchworm.frames.FrameModel`.
    left_sm:
        AA-17 state machine for the left arm.
    right_sm:
        AA-17 state machine for the right arm.
    thresholds:
        :class:`VerificationThresholds` with tolerances.
    left_chain:
        Optional kinematic chain for FK (left arm).
    right_chain:
        Optional kinematic chain for FK (right arm).
    T_chassis_left:
        4×4 HTM: left arm base in chassis frame.
    T_chassis_right:
        4×4 HTM: right arm base in chassis frame.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    dry_run:
        When ``True`` the verifier does not raise on ``refused=True``.
    """

    def __init__(
        self,
        *,
        frame_model: FrameModel,
        left_sm: GripperLockSM,
        right_sm: GripperLockSM,
        thresholds: VerificationThresholds,
        left_chain: Optional[Any] = None,
        right_chain: Optional[Any] = None,
        T_chassis_left: Optional[Any] = None,
        T_chassis_right: Optional[Any] = None,
        logger: Optional[Any] = None,
        dry_run: bool = False,
    ) -> None:
        self._fm = frame_model
        self._left_sm = left_sm
        self._right_sm = right_sm
        self._thresholds = thresholds
        self._left_chain = left_chain
        self._right_chain = right_chain
        self._T_chassis_left = (
            np.array(T_chassis_left, dtype=np.float64)
            if T_chassis_left is not None
            else np.eye(4, dtype=np.float64)
        )
        self._T_chassis_right = (
            np.array(T_chassis_right, dtype=np.float64)
            if T_chassis_right is not None
            else np.eye(4, dtype=np.float64)
        )
        self._logger = logger
        self._dry_run = dry_run

    def verify(
        self,
        *,
        left_q: Optional[Any] = None,
        right_q: Optional[Any] = None,
        context: str = "verify",
    ) -> VerificationResult:
        """Run all checks and return a :class:`VerificationResult`.

        Parameters
        ----------
        left_q:
            Optional joint angles (radians, 5-vector) for the left arm.  When
            provided, FK is computed for check #3.
        right_q:
            Optional joint angles for the right arm.
        context:
            String tag for logging (e.g. ``"pre_cycle"`` or ``"post_cycle"``).

        Returns
        -------
        VerificationResult
            Full per-arm results.  Inspect ``.passed`` before commanding motion.
        """
        held = self._fm.held_indices()
        left_result = self._verify_arm(
            arm="left",
            sm=self._left_sm,
            held_index=held["left"],
            q=left_q,
            T_chassis_base=self._T_chassis_left,
            chain=self._left_chain,
        )
        right_result = self._verify_arm(
            arm="right",
            sm=self._right_sm,
            held_index=held["right"],
            q=right_q,
            T_chassis_base=self._T_chassis_right,
            chain=self._right_chain,
        )
        result = VerificationResult(left=left_result, right=right_result)
        self._emit_result(result, context)
        return result

    # ------------------------------------------------------------------
    # Internal: verify one arm
    # ------------------------------------------------------------------

    def _verify_arm(
        self,
        *,
        arm: str,
        sm: GripperLockSM,
        held_index: Optional[int],
        q: Optional[Any],
        T_chassis_base: Any,
        chain: Optional[Any],
    ) -> ArmVerificationResult:
        reasons: List[str] = []
        refused = False

        sm_state = sm.state
        gripper_pos_deg = sm._last_commanded_pos_deg  # noqa: SLF001 (pkg-internal)

        # ── Check 1: SM state must be LOCKED if holding ──────────────────
        if held_index is not None and sm_state != GripperState.LOCKED:
            reasons.append(
                f"{arm}: model holds index {held_index} but SM state is "
                f"{sm_state.name}, not LOCKED"
            )

        # ── Check 2: gripper position in locked band ──────────────────────
        if held_index is not None:
            band_ok, band_reason = self._check_locked_band(arm, sm, gripper_pos_deg)
            if not band_ok:
                reasons.append(band_reason)
            # Mark refused if no threshold data is available to check band
            if gripper_pos_deg is None:
                refused = True

        # ── Check 3: FK TCP within tolerance of attachment-point pose ──────
        fk_tcp_chassis: Optional[List[float]] = None
        expected_tcp_chassis: Optional[List[float]] = None

        if held_index is not None:
            expected_tcp_m = attachment_point_in_chassis_frame(
                self._fm.tube_pose, held_index
            )[:3, 3]
            expected_tcp_chassis = expected_tcp_m.tolist()

            if q is not None and chain is not None:
                # Compute FK in arm base frame, then transform to chassis frame
                T_base_tcp = chain.fk(np.array(q, dtype=np.float64))
                T_chassis_tcp = T_chassis_base @ T_base_tcp
                tcp_m = T_chassis_tcp[:3, 3]
                fk_tcp_chassis = tcp_m.tolist()

                tol = self._thresholds.pos_tolerance_m
                if tol is None:
                    # Cannot verify: threshold not yet measured
                    if not self._dry_run:
                        reasons.append(
                            f"{arm}: pos_tolerance_m is None (unverified threshold); "
                            "FK TCP vs attachment pose cannot be checked in live mode"
                        )
                        refused = True
                else:
                    dist = float(np.linalg.norm(tcp_m - expected_tcp_m))
                    if dist > tol:
                        reasons.append(
                            f"{arm}: FK TCP dist={dist*1000:.2f} mm from expected "
                            f"attachment point (tol={tol*1000:.2f} mm) — tube may "
                            "have displaced"
                        )
            elif held_index is not None:
                # Orchestrator 2026-10-24: in LIVE mode missing joint readings
                # or a missing chain means tube displacement cannot be checked.
                # That must REFUSE (AC#1, AC#3), not silently pass.
                if not self._dry_run:
                    reasons.append(
                        f"{arm}: no joint readings or kinematic chain supplied; "
                        "FK TCP vs attachment pose cannot be verified in live mode"
                    )
                    refused = True

        passed = len(reasons) == 0 and not refused
        return ArmVerificationResult(
            arm=arm,
            index=held_index,
            sm_state=sm_state,
            gripper_pos_deg=gripper_pos_deg,
            fk_tcp_chassis_m=fk_tcp_chassis,
            expected_tcp_chassis_m=expected_tcp_chassis,
            passed=passed,
            refused=refused,
            reasons=reasons,
        )

    def _check_locked_band(
        self,
        arm: str,
        sm: GripperLockSM,
        gripper_pos_deg: Optional[float],
    ) -> Tuple[bool, str]:
        """Return (ok, reason_if_not_ok) for the locked-band check."""
        # Get the band config via the SM's private accessor
        try:
            bands = sm._bands_for_role()  # noqa: SLF001
        except Exception:
            return False, f"{arm}: could not read gripper band config"

        if not _bands_configured(bands):
            # Bands not yet measured — cannot check
            if self._dry_run:
                return True, ""  # dry-run: skip band check
            return False, (
                f"{arm}: gripper locked band not configured "
                "(null boundaries); cannot verify in live mode"
            )

        if gripper_pos_deg is None:
            if self._dry_run:
                return True, ""
            return False, f"{arm}: no gripper position reading to check against locked band"

        assert bands is not None
        low = bands.locked_low_deg
        high = bands.locked_high_deg
        assert low is not None and high is not None  # guaranteed by _bands_configured

        tol = self._thresholds.locked_band_tolerance_deg
        in_band = (low - tol) <= gripper_pos_deg <= (high + tol)
        if not in_band:
            return False, (
                f"{arm}: gripper pos {gripper_pos_deg:.2f}° outside locked band "
                f"[{low:.2f}°, {high:.2f}°] ± {tol:.2f}°"
            )
        return True, ""

    def _emit_result(self, result: VerificationResult, context: str) -> None:
        if self._logger is None:
            return
        severity = "INFO" if result.passed else ("WARNING" if not result.refused else "ERROR")
        try:
            self._logger.emit(
                role="orchestrator",
                subsystem="recovery",
                event_type="tube_state_verification",
                severity=severity,
                payload={
                    "context": context,
                    "passed": result.passed,
                    "refused": result.refused,
                    "reasons": result.reasons,
                    "left_sm_state": result.left.sm_state.name,
                    "right_sm_state": result.right.sm_state.name,
                    "left_index": result.left.index,
                    "right_index": result.right.index,
                },
            )
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# Safety invariant guard (AC#3)
# ---------------------------------------------------------------------------


def assert_not_both_unlocked(
    left_sm: GripperLockSM,
    right_sm: GripperLockSM,
    context: str = "",
) -> None:
    """Raise :class:`BothGrippersUnlockedError` if both grippers are unlocked.

    The check uses the AA-17 state machine states.  LOCKED is the only state
    that counts as "holding the tube".  UNKNOWN, UNLOCKED, APPROACH_READY,
    LOCKING, UNLOCKING, or FAULT are all treated as "not holding".

    This must be called at every state-transition that could lead to both
    grippers being open.  Never both grippers unlocked while the tube is
    attached (AGENTS.md invariant).

    Parameters
    ----------
    left_sm:
        AA-17 state machine for the left arm.
    right_sm:
        AA-17 state machine for the right arm.
    context:
        Human-readable label for the operation that triggered the check
        (used in the error message).
    """
    left_locked = left_sm.state == GripperState.LOCKED
    right_locked = right_sm.state == GripperState.LOCKED
    if not left_locked and not right_locked:
        raise BothGrippersUnlockedError(
            f"left arm state={left_sm.state.name}, "
            f"right arm state={right_sm.state.name}"
            + (f" [context: {context}]" if context else "")
        )


def assert_state_verified(
    result: VerificationResult,
    *,
    require_pass: bool = True,
) -> None:
    """Raise :class:`UnverifiedStateError` if the verification result is not passing.

    Parameters
    ----------
    result:
        :class:`VerificationResult` from :class:`TubeStateVerifier`.
    require_pass:
        When ``True`` (default), raises on any failure including refused.
        Set ``False`` in dry-run to get a diagnostic result without raising.
    """
    if not require_pass:
        return
    if result.refused:
        raise UnverifiedStateError(
            "verification thresholds are not configured (null); "
            "cannot confirm tube state in live mode. "
            f"Reasons: {'; '.join(result.reasons) or 'thresholds unset'}"
        )
    if not result.passed:
        raise UnverifiedStateError(
            f"tube state verification failed: {'; '.join(result.reasons)}"
        )


# ---------------------------------------------------------------------------
# Fault responder (AC#2)
# ---------------------------------------------------------------------------


class FaultResponder:
    """Executes the defined response for each fault kind (AC#2).

    Parameters
    ----------
    left_sm:
        AA-17 state machine for the left arm.
    right_sm:
        AA-17 state machine for the right arm.
    estop:
        Optional AA-10 :class:`~inchworm.estop.EStop`.  Required for
        :attr:`RecoveryResponse.ESTOP_HOLD_AND_HALT`; if ``None``, only a
        log event is emitted for that response.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    dry_run:
        When ``True``, e-stop is not triggered (only logged).
    """

    def __init__(
        self,
        *,
        left_sm: GripperLockSM,
        right_sm: GripperLockSM,
        estop: Optional[Any] = None,
        logger: Optional[Any] = None,
        dry_run: bool = False,
    ) -> None:
        self._left_sm = left_sm
        self._right_sm = right_sm
        self._estop = estop
        self._logger = logger
        self._dry_run = dry_run

    def respond(
        self,
        *,
        fault_kind: RecoveryFaultKind,
        detail: str,
        cycle_index: int = 0,
        arm: Optional[str] = None,
    ) -> Tuple[RecoveryResponse, bool]:
        """Execute the catalogue response for *fault_kind* and return (response, succeeded).

        Emits AA-7 fault event + response event.  Never raises (errors are
        returned as ``succeeded=False``).

        Returns
        -------
        (RecoveryResponse, bool)
            The response that was executed and whether it succeeded.
        """
        response = FAULT_CATALOGUE[fault_kind]

        # Emit the fault event first
        self._emit(**fault_event(
            fault_kind=fault_kind,
            response=response,
            detail=detail,
            cycle_index=cycle_index,
            arm=arm,
        ))

        succeeded = True
        try:
            if response == RecoveryResponse.SAFE_HALT:
                # Verify at least one gripper is still LOCKED
                left_locked = self._left_sm.state == GripperState.LOCKED
                right_locked = self._right_sm.state == GripperState.LOCKED
                if not left_locked and not right_locked:
                    _LOG.error(
                        "AA-28 safe_halt: both grippers unlocked at halt — "
                        "invariant violated; no further action"
                    )
                    succeeded = False
                else:
                    _LOG.info(
                        "AA-28 safe_halt: left=%s right=%s — holder stays LOCKED",
                        self._left_sm.state.name,
                        self._right_sm.state.name,
                    )

            elif response == RecoveryResponse.ESTOP_HOLD_AND_HALT:
                if self._dry_run or self._estop is None:
                    _LOG.warning(
                        "AA-28 estop_hold_and_halt (dry-run/no-estop): "
                        "would trigger EStop.stop() for %s", fault_kind.value
                    )
                else:
                    try:
                        self._estop.stop()
                    except Exception as exc:  # noqa: BLE001
                        _LOG.error("AA-28 EStop.stop() raised: %s", exc)
                        succeeded = False

            elif response == RecoveryResponse.HALT_REQUIRE_RECALIBRATION:
                _LOG.error(
                    "AA-28 halt_require_recalibration: tube displacement detected — "
                    "operator must re-run AA-16 calibration before resuming. "
                    "detail=%s", detail
                )

        except Exception as exc:  # noqa: BLE001
            _LOG.error("AA-28 fault response raised: %s", exc)
            succeeded = False

        self._emit(**recovery_response_event(
            fault_kind=fault_kind,
            response=response,
            succeeded=succeeded,
            detail=detail,
            cycle_index=cycle_index,
        ))
        return response, succeeded

    def _emit(self, **kwargs: Any) -> None:
        if self._logger is not None:
            try:
                self._logger.emit(**kwargs)
            except Exception:  # noqa: BLE001
                pass
        else:
            _LOG.log(
                logging.CRITICAL if kwargs.get("severity") == "CRITICAL" else logging.ERROR,
                "AA-28 fault: %s", kwargs.get("payload", {}),
            )


# ---------------------------------------------------------------------------
# Cycle phase enum
# ---------------------------------------------------------------------------


class CyclePhase(str, enum.Enum):
    """Phase labels for the crawl cycle (used in checkpoints)."""

    PRE_CYCLE = "pre_cycle"
    UNLOCK = "unlock"
    RETRACT = "retract"
    APPROACH = "approach"
    SUPERVISOR = "supervisor"
    LOCK = "lock"
    ADVANCE = "advance"
    FOLD = "fold"
    POST_CYCLE = "post_cycle"
    HALTED = "halted"


# ---------------------------------------------------------------------------
# CycleStateCheckpoint (AC#4)
# ---------------------------------------------------------------------------


class CycleStateCheckpoint:
    """Atomically persists and restores interrupted-cycle state (AC#4).

    After every cycle phase the caller should call :meth:`save`.  On resume,
    call :meth:`load` then :meth:`validate_against_live_state`; if the live
    state does not match the persisted state, a :class:`ResumeRefusedError` is
    raised with a clear reason.

    The checkpoint file is written atomically via write-to-tmp + fsync + rename
    so a power-cycle during the write never leaves a half-written file.

    Schema
    ------
    ::

        {
          "schema_version": 1,
          "cycle_index": <int>,
          "phase": "<CyclePhase value>",
          "frame_model": { ... },   # FrameModel.to_dict()
          "gripper_states": {
              "left": "<GripperState name>",
              "right": "<GripperState name>"
          },
          "ts_iso": "<UTC ISO timestamp>"
        }

    Parameters
    ----------
    checkpoint_path:
        Path to the checkpoint JSON file (inside the session directory).
    """

    def __init__(self, checkpoint_path: Path) -> None:
        self._path = Path(checkpoint_path)

    @property
    def path(self) -> Path:
        return self._path

    def save(
        self,
        *,
        cycle_index: int,
        phase: CyclePhase,
        frame_model: FrameModel,
        left_sm: GripperLockSM,
        right_sm: GripperLockSM,
    ) -> None:
        """Atomically write a checkpoint to disk.

        Parameters
        ----------
        cycle_index:
            Current zero-based cycle index.
        phase:
            The phase that just completed.
        frame_model:
            The live :class:`~inchworm.frames.FrameModel` (serialised via
            ``to_dict()``).
        left_sm / right_sm:
            AA-17 state machines; their ``.state`` names are recorded.
        """
        from datetime import datetime, timezone

        data = {
            "schema_version": _CHECKPOINT_SCHEMA_VERSION,
            "cycle_index": cycle_index,
            "phase": phase.value,
            "frame_model": frame_model.to_dict(),
            "gripper_states": {
                "left": left_sm.state.name,
                "right": right_sm.state.name,
            },
            "ts_iso": datetime.now(timezone.utc).isoformat(),
        }
        self._atomic_write(json.dumps(data, indent=2))
        _LOG.debug(
            "AA-28 checkpoint saved: cycle=%d phase=%s", cycle_index, phase.value
        )

    def load(self) -> Dict[str, Any]:
        """Load and parse the checkpoint from disk.

        Raises
        ------
        ResumeRefusedError
            If the file does not exist, is not valid JSON, or has an
            incompatible schema version.
        """
        if not self._path.exists():
            raise ResumeRefusedError(
                f"checkpoint file not found at {self._path}; "
                "no interrupted cycle to resume"
            )
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ResumeRefusedError(
                f"checkpoint file at {self._path} is not valid JSON: {exc}"
            ) from exc

        sv = data.get("schema_version")
        if sv != _CHECKPOINT_SCHEMA_VERSION:
            raise ResumeRefusedError(
                f"checkpoint schema_version {sv!r} != "
                f"current {_CHECKPOINT_SCHEMA_VERSION!r}; stale file"
            )
        return data

    def validate_against_live_state(
        self,
        data: Dict[str, Any],
        *,
        frame_model: FrameModel,
        left_sm: GripperLockSM,
        right_sm: GripperLockSM,
        verifier: Optional[TubeStateVerifier] = None,
        left_q: Optional[Any] = None,
        right_q: Optional[Any] = None,
    ) -> CyclePhase:
        """Validate persisted state against freshly verified live state.

        Uses the FrameModel's own resume checks (advance_count,
        config_hash, pitch_mm) via the persisted ``frame_model`` sub-dict, plus:

        * AA-17 gripper states must match the checkpoint.
        * If a :class:`TubeStateVerifier` is provided, it is called and its
          result must pass (or have ``refused=False``).

        Returns the :class:`CyclePhase` to resume from on success.

        Raises
        ------
        ResumeRefusedError
            If any check fails; the reason is human-readable.
        """
        # 1. Frame model consistency (reuse AA-15 checks)
        from inchworm.frames import _check_resume, ResumeError
        try:
            _check_resume(
                data["frame_model"],
                current_config_hash=frame_model._config_hash,  # noqa: SLF001
            )
        except ResumeError as exc:
            raise ResumeRefusedError(f"FrameModel resume check failed: {exc}") from exc

        # 2. Advance count must match live FrameModel
        persisted_advance = data["frame_model"]["tube_pose"]["advance_count"]
        live_advance = frame_model.tube_pose.advance_count
        if persisted_advance != live_advance:
            raise ResumeRefusedError(
                f"advance_count mismatch: checkpoint={persisted_advance} "
                f"live={live_advance}; robot has moved since checkpoint was written"
            )

        # 3. Lattice state must match
        ps_lattice = data["frame_model"].get("lattice_state", {})
        live_lattice = frame_model.lattice_state
        for side in ("left", "right"):
            ps_idx = ps_lattice.get(f"{side}_index")
            live_idx = getattr(live_lattice, f"{side}_index")
            if ps_idx != live_idx:
                raise ResumeRefusedError(
                    f"lattice_state.{side}_index mismatch: "
                    f"checkpoint={ps_idx!r} live={live_idx!r}"
                )

        # 4. Gripper states must match
        ps_grippers = data.get("gripper_states", {})
        for side, sm in (("left", left_sm), ("right", right_sm)):
            ps_name = ps_grippers.get(side, "UNKNOWN")
            live_name = sm.state.name
            if ps_name != live_name:
                raise ResumeRefusedError(
                    f"gripper_states.{side} mismatch: "
                    f"checkpoint={ps_name!r} live={live_name!r}"
                )

        # 5. If verifier supplied, run it and check
        if verifier is not None:
            v_result = verifier.verify(
                left_q=left_q,
                right_q=right_q,
                context="resume_validation",
            )
            if not v_result.passed and not v_result.refused:
                raise ResumeRefusedError(
                    "live tube state verification failed after checkpoint load: "
                    + "; ".join(v_result.reasons)
                )
            if v_result.refused:
                raise ResumeRefusedError(
                    "tube state verification refused (unverified thresholds); "
                    "cannot safely resume in live mode"
                )

        phase_str = data.get("phase", "")
        try:
            return CyclePhase(phase_str)
        except ValueError:
            raise ResumeRefusedError(
                f"checkpoint phase {phase_str!r} is not a valid CyclePhase value"
            )

    def _atomic_write(self, text: str) -> None:
        """Write ``text`` to ``self._path`` atomically.

        Writes to a sibling tmp file, fsyncs, then renames.
        """
        parent = self._path.parent
        parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(
            dir=str(parent), prefix=self._path.stem + "_tmp_", suffix=".json"
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                fh.write(text)
                fh.flush()
                os.fsync(fh.fileno())
        except Exception:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise
        os.replace(tmp_path, self._path)


# ---------------------------------------------------------------------------
# PostRunReport (AC#6)
# ---------------------------------------------------------------------------


@dataclass
class FaultRecord:
    """One fault entry in the post-run report."""

    ts_iso: str
    fault_kind: str
    response: str
    succeeded: bool
    detail: str
    cycle_index: int
    arm: Optional[str]


@dataclass
class PostRunReport:
    """Structured post-run fault report built from the AA-7 event log (AC#6).

    Attributes
    ----------
    total_faults:
        Total number of recovery faults encountered.
    faults:
        List of :class:`FaultRecord` objects, one per fault event.
    all_responses_succeeded:
        True iff every fault response was recorded as succeeded=True.
    summary_text:
        Multi-line human-readable summary.
    """

    total_faults: int
    faults: List[FaultRecord]
    all_responses_succeeded: bool
    summary_text: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_faults": self.total_faults,
            "faults": [
                {
                    "ts_iso": f.ts_iso,
                    "fault_kind": f.fault_kind,
                    "response": f.response,
                    "succeeded": f.succeeded,
                    "detail": f.detail,
                    "cycle_index": f.cycle_index,
                    "arm": f.arm,
                }
                for f in self.faults
            ],
            "all_responses_succeeded": self.all_responses_succeeded,
            "summary_text": self.summary_text,
        }


def build_post_run_report(event_records: List[Dict[str, Any]]) -> PostRunReport:
    """Build a :class:`PostRunReport` from a list of AA-7 event record dicts.

    Reads ``recovery_fault`` and ``recovery_response`` events from the
    event log.  Pairs them by cycle_index + fault_kind.

    Parameters
    ----------
    event_records:
        List of dicts in the :class:`~inchworm.telemetry.EventRecord` format
        (i.e. the parsed lines from ``event_log.jsonl``).

    Returns
    -------
    PostRunReport
        Structured report.
    """
    # Build a response lookup: (cycle_index, fault_kind) → response outcome
    response_outcomes: Dict[Tuple[int, str], Tuple[str, bool]] = {}
    for rec in event_records:
        if rec.get("event_type") == "recovery_response":
            p = rec.get("payload", {})
            key = (int(p.get("cycle_index", 0)), str(p.get("fault_kind", "")))
            response_outcomes[key] = (
                str(p.get("response", "")),
                bool(p.get("succeeded", False)),
            )

    faults: List[FaultRecord] = []
    for rec in event_records:
        if rec.get("event_type") != "recovery_fault":
            continue
        p = rec.get("payload", {})
        ci = int(p.get("cycle_index", 0))
        fk = str(p.get("fault_kind", "unknown"))
        key = (ci, fk)
        resp_name, succeeded = response_outcomes.get(key, (p.get("response", "unknown"), False))
        faults.append(FaultRecord(
            ts_iso=rec.get("ts_iso", ""),
            fault_kind=fk,
            response=resp_name,
            succeeded=succeeded,
            detail=str(p.get("detail", "")),
            cycle_index=ci,
            arm=p.get("arm"),
        ))

    all_ok = all(f.succeeded for f in faults) if faults else True

    lines = [
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━",
        "AA-28 Post-Run Fault Report",
        f"  Total faults:         {len(faults)}",
        f"  All responses OK:     {all_ok}",
    ]
    if not faults:
        lines.append("  (no faults encountered)")
    else:
        lines.append("")
        for i, f in enumerate(faults, 1):
            status = "✓" if f.succeeded else "✗"
            lines.append(
                f"  [{i:3d}] {f.ts_iso[:19]} cycle={f.cycle_index} "
                f"{f.fault_kind} → {f.response} {status}"
            )
            if f.detail:
                lines.append(f"        detail: {f.detail[:120]}")
    lines.append("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")

    return PostRunReport(
        total_faults=len(faults),
        faults=faults,
        all_responses_succeeded=all_ok,
        summary_text="\n".join(lines),
    )


def build_post_run_report_from_jsonl(log_path: Path) -> PostRunReport:
    """Load an AA-7 JSONL event log and build a :class:`PostRunReport`.

    Parameters
    ----------
    log_path:
        Path to the ``event_log.jsonl`` file from a session.

    Returns
    -------
    PostRunReport
    """
    records: List[Dict[str, Any]] = []
    if log_path.exists():
        for line in log_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return build_post_run_report(records)
