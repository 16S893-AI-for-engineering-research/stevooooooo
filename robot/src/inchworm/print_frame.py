"""AA-33: Apply the per-hop leveling correction to the DexArm print frame.

Overview
--------
After every crawl hop the tube settles in a slightly different position, so
the DexArm print frame must be corrected before the next segment is printed.
This module closes the leveling loop: it takes a :class:`~inchworm.leveling.LevelingResult`
(produced by AA-32), converts it into a 4×4 homogeneous transform (Z offset +
rx + ry tilt), and applies that transform to the nominal print frame before
any print motion is allowed.

Correction policy — Absolute replacement (AC#5)
------------------------------------------------
Each hop's correction **replaces** the previous one relative to the nominal
(uncorrected) frame.  The reasoning:

* The leveling scan measures the *absolute* pose of the tube end face relative
  to the DexArm frame, not the *delta* from the previous scan.
* Accumulating deltas would propagate measurement error across hops, causing
  the frame to drift away from the true tube pose.
* Replacing means each hop starts fresh from the nominal frame plus only the
  current measured correction — scan noise cannot compound.
* Therefore ``T_corrected = T_correction_current @ T_nominal`` where
  ``T_correction_current`` is derived solely from the latest
  :class:`~inchworm.leveling.LevelingResult`.

This policy is explicit: the :class:`PrintFrame` stores the nominal frame
separately from the active correction; ``apply_correction`` always computes
the corrected frame as ``T_correction @ T_nominal``, never as
``T_old_correction @ T_nominal @ T_delta``.

State machine — Ordering enforcement (AC#3)
-------------------------------------------
The :class:`PrintFrame` has four states:

``AWAITING_FOLD``
    Initial state.  No correction may be applied and no print motion
    is allowed.

``FOLDED_READY``
    Entered by calling :meth:`PrintFrame.accept_fold_token`.  The caller
    (AA-19 fold sequencer) calls this after a confirmed fold.  The
    correction may now be applied.

``APPLIED_VERIFIED``
    Entered by :meth:`PrintFrame.apply_correction` after the sanity check,
    transform application, and successful verification rescan.  Print motion
    requests (the AA-34 interface) are only allowed in this state.

``HALTED``
    Entered when a second successive sanity-bound violation occurs.  All
    operations except inspection are refused.

The state machine resets to ``AWAITING_FOLD`` after every hop via
:meth:`PrintFrame.reset_for_next_hop`.

Sanity bound and rescan (AC#2)
-------------------------------
Corrections whose magnitude (max of |Z|, |rx|, |ry| normalised to mm/rad
against configured bounds) exceed the configured sanity bound trigger a
rescan via the injected ``rescan_fn`` callable.  If the rescan also exceeds
the bound, a :class:`FrameCorrectionHalt` is raised and the state transitions
to ``HALTED``.

Verification rescan (AC#4)
---------------------------
After the correction transform is applied, a verification rescan is performed
(same ``rescan_fn``).  The residual (the magnitude of the *new* correction
that would be needed after the applied one) must be within the configured
``residual_tolerance_mm``.  A second call to ``rescan_fn`` is used for this
verification; real hardware would measure the actual residual with the new
frame active.  Synthetic ``LevelingResult`` objects with known residuals
serve as the test vector (see tests).

Per-hop record (AC#6)
----------------------
A JSON-lines record is written to the session artifacts directory for every
hop.  Each record contains the measured correction, the applied transform,
and the verified residual.  The record is written unconditionally (even on
failure paths) to aid post-mortem analysis.

AA-7 telemetry
--------------
All significant events are emitted via an optional
:class:`~inchworm.telemetry.SessionLogger`.

Usage::

    from inchworm.print_frame import PrintFrame, PrintFrameConfig
    from inchworm.leveling import LevelingResult

    cfg = PrintFrameConfig()
    pf = PrintFrame(config=cfg, artifacts_dir=ctx.artifact_dir)
    pf.accept_fold_token()
    pf.apply_correction(leveling_result, rescan_fn=my_rescan)
    T = pf.get_corrected_transform()   # 4×4 HTM ready for DexArm print moves
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum, auto
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import numpy.typing as npt

from inchworm.leveling import LevelingRefused, LevelingResult

log = logging.getLogger(__name__)

# Type aliases
Mat44 = npt.NDArray[np.float64]

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class FrameCorrectionHalt(RuntimeError):
    """Raised when the print frame cannot be corrected safely and must halt.

    This happens when both the initial scan and the rescan produce corrections
    that exceed the configured sanity bound.  The :class:`PrintFrame` state
    transitions to :attr:`~PrintFrameState.HALTED`.

    Attributes
    ----------
    reason:
        Machine-readable reason code:
        ``"double_sanity_violation"`` — both scans exceeded the sanity bound.
        ``"verification_failed"`` — verification rescan residual exceeds tolerance.
    detail:
        Human-readable explanation.
    scan_id_1:
        Scan identifier for the first (initial) scan.
    scan_id_2:
        Scan identifier for the rescan (or ``None`` if the halt is from
        verification rather than a double sanity violation).
    """

    def __init__(
        self,
        reason: str,
        detail: str = "",
        scan_id_1: str = "",
        scan_id_2: Optional[str] = None,
    ) -> None:
        self.reason = reason
        self.detail = detail
        self.scan_id_1 = scan_id_1
        self.scan_id_2 = scan_id_2
        super().__init__(f"FrameCorrectionHalt({reason}): {detail}")


class PrintFrameOrderingError(RuntimeError):
    """Raised when an operation is attempted in the wrong state.

    Attributes
    ----------
    current_state:
        The current :class:`PrintFrameState` when the violation occurred.
    attempted_operation:
        Name of the operation that was refused.
    """

    def __init__(self, current_state: "PrintFrameState", attempted_operation: str) -> None:
        self.current_state = current_state
        self.attempted_operation = attempted_operation
        super().__init__(
            f"PrintFrame ordering violation: attempted '{attempted_operation}' "
            f"while in state {current_state.name}. "
            f"Required state: see PrintFrame state machine documentation."
        )


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------


class PrintFrameState(Enum):
    """States of the :class:`PrintFrame` state machine.

    See the module docstring for the full state transition diagram.
    """

    AWAITING_FOLD = auto()
    """Initial state.  No correction may be applied; no print motion allowed."""

    FOLDED_READY = auto()
    """Entered after a confirmed fold (AA-19 fold token accepted).
    Correction may now be applied."""

    APPLIED_VERIFIED = auto()
    """Entered after a successful correction application and verification.
    Print motion requests (AA-34 interface) are allowed."""

    HALTED = auto()
    """Entered on a double sanity violation or a verification failure.
    All operations except inspection are refused."""


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass
class PrintFrameConfig:
    """Configuration for the :class:`PrintFrame` correction loop.

    All values are **unverified** defaults; calibrate against hardware.
    Set ``unverified: true`` in hardware.yaml until measured.

    Parameters
    ----------
    sanity_bound_z_mm:
        Maximum allowed |Z offset| (mm) from a single scan before triggering
        a rescan.  **unverified: true** — set to None for live mode to fail
        closed.
    sanity_bound_rx_rad:
        Maximum allowed |rx| tilt (radians).  **unverified: true**.
    sanity_bound_ry_rad:
        Maximum allowed |ry| tilt (radians).  **unverified: true**.
    residual_tolerance_mm:
        After applying the correction, the verification rescan must show a
        Z-offset residual no larger than this value (mm).  **unverified: true**.
    residual_tolerance_rx_rad:
        Verification residual tolerance for rx (radians).  **unverified: true**.
    residual_tolerance_ry_rad:
        Verification residual tolerance for ry (radians).  **unverified: true**.
    artifacts_subdir:
        Name of the sub-directory within the session artifact dir where per-hop
        records are written.  ``None`` → records written directly to artifact_dir.
    """

    # --- Sanity bounds (unverified: true) ---
    # None → live mode fails closed (sanity check always triggers rescan/halt)
    sanity_bound_z_mm: Optional[float] = 5.0             # unverified: true
    sanity_bound_rx_rad: Optional[float] = 0.1           # unverified: true (~5.7°)
    sanity_bound_ry_rad: Optional[float] = 0.1           # unverified: true (~5.7°)

    # --- Residual tolerances (unverified: true) ---
    residual_tolerance_mm: Optional[float] = 0.5         # unverified: true
    residual_tolerance_rx_rad: Optional[float] = 0.05    # unverified: true
    residual_tolerance_ry_rad: Optional[float] = 0.05    # unverified: true

    # --- Artifacts ---
    artifacts_subdir: Optional[str] = None


# ---------------------------------------------------------------------------
# Per-hop record
# ---------------------------------------------------------------------------


@dataclass
class HopCorrectionRecord:
    """Immutable record of one hop's leveling correction cycle.

    Written as a JSON-lines entry to the session artifacts directory.

    Attributes
    ----------
    hop_index:
        Zero-based hop counter.
    scan_id:
        Identifier of the primary scan (e.g. timestamp + random suffix).
    measured_z_mm, measured_rx_rad, measured_ry_rad:
        Raw correction from the primary :class:`~inchworm.leveling.LevelingResult`.
    applied_transform:
        The 4×4 HTM that was applied to the nominal frame (list of lists for
        JSON serialisability).
    rescan_id:
        Identifier of the verification rescan (or ``None`` if not performed,
        e.g. on halt before verification).
    residual_z_mm, residual_rx_rad, residual_ry_rad:
        Correction magnitude from the verification rescan
        (ideally near zero after the correction is applied).
        ``None`` when verification was not reached.
    outcome:
        ``"applied"`` — correction applied and verified.
        ``"sanity_rescan"`` — first scan exceeded sanity bound, rescan used.
        ``"halted_double_sanity"`` — both scans exceeded sanity bound.
        ``"halted_verification"`` — verification rescan residual exceeded tolerance.
    ts_iso:
        UTC timestamp of the record.
    residual_rms_scan:
        Plane-fit residual RMS of the primary scan (mm).
    residual_rms_verify:
        Plane-fit residual RMS of the verification scan (mm).  ``None`` if not reached.
    """

    hop_index: int
    scan_id: str
    measured_z_mm: float
    measured_rx_rad: float
    measured_ry_rad: float
    applied_transform: list  # 4×4 nested list
    rescan_id: Optional[str]
    residual_z_mm: Optional[float]
    residual_rx_rad: Optional[float]
    residual_ry_rad: Optional[float]
    outcome: str
    ts_iso: str
    residual_rms_scan: float
    residual_rms_verify: Optional[float]

    def to_dict(self) -> dict:
        return {
            "hop_index": self.hop_index,
            "scan_id": self.scan_id,
            "measured_z_mm": self.measured_z_mm,
            "measured_rx_rad": self.measured_rx_rad,
            "measured_ry_rad": self.measured_ry_rad,
            "applied_transform": self.applied_transform,
            "rescan_id": self.rescan_id,
            "residual_z_mm": self.residual_z_mm,
            "residual_rx_rad": self.residual_rx_rad,
            "residual_ry_rad": self.residual_ry_rad,
            "outcome": self.outcome,
            "ts_iso": self.ts_iso,
            "residual_rms_scan": self.residual_rms_scan,
            "residual_rms_verify": self.residual_rms_verify,
        }


# ---------------------------------------------------------------------------
# Core class
# ---------------------------------------------------------------------------


class PrintFrame:
    """DexArm print frame with per-hop leveling correction.

    Manages the corrected print frame for the DexArm, enforcing that:

    1. Corrections are only applied after a fold token (AA-19 ordering).
    2. Print motion is only allowed after a correction has been applied and
       verified (AA-34 ordering).
    3. Each hop's correction is an absolute replacement of the previous one
       relative to the nominal frame (no accumulation across hops).
    4. Corrections exceeding the sanity bound trigger a rescan; a second
       violation halts the system.
    5. After applying the correction, a verification rescan confirms the
       residual is within tolerance.

    Parameters
    ----------
    config:
        :class:`PrintFrameConfig` with bounds and tolerances.
    nominal_transform:
        4×4 HTM of the nominal (uncorrected) DexArm print frame.
        Defaults to the 4×4 identity (DexArm native frame).
    artifacts_dir:
        Directory for per-hop JSON-lines records.  ``None`` → records are
        logged but not written to disk.
    session_logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    """

    def __init__(
        self,
        config: Optional[PrintFrameConfig] = None,
        *,
        nominal_transform: Optional[Mat44] = None,
        artifacts_dir: Optional[Path] = None,
        session_logger: object = None,
    ) -> None:
        self._config = config if config is not None else PrintFrameConfig()
        self._nominal: Mat44 = (
            np.eye(4, dtype=np.float64)
            if nominal_transform is None
            else np.asarray(nominal_transform, dtype=np.float64).copy()
        )
        # Active correction transform — identity until first hop.
        # Updated by apply_correction; ALWAYS relative to nominal (AC#5).
        self._correction: Mat44 = np.eye(4, dtype=np.float64)
        self._state: PrintFrameState = PrintFrameState.AWAITING_FOLD
        self._artifacts_dir: Optional[Path] = artifacts_dir
        self._session_logger = session_logger
        self._hop_index: int = 0

    # ------------------------------------------------------------------
    # State inspection
    # ------------------------------------------------------------------

    @property
    def state(self) -> PrintFrameState:
        """Current state of the print frame state machine."""
        return self._state

    @property
    def hop_index(self) -> int:
        """Number of completed hops (incremented by reset_for_next_hop)."""
        return self._hop_index

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def accept_fold_token(self) -> None:
        """Signal that the arms have reached the folded transit pose (AA-19).

        This is the AA-19 seam: after a confirmed fold the orchestrator
        calls this method to allow the correction to be applied.

        Transitions ``AWAITING_FOLD`` → ``FOLDED_READY``.

        Raises
        ------
        PrintFrameOrderingError
            If not in ``AWAITING_FOLD`` state (e.g. called twice, or after halt).
        """
        if self._state is PrintFrameState.HALTED:
            raise PrintFrameOrderingError(self._state, "accept_fold_token")
        if self._state is not PrintFrameState.AWAITING_FOLD:
            raise PrintFrameOrderingError(self._state, "accept_fold_token")
        self._state = PrintFrameState.FOLDED_READY
        self._emit("INFO", "print_frame_folded_token_accepted", {
            "hop_index": self._hop_index,
            "new_state": self._state.name,
        })

    def reset_for_next_hop(self) -> None:
        """Reset state for the next hop.

        Clears the applied correction and transitions back to
        ``AWAITING_FOLD``.  The hop counter is incremented.

        Only callable from ``APPLIED_VERIFIED`` (normal path) or
        ``HALTED`` (operator reset after intervention).

        Raises
        ------
        PrintFrameOrderingError
            If called from ``AWAITING_FOLD`` or ``FOLDED_READY``
            (implies the previous hop's loop was never completed).
        """
        if self._state in (
            PrintFrameState.AWAITING_FOLD,
            PrintFrameState.FOLDED_READY,
        ):
            raise PrintFrameOrderingError(self._state, "reset_for_next_hop")
        self._correction = np.eye(4, dtype=np.float64)
        self._hop_index += 1
        self._state = PrintFrameState.AWAITING_FOLD
        self._emit("INFO", "print_frame_hop_reset", {
            "new_hop_index": self._hop_index,
            "new_state": self._state.name,
        })

    def require_applied_verified(self, operation: str = "print_motion") -> None:
        """Assert the frame is in ``APPLIED_VERIFIED`` state or raise.

        Call this at the top of every AA-34 print-motion function to enforce
        that the correction has been applied and verified before any move.

        Parameters
        ----------
        operation:
            Name of the operation being attempted (used in the error message).

        Raises
        ------
        PrintFrameOrderingError
            If not in ``APPLIED_VERIFIED`` state.
        """
        if self._state is not PrintFrameState.APPLIED_VERIFIED:
            raise PrintFrameOrderingError(self._state, operation)

    # ------------------------------------------------------------------
    # Correction application
    # ------------------------------------------------------------------

    def apply_correction(
        self,
        result: LevelingResult,
        rescan_fn: Callable[[], LevelingResult],
        *,
        dry_run: bool = False,
    ) -> Mat44:
        """Apply a leveling correction to the print frame.

        Steps:
        1. Verify the state machine is in ``FOLDED_READY``.
        2. Check the correction against the sanity bound (AC#2).
           - If within bound → proceed.
           - If exceeds bound → call ``rescan_fn()`` and check again.
             - If rescan also exceeds → :class:`FrameCorrectionHalt`
               (``reason="double_sanity_violation"``), state → ``HALTED``.
             - If rescan is within bound → use rescan result.
        3. Build the 4×4 correction transform from Z, rx, ry (AC#1).
        4. Apply to nominal frame: ``T_corrected = T_correction @ T_nominal``
           (AC#5 — absolute replacement).
        5. Call ``rescan_fn()`` for verification (AC#4).
           - If residual exceeds tolerance → :class:`FrameCorrectionHalt`
             (``reason="verification_failed"``), state → ``HALTED``.
        6. Write per-hop JSON-lines record (AC#6).
        7. Transition state to ``APPLIED_VERIFIED``.

        Parameters
        ----------
        result:
            :class:`~inchworm.leveling.LevelingResult` from AA-32.
        rescan_fn:
            Zero-argument callable that returns a new
            :class:`~inchworm.leveling.LevelingResult`.  Called for
            rescan (if sanity bound exceeded) and for verification.
            Must be injected; must not capture mutable state from this module.
        dry_run:
            When ``True``, skip live-mode config checks (``None`` bounds
            are treated as "no bound").  Default ``False``.

        Returns
        -------
        Mat44
            The corrected 4×4 HTM (``T_correction @ T_nominal``).

        Raises
        ------
        PrintFrameOrderingError
            If not in ``FOLDED_READY`` state.
        FrameCorrectionHalt
            On double sanity violation or verification failure.
        """
        if self._state is not PrintFrameState.FOLDED_READY:
            raise PrintFrameOrderingError(self._state, "apply_correction")

        scan_id_1 = _make_scan_id(result)
        self._emit("INFO", "print_frame_apply_start", {
            "hop_index": self._hop_index,
            "scan_id": scan_id_1,
            "z_offset_mm": round(result.z_offset_mm, 4),
            "rx_rad": round(result.rx_rad, 6),
            "ry_rad": round(result.ry_rad, 6),
        })

        # --- Step 2: sanity bound check ---
        active_result = result
        scan_id_active = scan_id_1
        rescan_id: Optional[str] = None
        outcome = "applied"

        if self._exceeds_sanity_bound(result, dry_run=dry_run):
            self._emit("WARNING", "print_frame_sanity_bound_exceeded", {
                "hop_index": self._hop_index,
                "scan_id": scan_id_1,
                "z_offset_mm": result.z_offset_mm,
                "rx_rad": result.rx_rad,
                "ry_rad": result.ry_rad,
            })
            log.warning(
                "AA-33 PrintFrame: sanity bound exceeded (z=%.3f mm, rx=%.5f rad, "
                "ry=%.5f rad); triggering rescan.",
                result.z_offset_mm, result.rx_rad, result.ry_rad,
            )
            # Rescan
            rescan_result = rescan_fn()
            rescan_id = _make_scan_id(rescan_result)
            outcome = "sanity_rescan"

            if self._exceeds_sanity_bound(rescan_result, dry_run=dry_run):
                self._emit("ERROR", "print_frame_double_sanity_violation", {
                    "hop_index": self._hop_index,
                    "scan_id_1": scan_id_1,
                    "scan_id_2": rescan_id,
                })
                log.error(
                    "AA-33 PrintFrame: rescan ALSO exceeded sanity bound "
                    "(z=%.3f mm); halting.",
                    rescan_result.z_offset_mm,
                )
                self._state = PrintFrameState.HALTED

                # Write record before raising
                record = HopCorrectionRecord(
                    hop_index=self._hop_index,
                    scan_id=scan_id_1,
                    measured_z_mm=result.z_offset_mm,
                    measured_rx_rad=result.rx_rad,
                    measured_ry_rad=result.ry_rad,
                    applied_transform=np.eye(4, dtype=np.float64).tolist(),
                    rescan_id=rescan_id,
                    residual_z_mm=rescan_result.z_offset_mm,
                    residual_rx_rad=rescan_result.rx_rad,
                    residual_ry_rad=rescan_result.ry_rad,
                    outcome="halted_double_sanity",
                    ts_iso=_utc_iso(),
                    residual_rms_scan=result.residual_rms_mm,
                    residual_rms_verify=rescan_result.residual_rms_mm,
                )
                self._write_record(record)

                raise FrameCorrectionHalt(
                    reason="double_sanity_violation",
                    detail=(
                        f"Both the initial scan (z={result.z_offset_mm:.3f} mm) and "
                        f"rescan (z={rescan_result.z_offset_mm:.3f} mm) exceeded the "
                        f"sanity bound."
                    ),
                    scan_id_1=scan_id_1,
                    scan_id_2=rescan_id,
                )

            # Rescan is within bound — use rescan result
            active_result = rescan_result
            scan_id_active = rescan_id

        # --- Step 3: build 4×4 correction transform ---
        T_correction = _build_correction_transform(active_result)

        # --- Step 4: apply to nominal (absolute replacement, AC#5) ---
        T_corrected = T_correction @ self._nominal

        # Store the correction transform (not the full corrected frame, for
        # clarity in reset — we store correction separately from nominal).
        self._correction = T_correction.copy()

        self._emit("INFO", "print_frame_correction_applied", {
            "hop_index": self._hop_index,
            "scan_id": scan_id_active,
            "z_mm": round(active_result.z_offset_mm, 4),
            "rx_rad": round(active_result.rx_rad, 6),
            "ry_rad": round(active_result.ry_rad, 6),
            "T_correction": T_correction.tolist(),
        })

        # --- Step 5: verification rescan ---
        verify_result = rescan_fn()
        verify_id = _make_scan_id(verify_result)

        # The residual is the magnitude of the correction that would still be
        # needed after the applied transform.  Since the scan measures the
        # absolute end-face pose in the *corrected* frame, the residual is
        # simply the reported z_offset_mm / rx_rad / ry_rad of the verify scan.
        residual_ok = self._residual_within_tolerance(verify_result, dry_run=dry_run)

        if not residual_ok:
            self._emit("ERROR", "print_frame_verification_failed", {
                "hop_index": self._hop_index,
                "scan_id": scan_id_active,
                "verify_id": verify_id,
                "residual_z_mm": verify_result.z_offset_mm,
                "residual_rx_rad": verify_result.rx_rad,
                "residual_ry_rad": verify_result.ry_rad,
            })
            log.error(
                "AA-33 PrintFrame: verification rescan residual too large "
                "(z=%.3f mm, rx=%.5f rad, ry=%.5f rad); halting.",
                verify_result.z_offset_mm, verify_result.rx_rad, verify_result.ry_rad,
            )
            self._state = PrintFrameState.HALTED

            record = HopCorrectionRecord(
                hop_index=self._hop_index,
                scan_id=scan_id_1,
                measured_z_mm=result.z_offset_mm,
                measured_rx_rad=result.rx_rad,
                measured_ry_rad=result.ry_rad,
                applied_transform=T_correction.tolist(),
                rescan_id=verify_id,
                residual_z_mm=verify_result.z_offset_mm,
                residual_rx_rad=verify_result.rx_rad,
                residual_ry_rad=verify_result.ry_rad,
                outcome="halted_verification",
                ts_iso=_utc_iso(),
                residual_rms_scan=result.residual_rms_mm,
                residual_rms_verify=verify_result.residual_rms_mm,
            )
            self._write_record(record)

            raise FrameCorrectionHalt(
                reason="verification_failed",
                detail=(
                    f"Verification rescan residual (z={verify_result.z_offset_mm:.3f} mm, "
                    f"rx={verify_result.rx_rad:.5f} rad, ry={verify_result.ry_rad:.5f} rad) "
                    f"exceeds tolerance."
                ),
                scan_id_1=scan_id_active,
                scan_id_2=verify_id,
            )

        # --- Step 6: write per-hop record ---
        record = HopCorrectionRecord(
            hop_index=self._hop_index,
            scan_id=scan_id_1,
            measured_z_mm=result.z_offset_mm,
            measured_rx_rad=result.rx_rad,
            measured_ry_rad=result.ry_rad,
            applied_transform=T_correction.tolist(),
            rescan_id=verify_id,
            residual_z_mm=verify_result.z_offset_mm,
            residual_rx_rad=verify_result.rx_rad,
            residual_ry_rad=verify_result.ry_rad,
            outcome=outcome,
            ts_iso=_utc_iso(),
            residual_rms_scan=result.residual_rms_mm,
            residual_rms_verify=verify_result.residual_rms_mm,
        )
        self._write_record(record)

        self._emit("INFO", "print_frame_verified", {
            "hop_index": self._hop_index,
            "verify_id": verify_id,
            "residual_z_mm": round(verify_result.z_offset_mm, 4),
            "residual_rx_rad": round(verify_result.rx_rad, 6),
            "residual_ry_rad": round(verify_result.ry_rad, 6),
        })

        # --- Step 7: transition to APPLIED_VERIFIED ---
        self._state = PrintFrameState.APPLIED_VERIFIED
        log.info(
            "AA-33 PrintFrame hop %d: correction applied and verified. "
            "State → APPLIED_VERIFIED.",
            self._hop_index,
        )
        return T_corrected

    # ------------------------------------------------------------------
    # Frame access
    # ------------------------------------------------------------------

    def get_corrected_transform(self) -> Mat44:
        """Return the current corrected 4×4 HTM.

        The corrected frame is ``T_correction @ T_nominal``.  Before any
        correction is applied the correction is the 4×4 identity, so the
        corrected frame equals the nominal frame.

        This method does **not** enforce state; use :meth:`require_applied_verified`
        in the AA-34 interface to assert the frame is ready before printing.

        Returns
        -------
        Mat44
            4×4 homogeneous transform of the corrected print frame.
        """
        return (self._correction @ self._nominal).copy()

    def get_nominal_transform(self) -> Mat44:
        """Return the nominal (uncorrected) 4×4 HTM."""
        return self._nominal.copy()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _exceeds_sanity_bound(
        self, result: LevelingResult, *, dry_run: bool
    ) -> bool:
        """Return True if any component of *result* exceeds the sanity bound.

        In live mode (``dry_run=False``), a ``None`` bound is treated as
        "zero tolerance" (always exceeds) to fail closed.  In dry-run mode,
        ``None`` bounds are treated as "no limit" (never exceeds).
        """
        cfg = self._config

        def _check(value: float, bound: Optional[float]) -> bool:
            if bound is None:
                return not dry_run  # live: fail closed; dry-run: pass
            return abs(value) > bound

        return (
            _check(result.z_offset_mm, cfg.sanity_bound_z_mm)
            or _check(result.rx_rad, cfg.sanity_bound_rx_rad)
            or _check(result.ry_rad, cfg.sanity_bound_ry_rad)
        )

    def _residual_within_tolerance(
        self, result: LevelingResult, *, dry_run: bool
    ) -> bool:
        """Return True if all residual components are within tolerance.

        In live mode, ``None`` tolerance is treated as zero tolerance
        (residual always fails) to fail closed.  In dry-run mode,
        ``None`` tolerances are treated as "always pass".
        """
        cfg = self._config

        def _check(value: float, tol: Optional[float]) -> bool:
            if tol is None:
                return dry_run  # live: fail closed; dry-run: pass
            return abs(value) <= tol

        return (
            _check(result.z_offset_mm, cfg.residual_tolerance_mm)
            and _check(result.rx_rad, cfg.residual_tolerance_rx_rad)
            and _check(result.ry_rad, cfg.residual_tolerance_ry_rad)
        )

    def _write_record(self, record: HopCorrectionRecord) -> None:
        """Write *record* as a JSON-lines entry to the artifacts directory."""
        record_dict = record.to_dict()
        # Always log at INFO level for telemetry
        self._emit("INFO", "print_frame_hop_record", record_dict)
        log.info("AA-33 PrintFrame hop %d record: %s", record.hop_index, record_dict)

        if self._artifacts_dir is None:
            return

        # Determine target directory
        target_dir = self._artifacts_dir
        if self._config.artifacts_subdir:
            target_dir = self._artifacts_dir / self._config.artifacts_subdir
        target_dir.mkdir(parents=True, exist_ok=True)

        record_path = target_dir / "leveling_corrections.jsonl"
        line = json.dumps(record_dict, separators=(",", ":")) + "\n"
        with open(record_path, "a", encoding="utf-8") as fh:
            fh.write(line)

        log.debug(
            "AA-33 PrintFrame: wrote hop %d record to %s",
            record.hop_index, record_path,
        )

    def _emit(self, severity: str, event_type: str, payload: dict) -> None:
        if self._session_logger is None:
            return
        try:
            self._session_logger.emit(
                role="dexarm",
                subsystem="print_frame",
                event_type=event_type,
                severity=severity,
                payload=payload,
            )
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Transform builder
# ---------------------------------------------------------------------------


def _build_correction_transform(result: LevelingResult) -> Mat44:
    """Build a 4×4 HTM from a :class:`~inchworm.leveling.LevelingResult`.

    The correction encodes:

    * Z translation: ``T[2, 3] = result.z_offset_mm``
    * Tilt about X (rx): ``Rx(result.rx_rad)``
    * Tilt about Y (ry): ``Ry(result.ry_rad)``

    Rotation order: Ry @ Rx (tilt about Y applied first, then about X).
    This matches the plane parameterisation ``Z = a*X + b*Y + c`` used in
    the leveling module:

      * ``ry = arctan(a)`` — the slope in X produces a rotation about Y.
      * ``rx = -arctan(b)`` — the slope in Y produces a rotation about X.

    The Z translation is applied as the last column (after rotation), so
    the corrected frame origin is at the measured end-face Z height.

    Parameters
    ----------
    result:
        :class:`~inchworm.leveling.LevelingResult` from the plane fit.

    Returns
    -------
    Mat44
        4×4 HTM encoding Z offset and rx/ry tilt.
    """
    rx = result.rx_rad
    ry = result.ry_rad
    z = result.z_offset_mm

    # Rx: rotation about X axis
    Rx = np.array([
        [1.0,       0.0,        0.0, 0.0],
        [0.0,  math.cos(rx), -math.sin(rx), 0.0],
        [0.0,  math.sin(rx),  math.cos(rx), 0.0],
        [0.0,       0.0,        0.0, 1.0],
    ], dtype=np.float64)

    # Ry: rotation about Y axis
    Ry = np.array([
        [ math.cos(ry), 0.0, math.sin(ry), 0.0],
        [        0.0,   1.0,        0.0,   0.0],
        [-math.sin(ry), 0.0, math.cos(ry), 0.0],
        [        0.0,   0.0,        0.0,   1.0],
    ], dtype=np.float64)

    # Combined rotation: Ry @ Rx (Y then X)
    T = Ry @ Rx

    # Z translation (last column)
    T[2, 3] = z

    return T


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_scan_id(result: LevelingResult) -> str:
    """Generate a short unique scan identifier from the result data.

    Uses the current timestamp (ISO format, ms precision) as the primary key.
    This is sufficient for correlating records within a session log.
    """
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")[:-3]
    return f"scan_{ts}"


def _utc_iso() -> str:
    """Current UTC time as ISO 8601 string."""
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# Config schema (appended to HardwareConfig in config.py under AA-33 markers)
# ---------------------------------------------------------------------------

# NOTE: The PrintFrameConfigSchema is defined here for co-location with
# the implementation.  It is imported and used in config.py's HardwareConfig.


class PrintFrameConfigSchema:
    """Placeholder — see inchworm.config.PrintFrameConfigSchema for the Pydantic model."""
