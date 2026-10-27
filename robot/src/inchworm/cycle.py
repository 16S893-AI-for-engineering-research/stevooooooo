"""AA-21: Dry crawl cycle runner.

Executes the full grip-advance-reapproach-relock-fold sequence for one or
more consecutive crawl cycles, **without a tube attached** and using a
scripted stub policy standing in for the real insertion.

Sequence per cycle (arm M = moving, arm H = holding)
------------------------------------------------------
1. **Preflight** — verify H is LOCKED (AA-17); check AA-16 calibration present,
   AA-9 limits verified, no UNKNOWN gripper states.  Refuse live start on any
   failing precondition (raise :class:`PreflightError` listing all failures).
   Dry-run is allowed with logged warnings.
2. **Unlock M** — AA-17 ``GripperLockSM.unlock()`` with the opposite-arm
   predicate wired to ``H.state == LOCKED``.
3. **Retract M** — AA-18 ``plan_approach`` / ``execute_approach`` with a
   nominal retract joint config (fallback: zero-q), dry-run by default.
4. **Approach M** — AA-18 ``plan_approach`` to the next lattice index from
   AA-15 ``FrameModel``.
5. **Supervisor handoff** — AA-20 ``PolicySupervisor`` with
   :class:`ScriptedSeatingDetector` injected; in dry-run use a scripted
   detector that fires success after a configured step count.
6. **Lock M** — AA-17 ``GripperLockSM.lock()`` (after supervisor SUCCESS).
7. **Advance bookkeeping** — ``FrameModel.complete_advance()`` called after M
   locks onto the new attachment point, immediately before fold.
   The holding arm H defines the baseline; M leapfrogs it.
   The advance direction is config-driven (``cycle_runner.advance_direction``);
   owner must confirm ``advance_direction_owner_confirmed: true`` in live mode.
8. **Swap** — M and H roles are exchanged for the next cycle.
9. **Fold** — AA-19 ``FoldSequencer.fold()`` (both arms).

Fault recovery paths
--------------------
* **Approach failure** (:class:`~inchworm.approach.ApproachUnreachableError`,
  :class:`~inchworm.approach.ApproachUnsafeError`):
  Retract M to safe pose, attempt to lock M (if still in APPROACH_READY /
  UNLOCKED, skip lock and leave M in that state with warning), fold, end cycle
  with :class:`CycleFaultError`.  Both grippers are asserted: H remains LOCKED,
  M is either LOCKED or in a documented intermediate state.
* **Supervisor abort/halt**
  (:attr:`~inchworm.supervisor.SupervisorOutcome.ABORT` /
  :attr:`~inchworm.supervisor.SupervisorOutcome.HALT`):
  M's gripper is commanded back to APPROACH_READY if possible, then locked.
  Arm is folded.  Cycle ends with :class:`CycleFaultError`.
* **Lock verification failure** (:class:`~inchworm.gripper_lock.LockVerificationFailed`):
  M's gripper enters FAULT.  Orchestrator asserts H is still LOCKED (raises
  :class:`SafetyInvariantViolation` if not), logs the fault, folds, ends with
  :class:`CycleFaultError`.

Safety invariants (AGENTS.md — enforced at every step)
-------------------------------------------------------
* Never both grippers unlocked while the tube is attached.  In this dry
  run (no tube) the invariant is maintained by construction:
  H is verified LOCKED before M is unlocked; M is only unlocked after H
  confirms LOCKED via the opposite-arm predicate.
* Gripper servo is only commanded through AA-17 ``GripperLockSM``.
* All arm motion goes through AA-9 ``GuardedArm`` / ``MotionGuard``
  (in tests: injected ``ArmExecutor`` mock).
* Dry-run is the default for all motion.

Advance arm documentation
--------------------------
Which arm physically advances the tube is determined by which arm **moves**
each cycle.  In cycle k, the moving arm M re-grips at the next lattice index
while H holds.  After M locks, ``FrameModel.complete_advance()`` records the
tube advancing by one pitch.  The holding arm H then becomes the moving arm
in cycle k+1, so both arms alternate advancing.  This is consistent with
doc-1 section 9's inchworm description ("one arm anchors while the other
leapfrogs").  **Owner confirmation required**: set
``cycle_runner.advance_direction_owner_confirmed: true`` after verifying the
physical crawl direction matches ``advance_direction`` in hardware.yaml.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np
import numpy.typing as npt

from inchworm.approach import (
    ApproachPlan,
    ApproachUnreachableError,
    ApproachUnsafeError,
    plan_approach,
    execute_approach,
)
from inchworm.config import (
    ApproachConfig,
    CycleRunnerConfig,
    HardwareConfig,
    PolicySupervisorConfig,
    SO101TrajectoryLimits,
)
from inchworm.events import (
    cycle_abort,
    cycle_end,
    cycle_start,
    motion_command,
)
from inchworm.frames import FrameModel
from inchworm.gripper_lock import (
    GripperLockSM,
    GripperState,
    LockVerificationFailed,
    LockVerdict,
)
from inchworm.kinematics import KinematicChain
from inchworm.motion_guard import MotionGuard
from inchworm.policy_contract import (
    JOINT_ORDER,
    N_JOINTS,
    Observation,
    PolicyAction,
)
from inchworm.stub_policy import StubPolicy
from inchworm.supervisor import (
    AbortReason,
    PolicySupervisor,
    SupervisorOutcome,
    SupervisorResult,
)
from inchworm.summarizer import SessionTimeline, summarize_session, format_timeline

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Phase labels (used for timeline events and dry-run output)
# ---------------------------------------------------------------------------

_PHASE_PREFLIGHT = "preflight"
_PHASE_UNLOCK = "unlock_moving"
_PHASE_RETRACT = "retract_moving"
_PHASE_APPROACH = "approach_moving"
_PHASE_SUPERVISOR = "supervisor_handoff"
_PHASE_LOCK = "lock_moving"
_PHASE_ADVANCE = "advance_bookkeeping"
_PHASE_FOLD = "fold"

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class PreflightError(RuntimeError):
    """Raised when one or more preflight conditions are not met.

    In live mode this always prevents the cycle from starting.
    In dry-run mode a warning is logged but the cycle proceeds.

    Attributes
    ----------
    failures:
        List of human-readable descriptions of every failing condition.
    """

    def __init__(self, failures: List[str]) -> None:
        self.failures = failures
        super().__init__(
            "AA-21 preflight failed — refusing to start live cycle:\n"
            + "\n".join(f"  • {f}" for f in failures)
        )


class CycleFaultError(RuntimeError):
    """Raised when a cycle terminates due to a recoverable fault.

    Recovery has been attempted (retract and fold) before this is raised.
    The robot is in a documented safe state.

    Attributes
    ----------
    phase:
        The cycle phase in which the fault occurred.
    reason:
        Human-readable reason string.
    safe:
        True if the robot is in a known safe state after recovery.
    """

    def __init__(self, phase: str, reason: str, safe: bool = True) -> None:
        self.phase = phase
        self.reason = reason
        self.safe = safe
        super().__init__(
            f"AA-21 cycle fault in phase '{phase}': {reason} "
            f"(safe={'yes' if safe else 'NO — operator intervention required'})"
        )


class SafetyInvariantViolation(RuntimeError):
    """Raised when the 'never both grippers unlocked' invariant would be violated.

    This is a hard stop: the cycle halts immediately without further motion.
    """

    def __init__(self, detail: str) -> None:
        self.detail = detail
        super().__init__(
            f"AA-21 SAFETY INVARIANT VIOLATION: {detail}. "
            "Both grippers would be unlocked simultaneously. Halting immediately."
        )


# ---------------------------------------------------------------------------
# Seating detector for dry-cycle use
# ---------------------------------------------------------------------------


class ScriptedSeatingDetector:
    """Scripted seating detector for dry-cycle use.

    Reports success after a configured number of policy steps.  In a real
    insertion (AA-27+) this is replaced by a camera-based detector.

    Parameters
    ----------
    success_at_step:
        Zero-based step index at which ``detect()`` returns True.  All
        earlier steps return False.
    """

    def __init__(self, success_at_step: int = 2) -> None:
        self._success_at_step = success_at_step
        self._call_count = 0

    def reset(self) -> None:
        """Reset the call counter (call at the start of each supervisor attempt)."""
        self._call_count = 0

    def detect(self) -> bool:
        """Return True when the call count reaches ``success_at_step``."""
        result = self._call_count >= self._success_at_step
        self._call_count += 1
        return result

    def __call__(self) -> bool:
        return self.detect()


# ---------------------------------------------------------------------------
# Per-cycle plan (dry-run output)
# ---------------------------------------------------------------------------


@dataclass
class CyclePhaseRecord:
    """Record for one phase of the planned/executed cycle.

    Attributes
    ----------
    phase:
        Phase label (one of the ``_PHASE_*`` constants).
    planned_duration_s:
        Estimated duration from trajectory planning (seconds).  None for
        non-motion phases.
    outcome:
        ``"ok"``, ``"skipped"``, ``"fault"``, or ``"planned"`` in dry-run.
    note:
        Optional detail string.
    """

    phase: str
    planned_duration_s: Optional[float] = None
    outcome: str = "planned"
    note: str = ""


@dataclass
class CyclePlan:
    """Complete plan for one dry-crawl cycle.

    Returned by :meth:`CycleRunner.plan_cycle` (dry-run only).
    Use ``.phases`` to inspect the planned sequence; ``.approach_plan`` for
    the full AA-18 plan object.
    """

    cycle_index: int
    moving_arm: str
    holding_arm: str
    target_lattice_index: int
    phases: List[CyclePhaseRecord] = field(default_factory=list)
    approach_plan: Optional[ApproachPlan] = None
    total_planned_duration_s: float = 0.0

    def summary(self) -> str:
        """Return a human-readable multi-line summary of the plan."""
        lines = [
            f"CyclePlan cycle={self.cycle_index}  "
            f"moving={self.moving_arm}  holding={self.holding_arm}  "
            f"target_index={self.target_lattice_index}",
            f"  total_planned_duration: {self.total_planned_duration_s:.3f} s",
        ]
        for ph in self.phases:
            dur = f"{ph.planned_duration_s:.3f}s" if ph.planned_duration_s is not None else "  ?    "
            lines.append(f"  [{ph.phase:<22s}] {dur:>8s}  {ph.outcome}  {ph.note}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Minimal mock executor for dry-run (no hardware)
# ---------------------------------------------------------------------------


class _DryRunExecutor:
    """No-op executor that logs but never sends servo commands."""

    def write_positions_rad(
        self,
        q: npt.NDArray[np.float64],
        bypass_guard: bool = False,
    ) -> None:
        _LOG.debug("dry-run executor: write_positions_rad %s", np.round(q, 4).tolist())

    def check_trajectory_rad(
        self,
        q_start: npt.NDArray[np.float64],
        q_end: npt.NDArray[np.float64],
    ) -> Optional[str]:
        return None


# ---------------------------------------------------------------------------
# CycleRunner
# ---------------------------------------------------------------------------


class CycleRunner:
    """AA-21 dry crawl cycle runner.

    Executes the full crawl cycle sequence (hold check, unlock moving arm,
    retract, coarse approach to next lattice index, stub-policy handoff,
    lock, advance bookkeeping, fold) without a tube attached.

    Parameters
    ----------
    frame_model:
        AA-15 :class:`~inchworm.frames.FrameModel` (shared mutable state).
    left_sm:
        AA-17 :class:`~inchworm.gripper_lock.GripperLockSM` for the left arm.
    right_sm:
        AA-17 :class:`~inchworm.gripper_lock.GripperLockSM` for the right arm.
    left_chain:
        Kinematic chain for the left arm.
    right_chain:
        Kinematic chain for the right arm.
    left_guard:
        :class:`~inchworm.motion_guard.MotionGuard` for the left arm.
    right_guard:
        :class:`~inchworm.motion_guard.MotionGuard` for the right arm.
    approach_cfg:
        AA-18 :class:`~inchworm.config.ApproachConfig`.
    traj_limits:
        :class:`~inchworm.config.SO101TrajectoryLimits` for time-parameterisation.
    supervisor_cfg:
        AA-20 :class:`~inchworm.config.PolicySupervisorConfig`.
    cycle_cfg:
        AA-21 :class:`~inchworm.config.CycleRunnerConfig`.  When ``None``,
        default values are used.
    left_executor:
        Optional injected motion executor for the left arm.  Defaults to a
        dry-run no-op executor.
    right_executor:
        Optional injected motion executor for the right arm.  Defaults to a
        dry-run no-op executor.
    session_logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    config:
        Optional full :class:`~inchworm.config.HardwareConfig` (used for
        preflight checks on calibration and limits).
    dry_run:
        When ``True`` (the default), no physical commands are sent.
    """

    def __init__(
        self,
        *,
        frame_model: FrameModel,
        left_sm: GripperLockSM,
        right_sm: GripperLockSM,
        left_chain: KinematicChain,
        right_chain: KinematicChain,
        left_guard: MotionGuard,
        right_guard: MotionGuard,
        approach_cfg: ApproachConfig,
        traj_limits: SO101TrajectoryLimits,
        supervisor_cfg: PolicySupervisorConfig,
        cycle_cfg: Optional[CycleRunnerConfig] = None,
        left_executor: Optional[Any] = None,
        right_executor: Optional[Any] = None,
        session_logger: Optional[Any] = None,
        config: Optional[HardwareConfig] = None,
        dry_run: bool = True,
        fold_sequencer: Optional[Any] = None,
    ) -> None:
        self._fm = frame_model
        self._left_sm = left_sm
        self._right_sm = right_sm
        self._left_chain = left_chain
        self._right_chain = right_chain
        self._left_guard = left_guard
        self._right_guard = right_guard
        self._approach_cfg = approach_cfg
        self._traj_limits = traj_limits
        self._supervisor_cfg = supervisor_cfg
        self._cycle_cfg = cycle_cfg or CycleRunnerConfig()
        self._session_logger = session_logger
        self._config = config
        self._dry_run = dry_run

        # Arm executors (default to dry-run no-ops)
        self._fold_sequencer = fold_sequencer  # AA-19, required for live
        self._left_executor = left_executor or _DryRunExecutor()
        self._right_executor = right_executor or _DryRunExecutor()

        # Moving/holding arm state — initialised from config, alternated each cycle
        initial = self._cycle_cfg.initial_moving_arm  # "left" or "right"
        self._moving_arm: str = initial
        self._holding_arm: str = "right" if initial == "left" else "left"

        # Zero q used as seed for IK (placeholder until retract pose is verified)
        self._zero_q = np.zeros(N_JOINTS, dtype=np.float64)

        # --- AA-28 ---
        # Optional AA-28 hook objects; injected by the caller after construction
        # or via set_aa28_hooks().  When None, the hooks are no-ops so the
        # cycle runs exactly as before.
        self._aa28_verifier: Optional[Any] = None     # TubeStateVerifier
        self._aa28_responder: Optional[Any] = None    # FaultResponder
        self._aa28_checkpoint: Optional[Any] = None   # CycleStateCheckpoint
        # --- AA-28 end ---

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def moving_arm(self) -> str:
        """Currently designated moving arm (``"left"`` or ``"right"``)."""
        return self._moving_arm

    @property
    def holding_arm(self) -> str:
        """Currently designated holding arm (``"left"`` or ``"right"``)."""
        return self._holding_arm

    @property
    def frame_model(self) -> FrameModel:
        """The shared AA-15 FrameModel (mutable state)."""
        return self._fm

    # --- AA-28 ---
    def set_aa28_hooks(
        self,
        *,
        verifier: Optional[Any] = None,
        responder: Optional[Any] = None,
        checkpoint: Optional[Any] = None,
    ) -> None:
        """Attach AA-28 recovery hooks (TubeStateVerifier, FaultResponder,
        CycleStateCheckpoint) to this runner.

        All three are optional; passing ``None`` clears that hook.  Hooks are
        called at the pre-cycle verification step (verifier), after phase
        transitions (checkpoint), and on fault detection (responder).

        Parameters
        ----------
        verifier:
            :class:`~inchworm.recovery.TubeStateVerifier` instance.
        responder:
            :class:`~inchworm.recovery.FaultResponder` instance.
        checkpoint:
            :class:`~inchworm.recovery.CycleStateCheckpoint` instance.
        """
        self._aa28_verifier = verifier
        self._aa28_responder = responder
        self._aa28_checkpoint = checkpoint
    # --- AA-28 end ---

    # ------------------------------------------------------------------
    # Public run methods
    # ------------------------------------------------------------------

    def run_cycles(self, n: int) -> List[CyclePlan]:
        """Run ``n`` consecutive dry crawl cycles.

        Parameters
        ----------
        n:
            Number of cycles to execute (≥ 1).

        Returns
        -------
        list of :class:`CyclePlan`
            One plan record per executed cycle (populated even on fault).

        Raises
        ------
        PreflightError
            If preflight fails in **live** mode.  In dry-run mode, preflight
            failures are logged as warnings and the cycle proceeds.
        SafetyInvariantViolation
            Immediately if both grippers would be unlocked simultaneously.
        """
        plans: List[CyclePlan] = []
        for cycle_index in range(n):
            plan = self.run_one_cycle(cycle_index)
            plans.append(plan)
        return plans

    def run_one_cycle(self, cycle_index: int = 0) -> CyclePlan:
        """Execute one complete dry crawl cycle and return its plan record.

        Parameters
        ----------
        cycle_index:
            Zero-based cycle index (for logging and bookkeeping).

        Returns
        -------
        :class:`CyclePlan`
            Populated plan record (all phases recorded).

        Raises
        ------
        PreflightError
            In live mode when preflight fails.
        SafetyInvariantViolation
            If the 'never both unlocked' invariant would be violated.
        CycleFaultError
            When a recoverable fault occurs (recovery already attempted).
        """
        # Compute next lattice index for the moving arm
        # Moving arm will leapfrog to: holding_index + 1
        # (or 0 if holding hasn't grabbed yet)
        h_idx = self._fm.lattice_state.left_index if self._holding_arm == "left" else self._fm.lattice_state.right_index
        if h_idx is None:
            h_idx = 0
        target_idx = h_idx + 1

        plan = CyclePlan(
            cycle_index=cycle_index,
            moving_arm=self._moving_arm,
            holding_arm=self._holding_arm,
            target_lattice_index=target_idx,
        )

        self._emit(**cycle_start(cycle_index=cycle_index))
        t_cycle_start = time.monotonic()

        try:
            self._run_cycle_body(cycle_index, target_idx, plan)
        except (PreflightError, SafetyInvariantViolation):
            raise
        except CycleFaultError:
            raise
        finally:
            self._emit(**cycle_end(cycle_index=cycle_index))
            elapsed = time.monotonic() - t_cycle_start
            plan.total_planned_duration_s = sum(
                ph.planned_duration_s for ph in plan.phases
                if ph.planned_duration_s is not None
            )
            _LOG.info(
                "AA-21 cycle %d complete in %.3f s (planned %.3f s)",
                cycle_index, elapsed, plan.total_planned_duration_s,
            )

        return plan

    def plan_cycle(self, cycle_index: int = 0) -> CyclePlan:
        """Plan one cycle without commanding any servos.

        Returns the full :class:`CyclePlan` including the AA-18 approach plan.
        Always dry-run; never commands hardware.
        """
        h_idx = self._fm.lattice_state.left_index if self._holding_arm == "left" else self._fm.lattice_state.right_index
        if h_idx is None:
            h_idx = 0
        target_idx = h_idx + 1

        plan = CyclePlan(
            cycle_index=cycle_index,
            moving_arm=self._moving_arm,
            holding_arm=self._holding_arm,
            target_lattice_index=target_idx,
        )

        # Plan the approach only (does not command servos)
        chain = self._left_chain if self._moving_arm == "left" else self._right_chain
        guard = self._left_guard if self._moving_arm == "left" else self._right_guard
        opposite_chain = self._right_chain if self._moving_arm == "left" else self._left_chain

        # Phase: preflight (always planned)
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_PREFLIGHT,
            planned_duration_s=0.0,
            outcome="planned",
            note="dry-run preflight check",
        ))

        # Phase: unlock (estimated ~0.05 s for servo travel)
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_UNLOCK,
            planned_duration_s=0.05,
            outcome="planned",
            note=f"unlock {self._moving_arm} arm",
        ))

        # Phase: approach (plan via AA-18)
        approach_plan: Optional[ApproachPlan] = None
        approach_duration_s = 0.0
        try:
            approach_plan = plan_approach(
                lattice_index=target_idx,
                arm=self._moving_arm,
                q_present=self._zero_q,
                frame_model=self._fm,
                chain=chain,
                guard=guard,
                approach_cfg=self._approach_cfg,
                traj_limits=self._traj_limits,
                opposite_arm_chain=opposite_chain,
                live=False,
                session_logger=self._session_logger,
            )
            approach_duration_s = approach_plan.total_duration_s
            plan.approach_plan = approach_plan
        except (ApproachUnreachableError, ApproachUnsafeError) as exc:
            _LOG.warning("plan_cycle: approach plan failed: %s", exc)

        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_RETRACT,
            planned_duration_s=approach_duration_s * 0.5,  # rough estimate
            outcome="planned",
            note="retract moving arm to safe pose",
        ))
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_APPROACH,
            planned_duration_s=approach_duration_s,
            outcome="planned",
            note=f"approach to lattice index {target_idx}",
        ))

        # Phase: supervisor (estimated)
        supervisor_duration_s = float(self._cycle_cfg.supervisor_success_at_step + 1) * 0.1
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_SUPERVISOR,
            planned_duration_s=supervisor_duration_s,
            outcome="planned",
            note=f"stub policy, success at step {self._cycle_cfg.supervisor_success_at_step}",
        ))

        # Phase: lock (~0.05 s)
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_LOCK,
            planned_duration_s=0.05,
            outcome="planned",
            note=f"lock {self._moving_arm} arm",
        ))

        # Phase: advance
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_ADVANCE,
            planned_duration_s=0.0,
            outcome="planned",
            note="FrameModel.complete_advance()",
        ))

        # Phase: fold (~0.5 s estimated, unverified)
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_FOLD,
            planned_duration_s=0.5,
            outcome="planned",
            note="fold both arms (unverified duration)",
        ))

        plan.total_planned_duration_s = sum(
            ph.planned_duration_s for ph in plan.phases
            if ph.planned_duration_s is not None
        )
        return plan

    # ------------------------------------------------------------------
    # Internal cycle body
    # ------------------------------------------------------------------

    def _run_cycle_body(
        self,
        cycle_index: int,
        target_idx: int,
        plan: CyclePlan,
    ) -> None:
        """Execute all phases of one cycle, populating ``plan.phases``."""
        moving = self._moving_arm
        holding = self._holding_arm
        moving_sm = self._left_sm if moving == "left" else self._right_sm
        holding_sm = self._left_sm if holding == "left" else self._right_sm

        # ── Phase 1: Preflight ──────────────────────────────────────────────
        failures = self._preflight_check(moving_sm, holding_sm)
        if failures:
            if not self._dry_run:
                plan.phases.append(CyclePhaseRecord(
                    phase=_PHASE_PREFLIGHT,
                    planned_duration_s=0.0,
                    outcome="fault",
                    note="; ".join(failures),
                ))
                raise PreflightError(failures)
            else:
                for f in failures:
                    _LOG.warning("AA-21 preflight warning (dry-run): %s", f)
                plan.phases.append(CyclePhaseRecord(
                    phase=_PHASE_PREFLIGHT,
                    planned_duration_s=0.0,
                    outcome="ok",
                    note=f"dry-run: {len(failures)} warning(s)",
                ))
        else:
            plan.phases.append(CyclePhaseRecord(
                phase=_PHASE_PREFLIGHT,
                planned_duration_s=0.0,
                outcome="ok",
            ))

        # --- AA-28 --- pre-cycle tube state verification
        if self._aa28_verifier is not None:
            v_result = self._aa28_verifier.verify(context="pre_cycle")
            if not v_result.passed and not self._dry_run:
                plan.phases.append(CyclePhaseRecord(
                    phase=_PHASE_PREFLIGHT,
                    planned_duration_s=0.0,
                    outcome="fault",
                    note="AA-28 pre-cycle verification failed: " + "; ".join(v_result.reasons),
                ))
                if self._aa28_responder is not None and v_result.reasons:
                    from inchworm.recovery import RecoveryFaultKind
                    self._aa28_responder.respond(
                        fault_kind=RecoveryFaultKind.LOCK_VERIFICATION_FAILURE,
                        detail="pre_cycle: " + "; ".join(v_result.reasons),
                        cycle_index=cycle_index,
                    )
                raise PreflightError(["AA-28 pre-cycle verification failed: " + r for r in v_result.reasons])
        if self._aa28_checkpoint is not None:
            from inchworm.recovery import CyclePhase
            self._aa28_checkpoint.save(
                cycle_index=cycle_index,
                phase=CyclePhase.PRE_CYCLE,
                frame_model=self._fm,
                left_sm=self._left_sm,
                right_sm=self._right_sm,
            )
        # --- AA-28 end ---

        # ── Phase 2: Unlock moving arm ──────────────────────────────────────
        # Safety gate: H must be LOCKED before M can be unlocked.
        # The opposite_arm_locked_fn is wired here to the holding SM's state.
        self._assert_holding_locked(holding_sm)

        self._log_phase(cycle_index, _PHASE_UNLOCK, {"moving_arm": moving})
        try:
            if moving_sm.state == GripperState.LOCKED:
                moving_sm.unlock()
            elif moving_sm.state == GripperState.UNKNOWN:
                # Reset from UNKNOWN to UNLOCKED; cannot verify it was locked.
                moving_sm.reset()
                _LOG.warning(
                    "AA-21: moving arm %s was UNKNOWN — reset to UNLOCKED (no unlock event)",
                    moving,
                )
        except Exception as exc:
            plan.phases.append(CyclePhaseRecord(
                phase=_PHASE_UNLOCK,
                planned_duration_s=0.05,
                outcome="fault",
                note=str(exc),
            ))
            self._emit(**cycle_abort(
                cycle_index=cycle_index,
                reason="unlock_failed",
                subsystem="cycle_runner",
                detail=str(exc),
            ))
            raise CycleFaultError(_PHASE_UNLOCK, str(exc)) from exc

        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_UNLOCK,
            planned_duration_s=0.05,
            outcome="ok",
        ))

        # ── Phase 3: Retract moving arm ─────────────────────────────────────
        # In dry-run: just log the retract phase; no servo command.
        # Use the supervisor retract pose if configured, else log a warning.
        retract_q = (
            np.array(self._supervisor_cfg.retract_pose_q_rad, dtype=np.float64)
            if self._supervisor_cfg.retract_pose_q_rad is not None
            else self._zero_q.copy()
        )
        self._log_phase(
            cycle_index, _PHASE_RETRACT,
            {"moving_arm": moving, "dry_run": self._dry_run},
        )
        if not self._dry_run:
            executor = self._left_executor if moving == "left" else self._right_executor
            executor.write_positions_rad(retract_q)
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_RETRACT,
            planned_duration_s=0.2,
            outcome="ok",
            note="retracted to safe pose",
        ))

        # Transition moving SM to APPROACH_READY (ready for lock after approach)
        if moving_sm.state == GripperState.UNLOCKED:
            moving_sm.set_approach_ready()

        # ── Phase 4: Approach moving arm ────────────────────────────────────
        chain = self._left_chain if moving == "left" else self._right_chain
        guard = self._left_guard if moving == "left" else self._right_guard
        opposite_chain = self._right_chain if moving == "left" else self._left_chain
        executor = self._left_executor if moving == "left" else self._right_executor

        self._log_phase(
            cycle_index, _PHASE_APPROACH,
            {"moving_arm": moving, "target_index": target_idx},
        )

        try:
            approach_plan = plan_approach(
                lattice_index=target_idx,
                arm=moving,
                q_present=self._zero_q,
                frame_model=self._fm,
                chain=chain,
                guard=guard,
                approach_cfg=self._approach_cfg,
                traj_limits=self._traj_limits,
                opposite_arm_chain=opposite_chain,
                live=not self._dry_run,
                session_logger=self._session_logger,
            )
            execute_approach(
                plan=approach_plan,
                executor=executor,
                chain=chain,
                approach_cfg=self._approach_cfg,
                live=not self._dry_run,
                session_logger=self._session_logger,
            )
        except (ApproachUnreachableError, ApproachUnsafeError) as exc:
            _LOG.error("AA-21 approach failed: %s", exc)
            plan.phases.append(CyclePhaseRecord(
                phase=_PHASE_APPROACH,
                planned_duration_s=None,
                outcome="fault",
                note=str(exc),
            ))
            # Recovery: assert H is still locked, attempt safe state for M
            self._assert_holding_locked(holding_sm)
            # M is in APPROACH_READY — leave it there (not locked, not tube-engaged)
            # and fold.
            self._recover_safe_halt(cycle_index, plan)
            self._emit(**cycle_abort(
                cycle_index=cycle_index,
                reason="approach_failure",
                subsystem="cycle_runner",
                detail=str(exc),
            ))
            raise CycleFaultError(
                _PHASE_APPROACH, str(exc), safe=True
            ) from exc

        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_APPROACH,
            planned_duration_s=approach_plan.total_duration_s,
            outcome="ok",
            note=f"target_index={target_idx}",
        ))

        # ── Phase 5: PolicySupervisor handoff ───────────────────────────────
        self._log_phase(cycle_index, _PHASE_SUPERVISOR, {"moving_arm": moving})

        seating_detector = ScriptedSeatingDetector(
            success_at_step=self._cycle_cfg.supervisor_success_at_step
        )
        stub = StubPolicy(noop=True)

        # Target attachment point in arm base frame (placeholder: use approach pose translation)
        T_preinsert = approach_plan.T_preinsert
        target_ap_m = T_preinsert[:3, 3].copy()

        # In dry-run, use a generous workspace radius so the scripted seating
        # detector can fire without IK/workspace concerns.
        # In live mode, the real supervisor_cfg.workspace_radius_m applies.
        sup_cfg = self._supervisor_cfg
        if self._dry_run and sup_cfg.workspace_radius_m < 1.0:
            from inchworm.config import PolicySupervisorConfig
            # Replace with a wide workspace so zero-q FK passes in dry-run.
            sup_cfg = PolicySupervisorConfig(
                max_steps=sup_cfg.max_steps,
                max_wall_time_s=sup_cfg.max_wall_time_s,
                workspace_radius_m=10.0,  # no workspace constraint in dry-run
                max_action_magnitude_deg=sup_cfg.max_action_magnitude_deg,
                clamp_on_magnitude_violation=sup_cfg.clamp_on_magnitude_violation,
                max_retries=sup_cfg.max_retries,
                retract_pose_q_rad=sup_cfg.retract_pose_q_rad,
                retract_pose_unverified=sup_cfg.retract_pose_unverified,
            )

        supervisor = PolicySupervisor(
            policy=stub,
            config=sup_cfg,
            chain=chain,
            target_attachment_point_m=target_ap_m,
            frame_source=_make_dry_frame_source(),
            motion_executor=_make_dry_motion_executor(),
            approach_callable=lambda: True,
            seating_detector=seating_detector,
            lock_callable=lambda: None,
            logger=self._session_logger,
            role=f"arm_{moving}",
            dry_run=True,  # always dry-run for the supervisor in a dry cycle
        )

        sup_result: SupervisorResult = supervisor.run(cycle_index=cycle_index)

        if sup_result.outcome not in (SupervisorOutcome.SUCCESS,):
            plan.phases.append(CyclePhaseRecord(
                phase=_PHASE_SUPERVISOR,
                planned_duration_s=sup_result.elapsed_s,
                outcome="fault",
                note=f"outcome={sup_result.outcome.name} reasons={sup_result.abort_reasons}",
            ))
            # Recovery: assert H locked, attempt to lock M, fold.
            self._assert_holding_locked(holding_sm)
            self._recover_safe_halt(cycle_index, plan)
            self._emit(**cycle_abort(
                cycle_index=cycle_index,
                reason="supervisor_abort",
                subsystem="cycle_runner",
                detail=sup_result.outcome.name,
            ))
            raise CycleFaultError(
                _PHASE_SUPERVISOR,
                f"supervisor {sup_result.outcome.name}: {sup_result.abort_reasons}",
                safe=True,
            )

        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_SUPERVISOR,
            planned_duration_s=sup_result.elapsed_s,
            outcome="ok",
            note=f"steps={sup_result.total_steps}",
        ))

        # ── Phase 6: Lock moving arm ─────────────────────────────────────────
        # Moving SM must be in APPROACH_READY to lock.
        self._log_phase(cycle_index, _PHASE_LOCK, {"moving_arm": moving})
        try:
            if moving_sm.state == GripperState.APPROACH_READY:
                verdict = moving_sm.lock()
            elif moving_sm.state == GripperState.LOCKED:
                verdict = LockVerdict.LATCHED  # already locked (should not happen in normal flow)
            else:
                raise CycleFaultError(
                    _PHASE_LOCK,
                    f"moving arm {moving} SM in unexpected state {moving_sm.state.name}",
                )
        except LockVerificationFailed as exc:
            _LOG.error("AA-21 lock verification failed: %s", exc)
            plan.phases.append(CyclePhaseRecord(
                phase=_PHASE_LOCK,
                planned_duration_s=0.05,
                outcome="fault",
                note=str(exc),
            ))
            # Recovery: assert H is LOCKED (halt if not)
            self._assert_holding_locked(holding_sm)
            self._recover_safe_halt(cycle_index, plan)
            self._emit(**cycle_abort(
                cycle_index=cycle_index,
                reason="lock_verification_failed",
                subsystem="cycle_runner",
                detail=str(exc),
            ))
            raise CycleFaultError(
                _PHASE_LOCK, str(exc), safe=True
            ) from exc

        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_LOCK,
            planned_duration_s=0.05,
            outcome="ok",
            note=f"verdict={getattr(verdict, 'name', str(verdict))}",
        ))

        # ── Phase 7: Advance bookkeeping ─────────────────────────────────────
        # M has locked onto the new attachment point → tube advanced by one pitch.
        # This is called AFTER M locks and BEFORE the fold.
        self._log_phase(cycle_index, _PHASE_ADVANCE, {"direction": self._cycle_cfg.advance_direction})
        self._fm.complete_advance(direction=self._cycle_cfg.advance_direction)
        self._fm.set_held_index(moving, target_idx)

        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_ADVANCE,
            planned_duration_s=0.0,
            outcome="ok",
            note=(
                f"advance_count={self._fm.tube_pose.advance_count} "
                f"direction={self._cycle_cfg.advance_direction}"
            ),
        ))

        # ── Phase 8: Swap roles ───────────────────────────────────────────────
        self._moving_arm, self._holding_arm = self._holding_arm, self._moving_arm

        # ── Phase 9: Fold ─────────────────────────────────────────────────────
        self._safe_fold_both(cycle_index, plan)

        # --- AA-28 --- post-cycle checkpoint and verification
        if self._aa28_checkpoint is not None:
            from inchworm.recovery import CyclePhase
            self._aa28_checkpoint.save(
                cycle_index=cycle_index,
                phase=CyclePhase.POST_CYCLE,
                frame_model=self._fm,
                left_sm=self._left_sm,
                right_sm=self._right_sm,
            )
        if self._aa28_verifier is not None:
            self._aa28_verifier.verify(context="post_cycle")
        # --- AA-28 end ---

    # ------------------------------------------------------------------
    # Preflight
    # ------------------------------------------------------------------

    def _preflight_check(
        self,
        moving_sm: GripperLockSM,
        holding_sm: GripperLockSM,
    ) -> List[str]:
        """Return a list of failing preflight conditions (empty = all OK).

        Checks:
        1. Holding gripper is in LOCKED state.
        2. Neither gripper is in UNKNOWN state.
        3. AA-16 calibration block is present (if config supplied).
        4. AA-9 motion guard limits are verified (if config supplied).
        5. advance_direction_owner_confirmed = True in live mode.
        """
        failures: List[str] = []

        # 1. Holding arm must be LOCKED
        if holding_sm.state != GripperState.LOCKED:
            failures.append(
                f"holding arm ({holding_sm.role}) is not LOCKED "
                f"(current state: {holding_sm.state.name})"
            )

        # 2. No UNKNOWN gripper states
        if moving_sm.state == GripperState.UNKNOWN:
            failures.append(
                f"moving arm ({moving_sm.role}) gripper state is UNKNOWN — "
                "run GripperLockSM.reset() before starting the cycle"
            )
        if holding_sm.state == GripperState.UNKNOWN:
            failures.append(
                f"holding arm ({holding_sm.role}) gripper state is UNKNOWN — "
                "run GripperLockSM.reset() before starting the cycle"
            )

        # 3. AA-16 calibration
        if self._config is not None:
            cal = self._config.calibration
            if cal is None:
                failures.append(
                    "AA-16 calibration block is absent from hardware.yaml. "
                    "Run calibration scripts before live use."
                )
            else:
                missing_cal = []
                if cal.left_arm_base is None:
                    missing_cal.append("left_arm_base")
                if cal.right_arm_base is None:
                    missing_cal.append("right_arm_base")
                if missing_cal:
                    failures.append(
                        f"AA-16 calibration missing for: {', '.join(missing_cal)}. "
                        "Run scripts/calibrate_arm_base.py --live."
                    )

        # 4. AA-9 motion guard limits verified
        if self._config is not None:
            mg = self._config.motion_guard
            if mg is None:
                failures.append(
                    "AA-9 motion_guard block is absent from hardware.yaml. "
                    "Run scripts/measure_limits.py --live."
                )
            else:
                # joint_limits_verified lives on HomingConfig (AA-10), not MotionGuardConfig.
                # Check whether any resolved limits are marked unverified via the guard.
                from inchworm.motion_guard import _resolve_limits_for_arm
                left_limits = _resolve_limits_for_arm(
                    mg.left_arm_limits if hasattr(mg, "left_arm_limits") else None,
                    self._config.so101_kinematics,
                    "left",
                )
                if not left_limits.live_ok:
                    failures.append(
                        "AA-9 left-arm joint limits are unverified "
                        "(null or unverified=True). "
                        "Run scripts/measure_limits.py --live."
                    )

        # 5. advance_direction must be owner-confirmed in live mode
        if not self._dry_run and not self._cycle_cfg.advance_direction_owner_confirmed:
            failures.append(
                "advance_direction_owner_confirmed is false. "
                "Confirm advance_direction in hardware.yaml, "
                "then set advance_direction_owner_confirmed: true before live use."
            )

        # 6. Physical advance sense (owner decision 2026-10-27: away from the
        #    DexArm).  Live only; fail closed when it cannot be checked.
        if not self._dry_run:
            failures.extend(self._advance_physical_failures())

        return failures

    def _advance_physical_failures(self) -> List[str]:
        """Check that ``advance_direction`` moves the tube the owner's way.

        Uses the AA-16 calibration: the advance vector in the chassis frame is
        ``advance_direction * tube_axis.axis_direction``.  "away_from_dexarm"
        means it points from the DexArm base toward the midpoint of the two
        arm bases (all three sit in line, doc-1 s2).  Any missing input is a
        failure (fail closed).
        """
        want = self._cycle_cfg.advance_physical
        if want is None:
            return [
                "cycle_runner.advance_physical is not set; the owner must state "
                "which way the tube advances (away_from_dexarm / toward_dexarm)."
            ]
        cal = self._config.calibration if self._config is not None else None
        missing: List[str] = []
        axis = None
        if cal is None or cal.tube_axis is None or cal.tube_axis.axis_direction is None:
            missing.append("tube_axis")
        else:
            axis = np.asarray(cal.tube_axis.axis_direction, dtype=np.float64)
        positions: Dict[str, np.ndarray] = {}
        for name in ("dexarm_base", "left_arm_base", "right_arm_base"):
            block = getattr(cal, name, None) if cal is not None else None
            if block is None or block.transform is None:
                missing.append(name)
            else:
                positions[name] = np.asarray(block.transform, dtype=np.float64)[:3, 3]
        if missing:
            return [
                "cannot verify the physical advance direction: AA-16 calibration "
                f"missing for {', '.join(missing)}."
            ]
        assert axis is not None
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            return ["calibration.tube_axis.axis_direction is a zero vector."]
        advance = self._cycle_cfg.advance_direction * axis / norm
        away = 0.5 * (positions["left_arm_base"] + positions["right_arm_base"]) - positions["dexarm_base"]
        away_norm = float(np.linalg.norm(away))
        if away_norm < 1e-9:
            return ["DexArm base and arm-base midpoint coincide; cannot orient the advance."]
        cos = float(np.dot(advance, away / away_norm))
        # Require a clear sense, not a near-perpendicular axis (|cos| >= 0.5).
        if abs(cos) < 0.5:
            return [
                f"tube axis is nearly perpendicular to the DexArm-to-arms line "
                f"(cos={cos:.3f}); check the AA-16 calibration."
            ]
        got = "away_from_dexarm" if cos > 0 else "toward_dexarm"
        if got != want:
            return [
                f"advance_direction={self._cycle_cfg.advance_direction} moves the tube "
                f"{got} (cos={cos:.3f}) but the owner decision is {want}; "
                "flip cycle_runner.advance_direction."
            ]
        return []

    # ------------------------------------------------------------------
    # Safety helpers
    # ------------------------------------------------------------------

    def _assert_holding_locked(self, holding_sm: GripperLockSM) -> None:
        """Raise :class:`SafetyInvariantViolation` if H is not LOCKED."""
        if holding_sm.state != GripperState.LOCKED:
            raise SafetyInvariantViolation(
                f"holding arm ({holding_sm.role}) is in state "
                f"{holding_sm.state.name} — not LOCKED"
            )

    def _safe_fold_both(self, cycle_index: int, plan: CyclePlan) -> None:
        """Fold both arms at the END of a successful cycle (both LOCKED).

        Orchestrator fix 2026-10-24: the earlier version recorded "fold
        executed" in live mode without folding anything, and was also called on
        fault paths where the moving arm is UNLOCKED (AA-19 forbids folding
        unless both grippers are LOCKED). Live mode now requires an injected
        ``fold_sequencer`` (AA-19 FoldSequencer-like, with ``.fold(dry_run=...)``).
        """
        self._log_phase(cycle_index, _PHASE_FOLD, {"dry_run": self._dry_run})
        if not self._dry_run:
            if self._fold_sequencer is None:
                raise CycleFaultError(_PHASE_FOLD, "no fold_sequencer injected for live fold", safe=True)
            self._fold_sequencer.fold(dry_run=False)
        plan.phases.append(CyclePhaseRecord(
            phase=_PHASE_FOLD,
            planned_duration_s=0.5,
            outcome="ok",
            note="fold both arms (dry-run: planned only)" if self._dry_run else "fold executed via FoldSequencer",
        ))

    def _recover_safe_halt(self, cycle_index: int, plan: CyclePlan) -> None:
        """Fault recovery: do NOT fold (the moving arm is not locked). Keep the
        holding arm LOCKED (already asserted by the caller), leave the moving
        arm where the failed phase left it (the AA-18/AA-20 layers retract or
        retreat before raising), record a safe-halt phase, and let the caller
        raise CycleFaultError. The operator or a later recovery task (AA-28)
        decides how to resume."""
        self._log_phase(cycle_index, "safe_halt", {"dry_run": self._dry_run})
        plan.phases.append(CyclePhaseRecord(
            phase="safe_halt",
            planned_duration_s=0.0,
            outcome="halted",
            note="fault recovery: holding arm LOCKED, moving arm not folded",
        ))

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _log_phase(
        self,
        cycle_index: int,
        phase: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Emit a structured phase-transition event via AA-7."""
        if self._session_logger is not None:
            emit = getattr(self._session_logger, "emit", None)
            if emit is not None:
                emit(
                    role="orchestrator",
                    subsystem="cycle_runner",
                    event_type="cycle_phase",
                    severity="INFO",
                    payload={
                        "cycle_index": cycle_index,
                        "phase": phase,
                        **(payload or {}),
                    },
                )
        else:
            _LOG.info(
                "AA-21 cycle %d phase '%s': %s",
                cycle_index, phase, payload or {},
            )

    def _emit(self, **kwargs: Any) -> None:
        if self._session_logger is not None:
            emit = getattr(self._session_logger, "emit", None)
            if emit is not None:
                try:
                    emit(**kwargs)
                except Exception:  # noqa: BLE001
                    pass
        else:
            _LOG.debug("AA-21 emit: %s", kwargs.get("event_type", "?"))


# ---------------------------------------------------------------------------
# Helpers for supervisor dry-run injection
# ---------------------------------------------------------------------------


def _make_dry_frame_source():
    """Return a frame source callable that yields synthetic data.

    Returns zeros for joint positions and a black frame — sufficient for
    the scripted stub policy and the workspace-bound check.
    """
    _frame = np.zeros((240, 320), dtype=np.uint8)
    _joints = np.zeros(N_JOINTS, dtype=np.float32)

    def _source():
        return _frame, _joints

    return _source


def _make_dry_motion_executor():
    """Return a motion executor callable that logs but never moves."""

    def _exec(action: Dict[str, float]) -> None:
        _LOG.debug("dry motion executor: action=%s", action)

    return _exec


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def build_cycle_runner(
    cfg: HardwareConfig,
    *,
    dry_run: bool = True,
    session_logger: Optional[Any] = None,
) -> "CycleRunner":
    """Build a :class:`CycleRunner` from a loaded :class:`HardwareConfig`.

    Parameters
    ----------
    cfg:
        Loaded hardware config.
    dry_run:
        ``True`` (default) = no physical commands.
    session_logger:
        Optional AA-7 session logger.

    Raises
    ------
    ValueError
        If required config blocks (approach, policy_supervisor, so101_kinematics,
        so101_kinematics.trajectory) are missing.
    """
    from inchworm.kinematics import build_chain
    from inchworm.motion_guard import build_motion_guard
    from inchworm.backends import MockSerialTransport
    from inchworm.servo_bus import ArmServoBus

    approach_cfg = cfg.approach
    if approach_cfg is None:
        raise ValueError(
            "build_cycle_runner: approach block is absent from hardware.yaml. "
            "Add an approach: block before using the cycle runner."
        )

    supervisor_cfg = cfg.policy_supervisor
    if supervisor_cfg is None:
        supervisor_cfg = PolicySupervisorConfig()

    cycle_cfg = cfg.cycle_runner
    traj_limits = cfg.so101_kinematics.trajectory

    left_chain = build_chain(cfg.so101_kinematics)
    right_chain = build_chain(cfg.so101_kinematics)
    left_guard = build_motion_guard(cfg, "left")
    right_guard = build_motion_guard(cfg, "right")

    # Build FrameModel from config
    frame_model = FrameModel.from_config(cfg, left_chain, right_chain)

    # Build gripper SMs using mock buses (dry-run)
    left_bus = _build_mock_bus(cfg, "arm_left")
    right_bus = _build_mock_bus(cfg, "arm_right")

    from inchworm.gripper_lock import GripperLockSM

    left_sm = GripperLockSM(
        bus=left_bus,
        role="arm_left",
        config=cfg.gripper_lock,
        opposite_arm_locked_fn=None,  # wired below
        dry_run=dry_run,
    )
    right_sm = GripperLockSM(
        bus=right_bus,
        role="arm_right",
        config=cfg.gripper_lock,
        opposite_arm_locked_fn=None,  # wired below
        dry_run=dry_run,
    )

    # Wire opposite-arm predicates
    left_sm._opposite_arm_locked_fn = lambda: right_sm.state == GripperState.LOCKED
    right_sm._opposite_arm_locked_fn = lambda: left_sm.state == GripperState.LOCKED

    return CycleRunner(
        frame_model=frame_model,
        left_sm=left_sm,
        right_sm=right_sm,
        left_chain=left_chain,
        right_chain=right_chain,
        left_guard=left_guard,
        right_guard=right_guard,
        approach_cfg=approach_cfg,
        traj_limits=traj_limits,
        supervisor_cfg=supervisor_cfg,
        cycle_cfg=cycle_cfg,
        session_logger=session_logger,
        config=cfg,
        dry_run=dry_run,
    )


def _build_mock_bus(cfg: HardwareConfig, role: str) -> "ArmServoBus":
    """Build a mock-backed ArmServoBus for dry-run mode."""
    from inchworm.backends import MockSerialTransport
    from inchworm.servo_bus import ArmServoBus

    arm_cfg = cfg.arms.left if role == "arm_left" else cfg.arms.right
    sj = arm_cfg.servo_joints
    # Use config servo IDs where present; fall back to conventional defaults.
    _DEFAULTS = {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }
    joint_map = {
        k: (getattr(sj, k) if getattr(sj, k) is not None else v)
        for k, v in _DEFAULTS.items()
    }
    transport = MockSerialTransport(role)
    return ArmServoBus(transport, role=role, joint_id_map=joint_map)
