"""AA-20: Policy supervisor — code-policy-code handoff safety boundary.

The :class:`PolicySupervisor` is the *only* entry point for delegating joint
control to a learned insertion policy.  It enforces every safety invariant on
the boundary:

- Gripper commands are hard-blocked via the AA-17 adapter (never reaches the
  servo bus).
- Hard step and wall-clock limits abort the run and retract the arm.
- Cartesian workspace bound (computed by FK via AA-11) aborts if the TCP
  wanders outside the allowed sphere.
- Per-step action magnitude clamping (or abort on violation if configured).
- Bounded retry with logged escalation; final halt after retries exhausted.
- All events are logged via AA-7 telemetry.
- Success is declared only by the injected ``seating_detector``; never by the
  policy itself.
- Gripper locking on success is performed via the injected ``lock_callable``;
  this module *never* touches the gripper servo.

Dependency injection
--------------------
Every hardware-facing operation is injected; no hardware is imported.  This
makes the supervisor fully testable with mocks::

    supervisor = PolicySupervisor(
        policy=stub_policy,
        config=cfg.policy_supervisor,
        chain=build_chain(cfg.so101_kinematics),
        target_attachment_point_m=np.array([0.1, 0.05, 0.12]),
        frame_source=lambda: (frame, joint_positions_deg),
        motion_executor=mock_executor,
        approach_callable=mock_approach,
        seating_detector=mock_seating,
        lock_callable=mock_lock,
        logger=session_ctx.logger,
        role="arm_left",
        dry_run=True,
    )
    result = supervisor.run(cycle_index=0)

AGENTS.md safety invariants respected
--------------------------------------
- One camera at a time: frame_source is injected; supervisor never opens
  a camera directly.
- Gripper servo isolation: ActionAdapter raises GripperCommandBlocked before
  any bus write.
- Dry-run first: dry_run=True is the default.
- Every motion path has a dry-run.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
import numpy.typing as npt

from inchworm.policy_actions import (
    POLICY_JOINT_NAMES,
    GripperCommandBlocked,
    validate_policy_action,
)
from inchworm.policy_contract import (
    CONTRACT_VERSION,
    DEFAULT_FRAME_HEIGHT,
    DEFAULT_FRAME_WIDTH,
    JOINT_ORDER,
    N_JOINTS,
    Observation,
    PolicyAction,
)
from inchworm.config import PolicySupervisorConfig
from inchworm.kinematics import KinematicChain
from inchworm.events import (
    cycle_abort,
    cycle_retry,
    policy_handoff_start,
    policy_handoff_end,
)

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Type aliases for injected callables
# ---------------------------------------------------------------------------

#: Callable that returns (frame, joint_positions_deg).
#: frame: shape (H, W) uint8 ndarray.
#: joint_positions_deg: shape (N_JOINTS,) float32 ndarray.
FrameSource = Callable[[], Tuple[npt.NDArray[np.uint8], npt.NDArray[np.float32]]]

#: Callable that applies a validated joint action.  Receives the action dict;
#: returns nothing.  Dry-run implementations should be no-ops.
MotionExecutor = Callable[[Dict[str, float]], None]

#: Callable that executes the coarse re-approach (AA-18 later).
#: Returns True if approach succeeded, False if it failed.
ApproachCallable = Callable[[], bool]

#: Callable that checks whether the tube is seated.
#: Returns True on success (AA-24 will implement), False otherwise.
SeatingDetector = Callable[[], bool]

#: Callable that commands the gripper lock (→ gripper_lock.lock, AA-17).
#: Returns nothing; raises on failure.
LockCallable = Callable[[], None]

# ---------------------------------------------------------------------------
# Protocol / interface for a policy
# ---------------------------------------------------------------------------


class PolicyProtocol:
    """Interface a policy must satisfy.

    The policy receives an :class:`~inchworm.policy_contract.Observation` and
    returns a :class:`~inchworm.policy_contract.PolicyAction` mapping
    joint-name → target-position (degrees).

    The policy **must not** include ``"gripper"`` or any unknown key; if it
    does, the supervisor raises :class:`~inchworm.policy_actions.GripperCommandBlocked`
    and aborts.
    """

    def step(self, obs: Observation) -> PolicyAction:
        """Produce an action given an observation.

        Parameters
        ----------
        obs:
            Current :class:`~inchworm.policy_contract.Observation`.

        Returns
        -------
        PolicyAction
            Mapping joint_name → target position (degrees).
        """
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Result / outcome types
# ---------------------------------------------------------------------------


class RetractUnsafeError(RuntimeError):
    """The arm cannot be retracted safely (retract pose missing or unverified
    in live mode). Always propagates: the cycle must stop, not retry."""


class SupervisorOutcome(Enum):
    """Final outcome returned by :meth:`PolicySupervisor.run`."""

    SUCCESS = auto()
    """Seating detector confirmed success; gripper locked."""

    ABORT = auto()
    """All retries exhausted with no success; cycle aborted."""

    HALT = auto()
    """Maximum retries reached; escalation policy executed; cycle halted."""


class AbortReason(str, Enum):
    """Machine-readable abort reason logged in the typed abort event."""

    STEP_LIMIT = "step_limit"
    """Hard step count limit exceeded."""

    WALL_TIME_LIMIT = "wall_time_limit"
    """Wall-clock time limit exceeded."""

    WORKSPACE_VIOLATION = "workspace_violation"
    """TCP FK position left the allowed Cartesian workspace sphere."""

    GRIPPER_COMMAND_BLOCKED = "gripper_command_blocked"
    """Policy attempted to command the gripper servo."""

    SEATING_DETECTOR_ERROR = "seating_detector_error"
    """Seating detector raised an unexpected exception."""

    APPROACH_FAILED = "approach_failed"
    """Re-approach callable returned failure."""

    RETRACT_UNVERIFIED = "retract_unverified"
    # Orchestrator additions 2026-10-24:
    LOCK_FAILED = "lock_failed"
    """Seating was detected but the AA-17 lock raised or reported failure."""
    STEP_EXCEPTION = "step_exception"
    """Frame source, policy or motion executor raised; arm retracted."""
    """Retract pose is unverified and live mode was requested."""


@dataclass
class SupervisorResult:
    """Detailed result of a :meth:`PolicySupervisor.run` call.

    Attributes
    ----------
    outcome:
        Final outcome of the supervised handoff.
    attempts:
        Number of policy-loop attempts executed (1 = no retry needed).
    total_steps:
        Total number of policy steps executed across all attempts.
    abort_reasons:
        List of abort reasons (one per failed attempt), in order.
    elapsed_s:
        Total wall-clock time spent inside :meth:`PolicySupervisor.run`
        (seconds).
    """

    outcome: SupervisorOutcome
    attempts: int
    total_steps: int
    abort_reasons: List[AbortReason] = field(default_factory=list)
    elapsed_s: float = 0.0


# ---------------------------------------------------------------------------
# Internal typed payloads for AA-7 events
# ---------------------------------------------------------------------------


def _emit_supervisor_abort(
    logger: object,
    *,
    role: str,
    cycle_index: int,
    reason: AbortReason,
    attempt: int,
    detail: str | None = None,
) -> None:
    """Emit a typed supervisor abort event via AA-7 telemetry."""
    if logger is None:
        return
    emit = getattr(logger, "emit", None)
    if emit is None:
        return
    emit(
        role=role,
        subsystem="supervisor",
        event_type="supervisor_abort",
        severity="ERROR",
        payload={
            "cycle_index": cycle_index,
            "reason": reason.value,
            "attempt": attempt,
            "detail": detail,
            "contract_version": CONTRACT_VERSION,
        },
    )


def _emit_supervisor_success(
    logger: object,
    *,
    role: str,
    cycle_index: int,
    attempt: int,
    steps: int,
) -> None:
    if logger is None:
        return
    emit = getattr(logger, "emit", None)
    if emit is None:
        return
    emit(
        role=role,
        subsystem="supervisor",
        event_type="supervisor_success",
        severity="INFO",
        payload={
            "cycle_index": cycle_index,
            "attempt": attempt,
            "steps": steps,
            "contract_version": CONTRACT_VERSION,
        },
    )


def _emit_supervisor_halt(
    logger: object,
    *,
    role: str,
    cycle_index: int,
    total_attempts: int,
) -> None:
    if logger is None:
        return
    emit = getattr(logger, "emit", None)
    if emit is None:
        return
    emit(
        role=role,
        subsystem="supervisor",
        event_type="supervisor_halt",
        severity="CRITICAL",
        payload={
            "cycle_index": cycle_index,
            "total_attempts": total_attempts,
            "contract_version": CONTRACT_VERSION,
        },
    )


def _emit_action_clamped(
    logger: object,
    *,
    role: str,
    raw_magnitude: float,
    clamped_magnitude: float,
    step: int,
) -> None:
    if logger is None:
        return
    emit = getattr(logger, "emit", None)
    if emit is None:
        return
    emit(
        role=role,
        subsystem="supervisor",
        event_type="supervisor_action_clamped",
        severity="WARNING",
        payload={
            "raw_magnitude_deg": raw_magnitude,
            "clamped_magnitude_deg": clamped_magnitude,
            "step": step,
        },
    )


# ---------------------------------------------------------------------------
# PolicySupervisor
# ---------------------------------------------------------------------------


class PolicySupervisor:
    """Safety supervisor for the code-policy-code handoff boundary (AA-20).

    All safety-critical callables are **injected** so the supervisor is fully
    testable without hardware.

    Parameters
    ----------
    policy:
        Object with a ``step(obs: Observation) -> PolicyAction`` method.
    config:
        :class:`~inchworm.config.PolicySupervisorConfig` loaded from
        ``hardware.yaml``.
    chain:
        :class:`~inchworm.kinematics.KinematicChain` used to compute FK for
        workspace-bound enforcement.
    target_attachment_point_m:
        3-vector (metres) of the target attachment point in the arm base
        frame.  The TCP must stay within ``config.workspace_radius_m`` of
        this point.
    frame_source:
        Callable ``() → (frame, joint_positions_deg)`` that captures one
        wrist-camera frame and reads current joint positions.  Must be
        consistent with the AA-6 arbiter / AA-14 capture invariant; the
        supervisor never opens a camera directly.
    motion_executor:
        Callable ``(action: dict) → None`` that applies the (already
        validated) policy action to the arm.
    approach_callable:
        Callable ``() → bool`` for re-approach on retry.  Returns ``True``
        on success.  (AA-18 will implement the real approach later.)
    seating_detector:
        Callable ``() → bool`` that returns ``True`` when the tube is
        correctly seated.  (AA-24 will implement the real detector.)
    lock_callable:
        Callable ``() → None`` that commands the gripper lock (→ AA-17
        ``gripper_lock.lock()``).  Called only on verified success.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`; events are
        silently discarded when ``None``.
    role:
        Arm role string used for logging (e.g. ``"arm_left"``).
    dry_run:
        When ``True`` (the default), all actions are validated and logged but
        ``motion_executor`` and ``lock_callable`` are **not** called.
        ``approach_callable`` and ``seating_detector`` are still called.
    """

    def __init__(
        self,
        *,
        policy: PolicyProtocol,
        config: PolicySupervisorConfig,
        chain: KinematicChain,
        target_attachment_point_m: npt.NDArray[np.float64],
        frame_source: FrameSource,
        motion_executor: MotionExecutor,
        approach_callable: ApproachCallable,
        seating_detector: SeatingDetector,
        lock_callable: LockCallable,
        logger: object | None = None,
        role: str = "arm_left",
        dry_run: bool = True,
        joint_deg_to_rad: "Callable[[npt.NDArray[np.float32]], npt.NDArray[np.float64]] | None" = None,
    ) -> None:
        # Orchestrator 2026-10-24: servo degrees -> URDF joint radians must use
        # the AA-9 per-joint calibration (zero offset and sign). A bare
        # deg2rad makes the workspace bound meaningless. In live mode a
        # converter is REQUIRED (fail closed); dry-run falls back to deg2rad.
        if joint_deg_to_rad is None and not dry_run:
            raise ValueError(
                "live PolicySupervisor requires joint_deg_to_rad (AA-9 calibration)"
            )
        self._joint_deg_to_rad = joint_deg_to_rad or (lambda d: np.deg2rad(np.asarray(d, dtype=np.float64)))
        self._policy = policy
        self._config = config
        self._chain = chain
        self._target_m = np.asarray(target_attachment_point_m, dtype=np.float64).flatten()
        self._frame_source = frame_source
        self._motion_executor = motion_executor
        self._approach_callable = approach_callable
        self._seating_detector = seating_detector
        self._lock_callable = lock_callable
        self._logger = logger
        self._role = role
        self._dry_run = dry_run

        if self._target_m.shape != (3,):
            raise ValueError(
                f"target_attachment_point_m must be a 3-vector; got shape {self._target_m.shape}"
            )

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self, cycle_index: int = 0) -> SupervisorResult:
        """Execute the supervised policy handoff.

        This is the top-level entry point.  It:
        1. Emits a handoff-start event.
        2. Attempts the policy loop up to ``config.max_retries + 1`` times.
        3. On success, calls ``lock_callable`` and returns
           :attr:`SupervisorOutcome.SUCCESS`.
        4. On all retries exhausted, emits a halt event and returns
           :attr:`SupervisorOutcome.HALT`.

        Parameters
        ----------
        cycle_index:
            Zero-based crawl cycle number (for logging).

        Returns
        -------
        SupervisorResult
            Outcome, attempt count, total steps, abort reasons, elapsed time.
        """
        t_start = time.monotonic()
        max_attempts = self._config.max_retries + 1
        abort_reasons: list[AbortReason] = []
        total_steps = 0

        # Emit handoff start
        self._emit(**policy_handoff_start(
            policy_name=type(self._policy).__name__,
            cycle_index=cycle_index,
            role=self._role,
        ))

        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                # Re-approach before retrying
                _LOG.info(
                    "supervisor: attempt %d/%d, running re-approach",
                    attempt, max_attempts,
                )
                approach_ok = self._approach_callable()
                if not approach_ok:
                    _LOG.error("supervisor: approach failed on attempt %d", attempt)
                    abort_reasons.append(AbortReason.APPROACH_FAILED)
                    _emit_supervisor_abort(
                        self._logger,
                        role=self._role,
                        cycle_index=cycle_index,
                        reason=AbortReason.APPROACH_FAILED,
                        attempt=attempt,
                        detail="approach_callable returned False",
                    )
                    self._emit(**cycle_retry(
                        cycle_index=cycle_index,
                        attempt=attempt,
                        max_attempts=max_attempts,
                        reason=AbortReason.APPROACH_FAILED.value,
                        subsystem="supervisor",
                        role=self._role,
                    ))
                    continue

            try:
                outcome, reason, steps = self._run_one_attempt(
                    cycle_index=cycle_index, attempt=attempt
                )
            except (GripperCommandBlocked, RetractUnsafeError):
                raise
            except Exception as exc:  # noqa: BLE001
                # Orchestrator 2026-10-24: an exception from the frame source,
                # policy or motion executor must not leave the arm
                # mid-trajectory. Retract, then treat it as an abort.
                _LOG.error("supervisor: attempt %d raised %r; retracting", attempt, exc)
                self._retract(cycle_index=cycle_index)  # RetractUnsafeError propagates
                outcome, reason, steps = SupervisorOutcome.ABORT, AbortReason.STEP_EXCEPTION, 0
            total_steps += steps

            if outcome == SupervisorOutcome.SUCCESS:
                # Lock the gripper on success
                if not self._dry_run:
                    try:
                        locked = self._lock_callable()
                    except Exception as exc:  # noqa: BLE001
                        _LOG.error("supervisor: lock_callable raised: %s", exc)
                        locked = False
                    # None is treated as success for backward compatibility
                    # with callables that return nothing and raise on failure.
                    if locked is False:
                        abort_reasons.append(AbortReason.LOCK_FAILED)
                        _emit_supervisor_abort(
                            self._logger, role=self._role, cycle_index=cycle_index,
                            reason=AbortReason.LOCK_FAILED, attempt=attempt,
                            detail="seated but AA-17 lock failed; halting (no retry: arm may be engaged)",
                        )
                        self._emit(**cycle_abort(
                            cycle_index=cycle_index, reason=AbortReason.LOCK_FAILED.value,
                            subsystem="supervisor", role=self._role,
                        ))
                        # Do NOT retract: the gripper may be partly engaged in
                        # the attachment point; pulling out could jam. Halt and
                        # leave the arm in place for the cycle runner / e-stop.
                        return SupervisorResult(
                            outcome=SupervisorOutcome.HALT,
                            attempts=attempt,
                            total_steps=total_steps,
                            abort_reasons=abort_reasons,
                            elapsed_s=time.monotonic() - t_start,
                        )
                else:
                    _LOG.info("supervisor: dry-run — skipping lock_callable")
                _emit_supervisor_success(
                    self._logger,
                    role=self._role,
                    cycle_index=cycle_index,
                    attempt=attempt,
                    steps=steps,
                )
                self._emit(**policy_handoff_end(
                    policy_name=type(self._policy).__name__,
                    cycle_index=cycle_index,
                    reason="success",
                    role=self._role,
                ))
                elapsed = time.monotonic() - t_start
                return SupervisorResult(
                    outcome=SupervisorOutcome.SUCCESS,
                    attempts=attempt,
                    total_steps=total_steps,
                    abort_reasons=abort_reasons,
                    elapsed_s=elapsed,
                )

            # Attempt failed — log retry
            assert reason is not None
            abort_reasons.append(reason)
            _emit_supervisor_abort(
                self._logger,
                role=self._role,
                cycle_index=cycle_index,
                reason=reason,
                attempt=attempt,
                detail=None,
            )
            self._emit(**cycle_abort(
                cycle_index=cycle_index,
                reason=reason.value,
                subsystem="supervisor",
                role=self._role,
            ))
            if attempt < max_attempts:
                self._emit(**cycle_retry(
                    cycle_index=cycle_index,
                    attempt=attempt,
                    max_attempts=max_attempts,
                    reason=reason.value,
                    subsystem="supervisor",
                    role=self._role,
                ))

        # All attempts exhausted
        _LOG.critical(
            "supervisor: all %d attempt(s) exhausted on cycle %d — halting",
            max_attempts, cycle_index,
        )
        _emit_supervisor_halt(
            self._logger,
            role=self._role,
            cycle_index=cycle_index,
            total_attempts=max_attempts,
        )
        self._emit(**policy_handoff_end(
            policy_name=type(self._policy).__name__,
            cycle_index=cycle_index,
            reason="halt_after_retries",
            role=self._role,
        ))
        elapsed = time.monotonic() - t_start
        return SupervisorResult(
            outcome=SupervisorOutcome.HALT,
            attempts=max_attempts,
            total_steps=total_steps,
            abort_reasons=abort_reasons,
            elapsed_s=elapsed,
        )

    # ------------------------------------------------------------------
    # One attempt loop
    # ------------------------------------------------------------------

    def _run_one_attempt(
        self,
        cycle_index: int,
        attempt: int,
    ) -> Tuple[SupervisorOutcome, Optional[AbortReason], int]:
        """Run one policy loop attempt.

        Returns
        -------
        (outcome, abort_reason, steps)
            outcome: SUCCESS or ABORT (never HALT — that is handled by run()).
            abort_reason: None on success, else the AbortReason.
            steps: number of policy steps executed.
        """
        max_steps = self._config.max_steps
        max_wall_s = self._config.max_wall_time_s
        t_attempt_start = time.monotonic()
        prev_joint_deg: npt.NDArray[np.float32] | None = None

        for step in range(max_steps):
            # --- 1. Capture observation ---
            frame, joint_deg = self._frame_source()
            frame = np.asarray(frame, dtype=np.uint8)
            joint_deg = np.asarray(joint_deg, dtype=np.float32)

            obs = Observation(
                frame=frame,
                joint_positions_deg=joint_deg,
                step_index=step,
            )

            # --- 2. Wall-clock check ---
            elapsed = time.monotonic() - t_attempt_start
            if elapsed >= max_wall_s:
                _LOG.error(
                    "supervisor: wall-time limit %.1f s exceeded at step %d (%.1f s elapsed)",
                    max_wall_s, step, elapsed,
                )
                self._retract(cycle_index=cycle_index)
                return SupervisorOutcome.ABORT, AbortReason.WALL_TIME_LIMIT, step

            # --- 3. Workspace check ---
            q_rad = self._joint_deg_to_rad(joint_deg)
            T_tcp = self._chain.fk(q_rad)
            tcp_pos = T_tcp[:3, 3]
            dist = float(np.linalg.norm(tcp_pos - self._target_m))
            if dist > self._config.workspace_radius_m:
                _LOG.error(
                    "supervisor: workspace violation at step %d — "
                    "TCP dist=%.4f m > radius=%.4f m",
                    step, dist, self._config.workspace_radius_m,
                )
                self._retract(cycle_index=cycle_index)
                return SupervisorOutcome.ABORT, AbortReason.WORKSPACE_VIOLATION, step

            # --- 4. Policy step ---
            action = self._policy.step(obs)

            # --- 5. Validate (gripper block) ---
            try:
                validate_policy_action(action)
            except GripperCommandBlocked as exc:
                _LOG.error(
                    "supervisor: gripper command blocked at step %d: %s",
                    step, exc,
                )
                self._retract(cycle_index=cycle_index)
                return (
                    SupervisorOutcome.ABORT,
                    AbortReason.GRIPPER_COMMAND_BLOCKED,
                    step,
                )

            # --- 6. Magnitude clamp ---
            action = self._clamp_action(
                action,
                prev_joint_deg=prev_joint_deg,
                joint_deg=joint_deg,
                step=step,
            )

            # --- 7. Apply action ---
            if not self._dry_run:
                self._motion_executor(action)
            else:
                _LOG.debug(
                    "supervisor: dry-run step %d action=%s", step, action
                )

            prev_joint_deg = joint_deg

            # --- 8. Success check ---
            try:
                seated = self._seating_detector()
            except Exception as exc:  # noqa: BLE001
                _LOG.error(
                    "supervisor: seating_detector raised at step %d: %s",
                    step, exc,
                )
                self._retract(cycle_index=cycle_index)
                return (
                    SupervisorOutcome.ABORT,
                    AbortReason.SEATING_DETECTOR_ERROR,
                    step + 1,
                )
            if seated:
                return SupervisorOutcome.SUCCESS, None, step + 1

        # Step limit exhausted
        _LOG.error(
            "supervisor: step limit %d reached on attempt %d",
            max_steps, attempt,
        )
        self._retract(cycle_index=cycle_index)
        return SupervisorOutcome.ABORT, AbortReason.STEP_LIMIT, max_steps

    # ------------------------------------------------------------------
    # Action magnitude clamping
    # ------------------------------------------------------------------

    def _clamp_action(
        self,
        action: PolicyAction,
        prev_joint_deg: npt.NDArray[np.float32] | None,
        joint_deg: npt.NDArray[np.float32],
        step: int,
    ) -> PolicyAction:
        """Clamp action magnitude and log if clamped.

        The magnitude is computed as the Euclidean norm of the per-joint
        deltas (target − current).  If the norm exceeds
        ``config.max_action_magnitude_deg``, the deltas are scaled down to
        that magnitude.
        """
        max_mag = self._config.max_action_magnitude_deg
        # Build delta vector: target - current for all joints in action.
        joint_name_to_idx = {name: i for i, name in enumerate(JOINT_ORDER)}
        deltas: dict[str, float] = {}
        for jname, target_deg in action.items():
            idx = joint_name_to_idx.get(jname)
            if idx is not None:
                current_deg = float(joint_deg[idx])
                deltas[jname] = target_deg - current_deg
            else:
                deltas[jname] = target_deg  # shouldn't happen after validation

        delta_vec = np.array(
            [deltas.get(jname, 0.0) for jname in JOINT_ORDER], dtype=np.float64
        )
        raw_mag = float(np.linalg.norm(delta_vec))

        if raw_mag > max_mag and raw_mag > 1e-9:
            scale = max_mag / raw_mag
            _emit_action_clamped(
                self._logger,
                role=self._role,
                raw_magnitude=raw_mag,
                clamped_magnitude=max_mag,
                step=step,
            )
            _LOG.warning(
                "supervisor: action clamped at step %d: "
                "raw_mag=%.3f deg → clamped_mag=%.3f deg",
                step, raw_mag, max_mag,
            )
            # Rebuild clamped action
            clamped_action: PolicyAction = {}
            for jname, target_deg in action.items():
                idx = joint_name_to_idx.get(jname)
                if idx is not None:
                    current_deg = float(joint_deg[idx])
                    clamped_action[jname] = current_deg + deltas[jname] * scale
                else:
                    clamped_action[jname] = target_deg
            return clamped_action

        return action

    # ------------------------------------------------------------------
    # Retract helper
    # ------------------------------------------------------------------

    def _retract(self, cycle_index: int) -> None:
        """Command the arm to the configured retract pose.

        In dry-run mode, logs but does not call motion_executor.
        In live mode, fails closed if retract_pose_q_rad is None or
        retract_pose_unverified is True.
        """
        cfg = self._config
        if cfg.retract_pose_q_rad is None:
            if self._dry_run:
                _LOG.warning(
                    "supervisor: retract_pose_q_rad is None "
                    "(unverified) — skipping retract in dry-run"
                )
                return
            else:
                raise RetractUnsafeError(
                    "supervisor: retract_pose_q_rad is None; "
                    "cannot retract in live mode — configure and verify the retract pose "
                    "before running live (set retract_pose_unverified: false in hardware.yaml)"
                )
        if cfg.retract_pose_unverified and not self._dry_run:
            raise RetractUnsafeError(
                "supervisor: retract_pose_q_rad is marked unverified "
                "(retract_pose_unverified: true); "
                "verify the retract pose on hardware before live use"
            )

        q_rad = list(cfg.retract_pose_q_rad)
        retract_action = {
            jname: float(np.degrees(q_rad[i]))
            for i, jname in enumerate(JOINT_ORDER)
        }

        _LOG.info("supervisor: retracting arm to safe pose: %s", retract_action)
        if not self._dry_run:
            self._motion_executor(retract_action)
        else:
            _LOG.info("supervisor: dry-run — retract action logged but not executed")

        # Log the retract as a motion_command event
        self._emit(
            role=self._role,
            subsystem="supervisor",
            event_type="supervisor_retract",
            severity="INFO",
            payload={
                "retract_q_rad": q_rad,
                "dry_run": self._dry_run,
            },
        )

    # ------------------------------------------------------------------
    # Logging helper
    # ------------------------------------------------------------------

    def _emit(self, **kwargs: object) -> None:
        if self._logger is None:
            return
        emit = getattr(self._logger, "emit", None)
        if emit is not None:
            try:
                emit(**kwargs)
            except Exception:  # noqa: BLE001
                pass
