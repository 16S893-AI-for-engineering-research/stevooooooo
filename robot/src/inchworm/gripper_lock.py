"""AA-17 Gripper lock state machine for the custom SO-101 gripper servo.

The owner replaced the stock SO-101 gripper with a custom design in which the
latch and the jaw are driven by the same servo (joint 6 / "gripper").  Lock and
unlock are therefore position bands of one servo rather than separate actuators.

This module is the **single authoritative interface** for commanding the
gripper servo.  It obtains a :class:`~inchworm.servo_bus.GripperChannel` via
the AA-8 capability token and is the **only** module in the codebase permitted
to import that token (enforced by the test in ``test_servo_bus.py`` and
extended in ``test_gripper_lock.py``).

State machine
-------------
::

    UNKNOWN ─────────────────────────────────────────────────────────┐
       │  reset() / initial                                           │
       ▼                                                              │
    UNLOCKED ──► APPROACH_READY ──► LOCKING ──► LOCKED ──► UNLOCKING ┘
                                                  │  hold_tick()
                                                  │  (re-asserts drift)
                                               (LOCKED)
    Any state ──► FAULT  (on unrecoverable error; only reset() escapes)

Transition table
----------------
- ``reset()``             : any → UNLOCKED  (clears FAULT; re-reads position)
- ``set_approach_ready()`` : UNLOCKED → APPROACH_READY
- ``lock()``              : APPROACH_READY → LOCKING → LOCKED  (or FAULT)
- ``hold_tick()``         : LOCKED → LOCKED  (re-asserts if drift detected)
- ``unlock()``            : LOCKED → UNLOCKING → UNLOCKED  (or FAULT)

Illegal transitions raise :class:`IllegalTransition`.

Band policy
-----------
Servo position bands (unlocked / engaged-but-unlocked / locked) are loaded
from :class:`~inchworm.config.GripperBandConfig` via the
:class:`~inchworm.config.HardwareConfig`.  When any band boundary is ``None``
(unverified / not yet measured):

- **Live mode**: every ``lock()`` and ``unlock()`` is *refused* with
  :class:`BandsNotConfigured` (fail-closed).
- **Dry-run mode**: the band check is skipped; positions are logged but never
  sent.

Use ``scripts/gripper_bands.py --live`` to measure bands with owner
supervision and populate them in ``config/hardware.yaml``.

Lock verification
-----------------
After issuing the lock command, :meth:`GripperLockSM.lock` reads back the
:class:`~inchworm.servo_bus.JointState` and calls :func:`verify_lock_state` to
classify the outcome as one of:

- :attr:`LockVerdict.LATCHED`  – position in locked band, load/current nominal.
- :attr:`LockVerdict.JAM`      – position in locked band, but load/current
                                  exceeds the jam threshold (servo stalled).
- :attr:`LockVerdict.MISS`     – position NOT in any expected locked region
                                  (attachment not reached / knocked away).
- :attr:`LockVerdict.UNVERIFIED` – thresholds are null; cannot classify
                                  (state machine enters FAULT in live mode).

Active hold
-----------
Call :meth:`GripperLockSM.hold_tick` periodically (e.g. every 20 ms from a
supervisor loop).  When drift from the last commanded position exceeds
``drift_threshold_deg`` (from config, or a built-in fallback), the gripper
re-asserts its position and logs a ``gripper_hold_correction`` event via AA-7.

Safety: unlock refused unless opposite arm is holding
------------------------------------------------------
:meth:`GripperLockSM.unlock` requires an *opposite-arm holding predicate*
injected at construction time (``opposite_arm_locked_fn``).  This callable
must return ``True`` when the other arm is verified LOCKED.  If it returns
``False`` or raises, the unlock is refused with :class:`UnlockRefused`.

This enforces the AGENTS.md invariant: **never both grippers unlocked while
the tube is attached**.

Unlock-under-load behaviour
---------------------------
If a lock-verification check shows the gripper is under significant load
(``load_raw`` above ``jam_load_threshold_raw``), unlocking releases under
load.  The state machine permits this but logs a WARNING event
``gripper_unlock_under_load`` so the operator is informed.  The calling
supervisor layer is responsible for deciding whether to proceed.

Policy isolation
----------------
This module is **explicitly excluded** from the policy action path.  The
policy-facing adapter (:mod:`inchworm.policy_actions`) accepts only joints
1–5 and raises on any gripper key.  The only path to this module is through
deliberate orchestrator code that holds the capability token.
"""

from __future__ import annotations

import enum
from typing import Callable, Optional

from inchworm.config import (
    GripperBandConfig,
    GripperLockConfig,
    GripperVerificationThresholds,
)
from inchworm.events import gripper_lock as _evt_lock
from inchworm.events import gripper_unlock as _evt_unlock
from inchworm.servo_bus import ArmServoBus, JointState

# Capability token – only this module (gripper_lock*.py) may import it.
from inchworm.gripper_access import _GRIPPER_TOKEN  # noqa: F401 (used below)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Consecutive hold_tick read failures before the lock is declared lost (FAULT).
_HOLD_MAX_READ_FAILURES: int = 3

#: Default drift threshold (degrees) used when config value is None.
#: Conservative placeholder; replace with a measured value.
_DEFAULT_DRIFT_THRESHOLD_DEG: float = 2.0

#: Goal position for the locked state (degrees).  UNVERIFIED placeholder.
#: Used only in dry-run mode or when config bands are null; must be replaced
#: by the owner after the measurement session.
_PLACEHOLDER_LOCKED_POS_DEG: float = 180.0

#: Goal position for the unlocked state (degrees).  UNVERIFIED placeholder.
_PLACEHOLDER_UNLOCKED_POS_DEG: float = 90.0

#: Goal position for the approach-ready state (degrees).  UNVERIFIED placeholder.
_PLACEHOLDER_APPROACH_POS_DEG: float = 120.0

# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class GripperLockError(Exception):
    """Base class for all gripper-lock state-machine errors."""


class IllegalTransition(GripperLockError):
    """Raised when a transition is requested from a state that does not allow it.

    Attributes
    ----------
    current_state:
        The state the machine is currently in.
    requested_op:
        The name of the operation that was requested.
    """

    def __init__(self, current_state: "GripperState", requested_op: str) -> None:
        self.current_state = current_state
        self.requested_op = requested_op
        super().__init__(
            f"cannot perform {requested_op!r} from state {current_state.name!r}"
        )


class BandsNotConfigured(GripperLockError):
    """Raised when a live lock/unlock is attempted but band boundaries are null.

    In live mode the state machine fails closed when bands are not configured.
    Use dry_run=True or populate the bands in hardware.yaml first.
    """

    def __init__(self, role: str, operation: str) -> None:
        self.role = role
        self.operation = operation
        super().__init__(
            f"{role}: cannot perform {operation!r} in live mode: "
            "gripper position bands are not configured (null). "
            "Run scripts/gripper_bands.py --live to measure them."
        )


class LockVerificationFailed(GripperLockError):
    """Raised when lock verification fails (jam, miss, or unverified thresholds).

    Attributes
    ----------
    verdict:
        The :class:`LockVerdict` returned by :func:`verify_lock_state`.
    """

    def __init__(self, verdict: "LockVerdict", detail: str) -> None:
        self.verdict = verdict
        self.detail = detail
        super().__init__(f"lock verification failed: {verdict.name} – {detail}")


class UnlockRefused(GripperLockError):
    """Raised when unlock is refused because the opposite arm is not holding.

    Enforces the AGENTS.md invariant: never both grippers unlocked while the
    tube is attached.
    """

    def __init__(self, role: str, reason: str) -> None:
        self.role = role
        self.reason = reason
        super().__init__(
            f"{role}: unlock refused – {reason}. "
            "The opposite arm must be verified LOCKED before unlocking."
        )


# ---------------------------------------------------------------------------
# State enum
# ---------------------------------------------------------------------------


class GripperState(enum.Enum):
    """States of the gripper lock state machine."""

    UNKNOWN = "UNKNOWN"
    UNLOCKED = "UNLOCKED"
    APPROACH_READY = "APPROACH_READY"
    LOCKING = "LOCKING"
    LOCKED = "LOCKED"
    UNLOCKING = "UNLOCKING"
    FAULT = "FAULT"


# ---------------------------------------------------------------------------
# Lock verdict
# ---------------------------------------------------------------------------


class LockVerdict(enum.Enum):
    """Result of a lock verification check.

    LATCHED:
        Position in locked band, load and current within nominal range.
        The latch is positively seated.
    JAM:
        Position nominally in locked band but load or current exceeds the jam
        threshold.  The servo stalled; the latch likely hit a hard stop before
        seating.
    MISS:
        Position NOT in the locked band.  The attachment point was not reached
        or was knocked away.
    UNVERIFIED:
        Verification thresholds are null (not yet measured).  Cannot classify.
        The state machine treats this as FAULT in live mode.
    """

    LATCHED = "LATCHED"
    JAM = "JAM"
    MISS = "MISS"
    UNVERIFIED = "UNVERIFIED"


# ---------------------------------------------------------------------------
# Band helpers
# ---------------------------------------------------------------------------


def _in_band(value: float, low: Optional[float], high: Optional[float]) -> bool:
    """Return True iff ``value`` is in [low, high] and neither bound is None."""
    if low is None or high is None:
        return False
    return low <= value <= high


def _bands_configured(bands: Optional[GripperBandConfig]) -> bool:
    """Return True only when all six band boundaries are non-None."""
    if bands is None:
        return False
    return all(
        v is not None
        for v in (
            bands.unlocked_low_deg,
            bands.unlocked_high_deg,
            bands.engaged_low_deg,
            bands.engaged_high_deg,
            bands.locked_low_deg,
            bands.locked_high_deg,
        )
    )


def _mid(low: Optional[float], high: Optional[float]) -> float:
    """Return the midpoint of a band (both must be non-None)."""
    assert low is not None and high is not None
    return (low + high) / 2.0


# ---------------------------------------------------------------------------
# Lock verification
# ---------------------------------------------------------------------------


def verify_lock_state(
    state: JointState,
    bands: Optional[GripperBandConfig],
    thresholds: Optional[GripperVerificationThresholds],
) -> LockVerdict:
    """Classify the lock state given a freshly-read :class:`~inchworm.servo_bus.JointState`.

    Parameters
    ----------
    state:
        The servo joint state read immediately after the lock command.
    bands:
        Per-arm band config (from hardware.yaml).  Null bands mean
        :attr:`LockVerdict.UNVERIFIED`.
    thresholds:
        Verification thresholds (jam/latch load and current).  Null
        thresholds mean :attr:`LockVerdict.UNVERIFIED`.

    Returns
    -------
    :class:`LockVerdict`
        LATCHED, JAM, MISS, or UNVERIFIED.

    Note: ``load_raw`` uses the sign-magnitude convention from the STS3215
    (bit 10 = sign, bits 0-9 = magnitude); we compare the *absolute value*.
    """
    if bands is None or not _bands_configured(bands):
        return LockVerdict.UNVERIFIED
    if thresholds is None or thresholds.unverified:
        return LockVerdict.UNVERIFIED

    pos = state.position_deg
    abs_load = abs(state.load_raw)
    current = state.current_raw

    in_locked = _in_band(pos, bands.locked_low_deg, bands.locked_high_deg)

    if not in_locked:
        return LockVerdict.MISS

    # Position is in locked band – check load/current
    jam_thresh = thresholds.jam_load_threshold_raw
    latch_load_max = thresholds.latch_load_max_raw
    latch_current_max = thresholds.latch_current_max_raw

    if jam_thresh is None or latch_load_max is None or latch_current_max is None:
        return LockVerdict.UNVERIFIED

    # JAM: load exceeds the maximum expected for a true latch
    if abs_load > latch_load_max:
        return LockVerdict.JAM
    # JAM: current exceeds the maximum expected for a true latch
    if current > latch_current_max:
        return LockVerdict.JAM

    return LockVerdict.LATCHED


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------


class GripperLockSM:
    """Gripper lock state machine for one SO-101 arm.

    Parameters
    ----------
    bus:
        The :class:`~inchworm.servo_bus.ArmServoBus` for the arm owning
        this gripper.
    role:
        ``"arm_left"`` or ``"arm_right"``.
    config:
        :class:`~inchworm.config.GripperLockConfig` loaded from hardware.yaml.
        May be ``None``; null bands cause live lock/unlock to fail closed.
    opposite_arm_locked_fn:
        A zero-argument callable that returns ``True`` when the **other** arm
        is verified LOCKED.  Injected at construction to avoid circular
        imports and to allow easy testing with a mock.  If ``None``, every
        ``unlock()`` call is refused (safest possible default).
    logger:
        Optional :class:`~inchworm.telemetry.SessionLogger` for AA-7 event
        emission.  When ``None``, events are silently discarded.
    dry_run:
        When ``True``, servo commands are logged but never sent; band
        boundary null-checks are skipped.

    Usage::

        from inchworm.gripper_lock import GripperLockSM
        from inchworm.config import load_config
        from inchworm.backends import MockBackend

        config = load_config()
        backend = MockBackend(config)
        transport = backend.open_serial("arm_left")
        bus = ArmServoBus(transport, role="arm_left", joint_id_map={...})

        sm = GripperLockSM(
            bus=bus,
            role="arm_left",
            config=config.gripper_lock,
            opposite_arm_locked_fn=lambda: right_sm.state == GripperState.LOCKED,
            dry_run=True,
        )
        sm.reset()
        sm.set_approach_ready()
        sm.lock()
        # ... robot holds tube ...
        # right_sm must be LOCKED before this succeeds:
        sm.unlock()
    """

    def __init__(
        self,
        bus: ArmServoBus,
        role: str,
        config: Optional[GripperLockConfig],
        opposite_arm_locked_fn: Optional[Callable[[], bool]] = None,
        logger: Optional[object] = None,
        dry_run: bool = False,
    ) -> None:
        if role not in ("arm_left", "arm_right"):
            raise ValueError(f"role must be 'arm_left' or 'arm_right', got {role!r}")
        self._bus = bus
        self._role = role
        self._config = config
        self._opposite_arm_locked_fn = opposite_arm_locked_fn
        self._logger = logger
        self._dry_run = dry_run

        # Obtain the capability channel (only this module may hold it).
        self._channel = bus.gripper_channel(_GRIPPER_TOKEN)

        # Internal state
        self._state: GripperState = GripperState.UNKNOWN
        self._last_commanded_pos_deg: Optional[float] = None

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> GripperState:
        """Current state of the gripper lock state machine."""
        return self._state

    @property
    def role(self) -> str:
        """Arm role this state machine controls."""
        return self._role

    @property
    def dry_run(self) -> bool:
        """True when running in dry-run mode (no physical commands sent)."""
        return self._dry_run

    # ------------------------------------------------------------------
    # Band / config helpers
    # ------------------------------------------------------------------

    def _bands_for_role(self) -> Optional[GripperBandConfig]:
        """Return the GripperBandConfig for this arm, or None."""
        if self._config is None:
            return None
        return (
            self._config.arm_left
            if self._role == "arm_left"
            else self._config.arm_right
        )

    def _drift_threshold(self) -> float:
        """Return the drift threshold in degrees (config or default)."""
        if self._config is not None and self._config.drift_threshold_deg is not None:
            return float(self._config.drift_threshold_deg)
        return _DEFAULT_DRIFT_THRESHOLD_DEG

    def _verification_thresholds(self) -> Optional[GripperVerificationThresholds]:
        """Return the lock-verification thresholds, or None."""
        if self._config is None:
            return None
        return self._config.verification

    # ------------------------------------------------------------------
    # AA-7 event helpers
    # ------------------------------------------------------------------

    def _emit(self, severity: str, event_type: str, payload: dict) -> None:
        """Emit a structured event via the injected SessionLogger."""
        if self._logger is None:
            return
        method = getattr(self._logger, severity.lower(), None)
        if method is not None:
            method(
                role=self._role,
                subsystem="gripper_lock",
                event_type=event_type,
                payload=payload,
            )

    def _emit_gripper_lock_event(self, from_state: GripperState) -> None:
        """Emit an AA-7 gripper_lock event via events.py constructor."""
        if self._logger is None:
            return
        from_str = from_state.name.lower()
        # events.py accepts: 'locked', 'unlocked', 'unknown'
        if from_str not in ("locked", "unlocked", "unknown"):
            from_str = "unknown"
        kwargs = _evt_lock(role=self._role, from_state=from_str)  # type: ignore[arg-type]
        self._logger.emit(**kwargs)  # type: ignore[union-attr]

    def _emit_gripper_unlock_event(self, from_state: GripperState) -> None:
        """Emit an AA-7 gripper_unlock event via events.py constructor."""
        if self._logger is None:
            return
        from_str = from_state.name.lower()
        if from_str not in ("locked", "unlocked", "unknown"):
            from_str = "unknown"
        kwargs = _evt_unlock(role=self._role, from_state=from_str)  # type: ignore[arg-type]
        self._logger.emit(**kwargs)  # type: ignore[union-attr]

    # ------------------------------------------------------------------
    # Live-mode band guard
    # ------------------------------------------------------------------

    def _require_bands_or_dry_run(self, operation: str) -> None:
        """Raise BandsNotConfigured if bands are null and not in dry-run mode."""
        if self._dry_run:
            return  # dry-run skips null check
        bands = self._bands_for_role()
        if not _bands_configured(bands):
            raise BandsNotConfigured(self._role, operation)

    # ------------------------------------------------------------------
    # Servo write helpers
    # ------------------------------------------------------------------

    def _goal_pos_locked(self) -> float:
        """Return the goal position for the locked state (midpoint of locked band)."""
        bands = self._bands_for_role()
        if bands is not None and _bands_configured(bands):
            return _mid(bands.locked_low_deg, bands.locked_high_deg)
        return _PLACEHOLDER_LOCKED_POS_DEG

    def _goal_pos_unlocked(self) -> float:
        """Return the goal position for the unlocked state."""
        bands = self._bands_for_role()
        if bands is not None and _bands_configured(bands):
            return _mid(bands.unlocked_low_deg, bands.unlocked_high_deg)
        return _PLACEHOLDER_UNLOCKED_POS_DEG

    def _goal_pos_approach(self) -> float:
        """Return the goal position for the approach-ready state."""
        bands = self._bands_for_role()
        if bands is not None and _bands_configured(bands):
            return _mid(bands.engaged_low_deg, bands.engaged_high_deg)
        return _PLACEHOLDER_APPROACH_POS_DEG

    def _write_gripper(self, pos_deg: float, context: str) -> None:
        """Write position to the gripper channel and record the commanded value.

        In dry-run mode the command is logged but never sent to the transport.
        This method respects the state machine's own dry_run flag; the
        GripperChannel additionally checks the bus dry_run flag when present.
        """
        if self._dry_run:
            self._last_commanded_pos_deg = pos_deg
            self._emit(
                "info",
                "gripper_position_command",
                {"position_deg": pos_deg, "context": context, "dry_run": True},
            )
            return
        self._channel.write_position_deg(pos_deg)
        self._last_commanded_pos_deg = pos_deg
        self._emit(
            "info",
            "gripper_position_command",
            {"position_deg": pos_deg, "context": context, "dry_run": self._dry_run},
        )

    # ------------------------------------------------------------------
    # Public transitions
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Transition from any state to UNLOCKED.

        Clears FAULT state.  Commands the gripper to the unlocked position
        (or logs the dry-run command).  Safe to call at any time.
        """
        prev = self._state
        self._state = GripperState.UNLOCKED
        pos = self._goal_pos_unlocked()
        self._write_gripper(pos, "reset_to_unlocked")
        self._emit(
            "info",
            "gripper_state_reset",
            {"prev_state": prev.name, "new_state": "UNLOCKED", "position_deg": pos},
        )

    def set_approach_ready(self) -> None:
        """Transition UNLOCKED → APPROACH_READY.

        Commands the gripper to the engaged-but-unlocked band (approach
        position), where the jaw contacts the attachment but the latch is
        not yet seated.

        Raises :class:`IllegalTransition` if not currently UNLOCKED.
        Raises :class:`BandsNotConfigured` in live mode when bands are null.
        """
        if self._state != GripperState.UNLOCKED:
            raise IllegalTransition(self._state, "set_approach_ready")
        self._require_bands_or_dry_run("set_approach_ready")
        self._state = GripperState.APPROACH_READY
        pos = self._goal_pos_approach()
        self._write_gripper(pos, "approach_ready")
        self._emit(
            "info",
            "gripper_approach_ready",
            {"position_deg": pos},
        )

    def lock(self) -> LockVerdict:
        """Transition APPROACH_READY → LOCKING → LOCKED (or FAULT).

        Commands the gripper to the locked-band position, reads back the
        joint state, and verifies the lock via :func:`verify_lock_state`.

        In dry-run mode:
            - The position command is logged but not sent.
            - Verification is skipped; the machine transitions to LOCKED.
            - Returns :attr:`LockVerdict.LATCHED`.

        In live mode with null bands:
            - Raises :class:`BandsNotConfigured` (fail-closed).

        In live mode with configured bands but unverified thresholds:
            - Transitions to FAULT.
            - Raises :class:`LockVerificationFailed` with UNVERIFIED verdict.

        Returns
        -------
        :class:`LockVerdict`
            LATCHED on success; raises on MISS/JAM/UNVERIFIED.

        Raises
        ------
        :class:`IllegalTransition`
            If not currently in APPROACH_READY.
        :class:`BandsNotConfigured`
            In live mode when bands are null.
        :class:`LockVerificationFailed`
            When the servo does not confirm a true latch (JAM, MISS, or
            UNVERIFIED thresholds in live mode).
        """
        if self._state != GripperState.APPROACH_READY:
            raise IllegalTransition(self._state, "lock")
        self._require_bands_or_dry_run("lock")

        prev_state = self._state
        self._state = GripperState.LOCKING
        self._emit(
            "info",
            "gripper_locking_started",
            {"prev_state": prev_state.name},
        )

        # Command the locked position
        pos = self._goal_pos_locked()
        self._write_gripper(pos, "lock_command")

        if self._dry_run:
            # Dry-run: skip verification, transition to LOCKED
            self._state = GripperState.LOCKED
            self._emit_gripper_lock_event(GripperState.UNLOCKED)
            self._emit(
                "info",
                "gripper_locked",
                {"position_deg": pos, "verdict": "LATCHED", "dry_run": True},
            )
            return LockVerdict.LATCHED

        # Live: read back and verify
        gripper_servo_id = self._bus._servo_id_for_joint("gripper")
        joint_state = self._bus.read_joint_state(gripper_servo_id)
        bands = self._bands_for_role()
        thresholds = self._verification_thresholds()
        verdict = verify_lock_state(joint_state, bands, thresholds)

        if verdict == LockVerdict.LATCHED:
            self._state = GripperState.LOCKED
            self._emit_gripper_lock_event(GripperState.UNLOCKED)
            self._emit(
                "info",
                "gripper_locked",
                {
                    "position_deg": joint_state.position_deg,
                    "load_raw": joint_state.load_raw,
                    "current_raw": joint_state.current_raw,
                    "verdict": verdict.name,
                },
            )
            return LockVerdict.LATCHED

        # Verification failed – go to FAULT
        self._state = GripperState.FAULT
        detail = (
            f"position={joint_state.position_deg:.2f}deg, "
            f"load_raw={joint_state.load_raw}, "
            f"current_raw={joint_state.current_raw}"
        )
        self._emit(
            "error",
            "gripper_lock_failed",
            {"verdict": verdict.name, "detail": detail},
        )
        raise LockVerificationFailed(verdict, detail)

    def hold_tick(self) -> bool:
        """Re-assert locked position if drift exceeds threshold.

        Call this method periodically (e.g. every 20 ms from a supervisor
        loop) while the gripper is in LOCKED state.  If the present position
        has drifted more than ``drift_threshold_deg`` from the last commanded
        position, the servo is re-commanded and a ``gripper_hold_correction``
        event is logged.

        Returns
        -------
        bool
            ``True`` if a correction was issued, ``False`` otherwise.

        Raises
        ------
        :class:`IllegalTransition`
            If the state machine is not currently in LOCKED state.
        """
        if self._state != GripperState.LOCKED:
            raise IllegalTransition(self._state, "hold_tick")

        if self._last_commanded_pos_deg is None:
            # No commanded position recorded – nothing to drift from
            return False

        if self._dry_run:
            # Dry-run: log but don't actually read or write
            self._emit(
                "debug",
                "gripper_hold_tick_dry_run",
                {"commanded_pos_deg": self._last_commanded_pos_deg},
            )
            return False

        gripper_servo_id = self._bus._servo_id_for_joint("gripper")
        try:
            present_pos = self._bus.read_position_deg(gripper_servo_id)
        except Exception as exc:  # noqa: BLE001
            # Orchestrator 2026-10-24: a silently dead gripper servo must not
            # be reported as "holding". Escalate to FAULT after N consecutive
            # read failures so the supervisor stops the cycle.
            self._hold_read_failures = getattr(self, "_hold_read_failures", 0) + 1
            self._emit(
                "warning",
                "gripper_hold_read_failed",
                {"error": str(exc), "consecutive": self._hold_read_failures},
            )
            if self._hold_read_failures >= _HOLD_MAX_READ_FAILURES:
                self._state = GripperState.FAULT
                self._emit(
                    "error",
                    "gripper_hold_lost",
                    {"consecutive_read_failures": self._hold_read_failures},
                )
                raise GripperLockError(
                    f"{self._role}: gripper hold cannot be verified after "
                    f"{self._hold_read_failures} consecutive read failures"
                ) from exc
            return False
        self._hold_read_failures = 0

        drift = abs(present_pos - self._last_commanded_pos_deg)
        threshold = self._drift_threshold()

        if drift > threshold:
            correction_pos = self._last_commanded_pos_deg
            self._write_gripper(correction_pos, "hold_correction")
            self._emit(
                "warning",
                "gripper_hold_correction",
                {
                    "present_deg": present_pos,
                    "commanded_deg": correction_pos,
                    "drift_deg": drift,
                    "threshold_deg": threshold,
                },
            )
            return True

        return False

    def unlock(self) -> None:
        """Transition LOCKED → UNLOCKING → UNLOCKED.

        **Safety gate**: refuses to unlock unless the injected
        ``opposite_arm_locked_fn`` callable returns ``True`` (verified LOCKED
        on the other arm).  If no predicate was injected, unlock is always
        refused.

        **Under-load behaviour**: if verification thresholds are configured
        and the present load exceeds ``jam_load_threshold_raw``, a WARNING
        event ``gripper_unlock_under_load`` is emitted before releasing.  The
        calling supervisor is responsible for deciding whether to abort or
        proceed.  The state machine proceeds with the unlock in either case
        (the warning is informational; the owner must handle it at a higher
        level).

        Raises
        ------
        :class:`IllegalTransition`
            If not currently in LOCKED state.
        :class:`BandsNotConfigured`
            In live mode when bands are null.
        :class:`UnlockRefused`
            When the opposite-arm predicate returns False or raises.
        """
        if self._state != GripperState.LOCKED:
            raise IllegalTransition(self._state, "unlock")
        self._require_bands_or_dry_run("unlock")

        # Opposite-arm holding gate (AGENTS.md invariant)
        if self._opposite_arm_locked_fn is None:
            raise UnlockRefused(
                self._role,
                "no opposite-arm holding predicate was injected; "
                "cannot verify the other arm is LOCKED",
            )
        try:
            other_locked = self._opposite_arm_locked_fn()
        except Exception as exc:  # noqa: BLE001
            raise UnlockRefused(
                self._role,
                f"opposite-arm locked predicate raised: {exc}",
            ) from exc
        if not other_locked:
            raise UnlockRefused(
                self._role,
                "opposite arm is not in LOCKED state; "
                "unlock refused to preserve tube attachment",
            )

        # Under-load warning
        if not self._dry_run:
            gripper_servo_id = self._bus._servo_id_for_joint("gripper")
            try:
                joint_state = self._bus.read_joint_state(gripper_servo_id)
                thresholds = self._verification_thresholds()
                if (
                    thresholds is not None
                    and not thresholds.unverified
                    and thresholds.jam_load_threshold_raw is not None
                    and abs(joint_state.load_raw) > thresholds.jam_load_threshold_raw
                ):
                    self._emit(
                        "warning",
                        "gripper_unlock_under_load",
                        {
                            "load_raw": joint_state.load_raw,
                            "threshold_raw": thresholds.jam_load_threshold_raw,
                            "note": (
                                "Unlocking under significant load. "
                                "Supervisor should verify safety before proceeding."
                            ),
                        },
                    )
            except Exception as exc:  # noqa: BLE001
                # Orchestrator 2026-10-24 (audit finding 6): if the gripper
                # servo cannot be read, refuse to unlock (fail closed), and
                # stay LOCKED.
                self._emit(
                    "error",
                    "gripper_unlock_load_read_failed",
                    {"error": str(exc)},
                )
                raise UnlockRefused(
                    self._role,
                    f"cannot read gripper servo state before unlock: {exc}",
                ) from exc

        prev_state = self._state
        self._state = GripperState.UNLOCKING
        self._emit(
            "info",
            "gripper_unlocking_started",
            {"prev_state": prev_state.name},
        )

        pos = self._goal_pos_unlocked()
        self._write_gripper(pos, "unlock_command")

        self._state = GripperState.UNLOCKED
        self._emit_gripper_unlock_event(GripperState.LOCKED)
        self._emit(
            "info",
            "gripper_unlocked",
            {"position_deg": pos, "dry_run": self._dry_run},
        )
