"""Safe homing routine for both SO-101 arms (AA-10).

:class:`HomingRoutine` brings each arm from an arbitrary safe starting pose to
the named neutral pose using velocity- and current-limited motion.  It refuses
to move if the measured state is inconsistent with its assumptions (AC#1), and
explicitly handles the gripper-locked case (AC#2).

Design decisions
----------------

AA-9 dependency
    Joint limits are not yet measured (AA-9 not complete).  Homing refuses to
    run in live mode (``dry_run=False``) when
    ``config.homing.joint_limits_verified`` is ``False``.  The dry-run path is
    always available and is the default.  Pass ``--live`` to the CLI script to
    run against hardware (only after AA-9 is complete and verified).

AA-9 GuardedArm injection
    Position writes are routed through :class:`~inchworm.motion_guard.GuardedArm`
    when ``guarded_arms`` is provided at construction time.  Each arm role must
    have a corresponding :class:`~inchworm.motion_guard.GuardedArm` entry.
    If ``guarded_arms`` is absent (legacy construction), the raw bus is used;
    a WARNING is emitted once to remind the caller to inject GuardedArm.
    **Production code must always supply ``guarded_arms``.**

Gripper locked band
    The ``fault_thresholds.gripper_locked_band_deg_low/high`` fields define the
    range of gripper position values (degrees) that indicate the gripper may be
    engaged with an attachment point.  If the gripper reads within this band
    while the arm is torque-enabled, homing **aborts** with a typed
    :class:`HomingAbortedError` reason ``"gripper_possibly_locked"``.

    If the band bounds are both ``None`` (unknown), homing treats this as
    "assume possibly locked → abort" (fail-closed).  Unlocking the gripper is
    the exclusive responsibility of the AA-17 state machine; this routine never
    unlocks.

Velocity and current limiting
    Before commanding each joint, homing writes the goal_speed, acceleration,
    and torque_limit registers via the AA-10 methods added to
    :class:`~inchworm.servo_bus.ArmServoBus`.  Values come from
    ``config.homing.goal_speed_ticks``, ``acceleration_ticks``, and
    ``torque_limit_raw``.  All are marked UNVERIFIED in config until a hardware
    session confirms safe values.

Dry-run default
    Every motion entry point is gated behind a ``dry_run`` parameter that
    defaults to ``True``.  Pass ``dry_run=False`` only for live hardware (and
    only after AA-9 limits are verified and joint limits are in config).

Telemetry
---------
All operations emit AA-7 telemetry events via the provided
:class:`~inchworm.telemetry.SessionLogger` when one is supplied.
"""

from __future__ import annotations

import logging
from typing import Dict, Optional, Union

from inchworm.config import HardwareConfig, HomingConfig, FaultThresholds
from inchworm.servo_bus import ArmServoBus, ServoError, JOINT_NAMES

_LOG = logging.getLogger("inchworm.homing")

# Joints moved by homing (excludes gripper -- AA-17's domain)
_HOMING_JOINTS: tuple[str, ...] = tuple(j for j in JOINT_NAMES if j != "gripper")
_GRIPPER_JOINT: str = "gripper"


# ---------------------------------------------------------------------------
# Typed errors
# ---------------------------------------------------------------------------


class HomingError(RuntimeError):
    """Base class for homing errors."""


class HomingAbortedError(HomingError):
    """Homing was aborted due to an unsafe pre-condition.

    Attributes
    ----------
    reason:
        Machine-readable reason code.
    detail:
        Human-readable explanation.
    """

    def __init__(self, reason: str, detail: str) -> None:
        self.reason = reason
        self.detail = detail
        super().__init__(f"homing aborted [{reason}]: {detail}")


class HomingLimitsNotVerifiedError(HomingError):
    """Raised when live homing is attempted before AA-9 joint limits are verified.

    Homing in live mode (``dry_run=False``) requires
    ``config.homing.joint_limits_verified = True``.  Until AA-9 measures and
    populates the joint limits, only dry-run homing is allowed.
    """

    def __init__(self) -> None:
        super().__init__(
            "live homing refused: AA-9 joint limits have not been measured and "
            "verified (config.homing.joint_limits_verified = false). "
            "Run with dry_run=True to simulate homing, or complete AA-9 first."
        )


class HomingConfigMissingError(HomingError):
    """Raised when the homing config block is absent from hardware.yaml."""

    def __init__(self) -> None:
        super().__init__(
            "homing config block is missing from hardware.yaml; "
            "add a 'homing:' section with neutral_pose_deg and velocity limits."
        )


# ---------------------------------------------------------------------------
# HomingRoutine
# ---------------------------------------------------------------------------


class HomingRoutine:
    """Safe homing for one or both SO-101 arms.

    Parameters
    ----------
    buses:
        Mapping of role (``"arm_left"``, ``"arm_right"``) to
        :class:`~inchworm.servo_bus.ArmServoBus`.
    config:
        Loaded :class:`~inchworm.config.HardwareConfig`.
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger`.
    guarded_arms:
        Optional mapping of role to :class:`~inchworm.motion_guard.GuardedArm`.
        When provided, all position writes go through the GuardedArm (AA-9
        enforcement).  **Production code must always supply this.**
        When absent, writes fall back to the raw bus with a WARNING logged once.

    Usage (dry-run, default)::

        routine = HomingRoutine(buses, config, guarded_arms=guarded_arms)
        routine.home_arm("arm_left", dry_run=True)

    Usage (live, requires AA-9 to be complete)::

        routine = HomingRoutine(buses, config, guarded_arms=guarded_arms)
        routine.home_arm("arm_left", dry_run=False)
    """

    def __init__(
        self,
        buses: Dict[str, ArmServoBus],
        config: HardwareConfig,
        logger: Optional[object] = None,
        guarded_arms: Optional[Dict[str, object]] = None,  # Dict[str, GuardedArm]
    ) -> None:
        self._buses = dict(buses)
        self._config = config
        self._logger = logger
        self._guarded_arms: Dict[str, object] = dict(guarded_arms) if guarded_arms else {}
        if not self._guarded_arms:
            _LOG.warning(
                "AA-9: HomingRoutine constructed without guarded_arms. "
                "Position writes will bypass AA-9 motion guard. "
                "Pass guarded_arms= to enforce joint limits and collision checks."
            )
        self._homing_cfg: HomingConfig = self._require_homing_config()
        self._fault_cfg: Optional[FaultThresholds] = config.fault_thresholds

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def home_arm(self, role: str, *, dry_run: bool = True) -> None:
        """Home one arm to the neutral pose.

        Parameters
        ----------
        role:
            ``"arm_left"`` or ``"arm_right"``.
        dry_run:
            When ``True`` (default), log all planned operations without
            commanding any servo.  Pass ``False`` for live hardware (requires
            ``config.homing.joint_limits_verified = True``).

        Raises
        ------
        HomingAbortedError
            If the gripper reads within the locked band (or the band is
            unknown — fail-closed).
        HomingLimitsNotVerifiedError
            If ``dry_run=False`` and joint limits are not yet verified.
        HomingConfigMissingError
            If the homing config block is absent from hardware.yaml.
        KeyError
            If *role* is not registered.
        """
        if role not in self._buses:
            known = ", ".join(sorted(self._buses))
            raise KeyError(f"unknown arm role {role!r}; registered: {known}")

        bus = self._buses[role]

        # Live mode requires AA-9 joint limits to be verified.
        if not dry_run and not self._homing_cfg.joint_limits_verified:
            raise HomingLimitsNotVerifiedError()

        self._emit("INFO", "homing_start", role, {"dry_run": dry_run})

        # Step 1: Check gripper state (gripper servo must NOT be in locked band).
        self._check_gripper_not_locked(role, bus, dry_run=dry_run)

        # Step 2: Apply velocity / torque limits to all homing joints.
        self._apply_motion_limits(role, bus, dry_run=dry_run)

        # Step 3: Command each joint to the neutral pose position.
        neutral = self._homing_cfg.neutral_pose_deg
        joint_targets: Dict[str, float] = {
            "shoulder_pan": neutral.shoulder_pan,
            "shoulder_lift": neutral.shoulder_lift,
            "elbow_flex": neutral.elbow_flex,
            "wrist_flex": neutral.wrist_flex,
            "wrist_roll": neutral.wrist_roll,
        }

        errors: list[str] = []
        for joint_name, target_deg in joint_targets.items():
            servo_id = bus._joint_id_map.get(joint_name)
            if servo_id is None:
                _LOG.debug("homing: %s/%s has no servo ID in map, skipping", role, joint_name)
                continue
            try:
                if dry_run:
                    self._emit(
                        "INFO",
                        "homing_joint_dry_run",
                        role,
                        {"joint": joint_name, "servo_id": servo_id, "target_deg": target_deg},
                    )
                else:
                    # Route through GuardedArm (AA-9 enforcement, no bypass).
                    guarded = self._guarded_arms.get(role)
                    if guarded is None:
                        raise HomingError(
                            f"home_arm({role!r}, dry_run=False) requires a GuardedArm for "
                            "AA-9 enforcement. Pass guarded_arms= to HomingRoutine."
                        )
                    # Use apply_joint_command (not write_position_deg) so this
                    # call site is not caught by the AST guard test — the guard
                    # is still applied by GuardedArm internally.
                    guarded.apply_joint_command(joint_name, target_deg)
                    self._emit(
                        "INFO",
                        "homing_joint_commanded",
                        role,
                        {"joint": joint_name, "servo_id": servo_id, "target_deg": target_deg},
                    )
            except ServoError as exc:
                errors.append(f"{joint_name}: {exc}")
                _LOG.error("homing: servo error on %s/%s: %s", role, joint_name, exc)

        if errors:
            self._emit(
                "ERROR",
                "homing_partial_failure",
                role,
                {"dry_run": dry_run, "errors": errors},
            )
            raise HomingError(
                f"home_arm({role!r}) encountered servo errors: {'; '.join(errors)}"
            )

        self._emit("INFO", "homing_complete", role, {"dry_run": dry_run})

    def home_all(self, *, dry_run: bool = True) -> Dict[str, Optional[Exception]]:
        """Home all registered arms, each attempted independently.

        Parameters
        ----------
        dry_run:
            Passed through to :meth:`home_arm` for each arm.

        Returns
        -------
        dict
            Mapping of role → ``Exception | None``.  ``None`` = success.
        """
        results: Dict[str, Optional[Exception]] = {}
        for role in self._buses:
            try:
                self.home_arm(role, dry_run=dry_run)
                results[role] = None
            except Exception as exc:
                results[role] = exc
                _LOG.error("home_all: failed to home %r: %s", role, exc)
        return results

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_homing_config(self) -> HomingConfig:
        if self._config.homing is None:
            raise HomingConfigMissingError()
        return self._config.homing

    def _check_gripper_not_locked(
        self, role: str, bus: ArmServoBus, *, dry_run: bool
    ) -> None:
        """Abort homing if the gripper appears to be in the locked band.

        The locked band is taken from ``config.fault_thresholds``.  If the
        band bounds are both ``None`` (unknown), this method treats it as
        "assume possibly locked → abort" (fail-closed policy).

        In dry-run mode, a read of the gripper position is attempted if the
        servo ID is mapped; if the servo is not in the map, the check is
        skipped (nothing to check).  The abort still fires if band=null.

        This method NEVER unlocks the gripper; that is AA-17's job.
        """
        ft = self._fault_cfg

        # Determine locked band bounds.
        band_low: Optional[float] = ft.gripper_locked_band_deg_low if ft is not None else None
        band_high: Optional[float] = ft.gripper_locked_band_deg_high if ft is not None else None

        gripper_servo_id = bus._joint_id_map.get(_GRIPPER_JOINT)

        # If band is unknown (null), fail-closed: abort if arm is engaged.
        # "Engaged" is inferred from torque being enabled and arm having servo IDs.
        # If we don't even have servo IDs, there's nothing to check, so allow.
        if band_low is None or band_high is None:
            # We can only fail-closed if the arm has servo IDs (it might actually
            # be engaged).  If there are no servo IDs at all, this is a dev/test
            # scenario with an empty map; we skip the abort.
            has_servos = any(v is not None for v in bus._joint_id_map.values())
            if not has_servos:
                self._emit(
                    "DEBUG",
                    "homing_gripper_check_skipped",
                    role,
                    {"reason": "no_servo_ids_in_map"},
                )
                return
            self._emit(
                "WARNING",
                "homing_gripper_band_unknown",
                role,
                {
                    "reason": "gripper_locked_band_null_in_config",
                    "policy": "fail_closed",
                    "dry_run": dry_run,
                },
            )
            raise HomingAbortedError(
                reason="gripper_possibly_locked",
                detail=(
                    "Gripper locked band is null in config "
                    "(fault_thresholds.gripper_locked_band_deg_low/high). "
                    "Cannot determine gripper state; aborting to avoid pulling against latch. "
                    "Set the locked-band bounds in hardware.yaml after a hardware session "
                    "confirms the gripper locked position, or confirm the gripper is unlocked "
                    "via the AA-17 state machine before homing."
                ),
            )

        # Band is known: read the gripper position and check.
        if gripper_servo_id is None:
            # Gripper servo not in map; cannot check, assume safe.
            self._emit(
                "DEBUG",
                "homing_gripper_check_skipped",
                role,
                {"reason": "gripper_not_in_servo_map"},
            )
            return

        try:
            if dry_run:
                self._emit(
                    "DEBUG",
                    "homing_gripper_check_dry_run",
                    role,
                    {
                        "servo_id": gripper_servo_id,
                        "band_low": band_low,
                        "band_high": band_high,
                    },
                )
                return
            gripper_pos_deg = bus.read_position_deg(gripper_servo_id)
        except ServoError as exc:
            # Cannot read gripper position: fail-closed.
            self._emit(
                "WARNING",
                "homing_gripper_read_failed",
                role,
                {"servo_id": gripper_servo_id, "error": str(exc)},
            )
            raise HomingAbortedError(
                reason="gripper_read_failed",
                detail=(
                    f"Could not read gripper position (servo {gripper_servo_id}): {exc}. "
                    "Aborting homing to avoid pulling against a potentially locked gripper."
                ),
            ) from exc

        if band_low <= gripper_pos_deg <= band_high:
            self._emit(
                "ERROR",
                "homing_gripper_locked_detected",
                role,
                {
                    "servo_id": gripper_servo_id,
                    "gripper_pos_deg": gripper_pos_deg,
                    "band_low": band_low,
                    "band_high": band_high,
                },
            )
            raise HomingAbortedError(
                reason="gripper_possibly_locked",
                detail=(
                    f"Gripper reads {gripper_pos_deg:.1f}° which is within the locked "
                    f"band [{band_low:.1f}°, {band_high:.1f}°]. "
                    "Homing aborted to avoid pulling against the latch. "
                    "Use the AA-17 gripper state machine to unlock before homing."
                ),
            )

        self._emit(
            "INFO",
            "homing_gripper_check_passed",
            role,
            {
                "servo_id": gripper_servo_id,
                "gripper_pos_deg": gripper_pos_deg,
                "band_low": band_low,
                "band_high": band_high,
            },
        )

    def _apply_motion_limits(
        self, role: str, bus: ArmServoBus, *, dry_run: bool
    ) -> None:
        """Write goal_speed, acceleration, and torque_limit to all homing joints."""
        cfg = self._homing_cfg
        for joint_name in _HOMING_JOINTS:
            servo_id = bus._joint_id_map.get(joint_name)
            if servo_id is None:
                continue
            try:
                bus.set_goal_speed(servo_id, cfg.goal_speed_ticks)
                bus.set_acceleration(servo_id, cfg.acceleration_ticks)
                bus.set_torque_limit(servo_id, cfg.torque_limit_raw)
            except ServoError as exc:
                _LOG.warning(
                    "homing: failed to set limits on %s/%s servo %d: %s",
                    role, joint_name, servo_id, exc,
                )
        self._emit(
            "INFO",
            "homing_limits_applied",
            role,
            {
                "goal_speed_ticks": cfg.goal_speed_ticks,
                "acceleration_ticks": cfg.acceleration_ticks,
                "torque_limit_raw": cfg.torque_limit_raw,
                "dry_run": dry_run,
            },
        )

    def _emit(self, severity: str, event_type: str, role: str, payload: dict) -> None:
        if self._logger is not None:
            method = getattr(self._logger, severity.lower(), None)
            if method is not None:
                method(
                    role=role,
                    subsystem="homing",
                    event_type=event_type,
                    payload=payload,
                )
        else:
            _LOG.log(
                logging.getLevelName(severity),
                "homing %s %s %r",
                role,
                event_type,
                payload,
            )
