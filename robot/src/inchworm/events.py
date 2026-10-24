"""Typed event constructors for the inchworm robot session logger.

Each function produces a dict suitable for the ``payload`` argument of
:meth:`~inchworm.telemetry.SessionLogger.emit` **and** carries a fixed
``event_type`` string so callers never spell it by hand.

The payload models are frozen Pydantic models for schema stability: any field
renaming or removal breaks existing tests immediately.

Supported event families
------------------------
- **Motion** – ``motion_command_sent``: a motion target was dispatched to an arm
  or the DexArm, with the target pose/joints encoded in the payload.
- **Gripper** – ``gripper_lock`` / ``gripper_unlock``: the gripper servo
  transitioned between locked and unlocked states.
- **Policy** – ``policy_handoff_start`` / ``policy_handoff_end``: the orchestrator
  handed control to the learned insertion policy or reclaimed it.
- **Abort** – ``cycle_abort``: a crawl cycle was aborted with a reason and context.
- **Retry** – ``cycle_retry``: a recoverable error triggered a retry attempt.
- **Cycle** – ``cycle_start`` / ``cycle_end``: convenience bookends so the
  summariser can demarcate cycle boundaries.

Usage::

    from inchworm.events import motion_command, gripper_lock, cycle_start
    from inchworm.session import new_session
    from inchworm.config import load_config

    with new_session(load_config()) as ctx:
        ctx.logger.emit(**cycle_start(cycle_index=0))
        ctx.logger.emit(**motion_command(
            role="arm_left",
            target_type="joint_angles",
            target={"shoulder": 10.0, "elbow": -5.0},
        ))
        ctx.logger.emit(**gripper_lock(role="arm_left", from_state="unlocked"))
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

# ---------------------------------------------------------------------------
# Internal payload models (frozen for schema stability tests)
# ---------------------------------------------------------------------------


class _MotionPayload(BaseModel):
    model_config = ConfigDict(frozen=True)

    role: str = Field(description="arm or device role dispatching the motion")
    target_type: Literal["joint_angles", "cartesian_pose", "gcode"] = Field(
        description="how the target is expressed"
    )
    target: dict[str, Any] = Field(description="target coordinates or G-code string")
    # Optional context fields – present when known.
    speed_fraction: float | None = Field(default=None, description="0.0–1.0 speed scale")
    dry_run: bool = Field(default=False, description="True when in dry-run mode")


class _GripperPayload(BaseModel):
    model_config = ConfigDict(frozen=True)

    role: str = Field(description="arm role owning the gripper")
    from_state: Literal["locked", "unlocked", "unknown"] = Field(
        description="gripper state before this transition"
    )
    to_state: Literal["locked", "unlocked"] = Field(
        description="gripper state after this transition"
    )
    servo_id: int | None = Field(default=None, description="servo bus ID if known")


class _PolicyHandoffPayload(BaseModel):
    model_config = ConfigDict(frozen=True)

    direction: Literal["code_to_policy", "policy_to_code"] = Field(
        description="which direction control is flowing"
    )
    policy_name: str = Field(description="identifier of the learned policy")
    cycle_index: int = Field(description="zero-based crawl cycle number")
    reason: str | None = Field(
        default=None, description="human note on why handoff is occurring (policy→code)"
    )


class _AbortPayload(BaseModel):
    model_config = ConfigDict(frozen=True)

    cycle_index: int = Field(description="zero-based crawl cycle that was aborted")
    reason: str = Field(description="machine-readable abort reason code")
    detail: str | None = Field(default=None, description="optional human-readable context")
    subsystem: str = Field(description="subsystem that raised the abort")


class _RetryPayload(BaseModel):
    model_config = ConfigDict(frozen=True)

    cycle_index: int = Field(description="zero-based crawl cycle")
    attempt: int = Field(description="current attempt number (1-based)")
    max_attempts: int = Field(description="maximum allowed attempts")
    reason: str = Field(description="reason for this retry")
    subsystem: str = Field(description="subsystem requesting the retry")


class _CyclePayload(BaseModel):
    model_config = ConfigDict(frozen=True)

    cycle_index: int = Field(description="zero-based crawl cycle number")
    phase: str | None = Field(
        default=None,
        description="optional phase tag (e.g. 'approach', 'insertion', 'retract')",
    )


# ---------------------------------------------------------------------------
# Public constructors – return kwargs for SessionLogger.emit()
# ---------------------------------------------------------------------------

_EmitKwargs = dict  # type alias for clarity


def motion_command(
    *,
    role: str,
    target_type: Literal["joint_angles", "cartesian_pose", "gcode"],
    target: dict[str, Any],
    speed_fraction: float | None = None,
    dry_run: bool = False,
    severity: str = "INFO",
    subsystem: str = "motion",
) -> _EmitKwargs:
    """Return emit kwargs for a motion command dispatched to *role*.

    Parameters
    ----------
    role:
        Logical device role (e.g. ``"arm_left"``, ``"dexarm"``).
    target_type:
        ``"joint_angles"`` – keyed joint-angle dict (degrees);
        ``"cartesian_pose"`` – keyed Cartesian-coordinate dict;
        ``"gcode"`` – raw G-code string under key ``"cmd"``.
    target:
        Dict describing the motion target.
    speed_fraction:
        Optional speed scale 0.0–1.0.
    dry_run:
        Set ``True`` when the command will not cause physical motion.
    """
    payload = _MotionPayload(
        role=role,
        target_type=target_type,
        target=target,
        speed_fraction=speed_fraction,
        dry_run=dry_run,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "motion_command_sent",
        "severity": severity,
        "payload": payload.model_dump(),
    }


def gripper_lock(
    *,
    role: str,
    from_state: Literal["locked", "unlocked", "unknown"] = "unlocked",
    servo_id: int | None = None,
    subsystem: str = "gripper",
) -> _EmitKwargs:
    """Return emit kwargs for a gripper → locked transition."""
    payload = _GripperPayload(
        role=role,
        from_state=from_state,
        to_state="locked",
        servo_id=servo_id,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "gripper_lock",
        "severity": "INFO",
        "payload": payload.model_dump(),
    }


def gripper_unlock(
    *,
    role: str,
    from_state: Literal["locked", "unlocked", "unknown"] = "locked",
    servo_id: int | None = None,
    subsystem: str = "gripper",
) -> _EmitKwargs:
    """Return emit kwargs for a gripper → unlocked transition."""
    payload = _GripperPayload(
        role=role,
        from_state=from_state,
        to_state="unlocked",
        servo_id=servo_id,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "gripper_unlock",
        "severity": "INFO",
        "payload": payload.model_dump(),
    }


def policy_handoff_start(
    *,
    policy_name: str,
    cycle_index: int,
    role: str = "orchestrator",
    subsystem: str = "policy",
) -> _EmitKwargs:
    """Return emit kwargs when code hands control to the learned policy."""
    payload = _PolicyHandoffPayload(
        direction="code_to_policy",
        policy_name=policy_name,
        cycle_index=cycle_index,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "policy_handoff_start",
        "severity": "INFO",
        "payload": payload.model_dump(),
    }


def policy_handoff_end(
    *,
    policy_name: str,
    cycle_index: int,
    reason: str | None = None,
    role: str = "orchestrator",
    subsystem: str = "policy",
) -> _EmitKwargs:
    """Return emit kwargs when the policy returns control to code."""
    payload = _PolicyHandoffPayload(
        direction="policy_to_code",
        policy_name=policy_name,
        cycle_index=cycle_index,
        reason=reason,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "policy_handoff_end",
        "severity": "INFO",
        "payload": payload.model_dump(),
    }


def cycle_abort(
    *,
    cycle_index: int,
    reason: str,
    subsystem: str,
    detail: str | None = None,
    role: str = "orchestrator",
) -> _EmitKwargs:
    """Return emit kwargs when a crawl cycle is aborted."""
    payload = _AbortPayload(
        cycle_index=cycle_index,
        reason=reason,
        detail=detail,
        subsystem=subsystem,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "cycle_abort",
        "severity": "ERROR",
        "payload": payload.model_dump(),
    }


def cycle_retry(
    *,
    cycle_index: int,
    attempt: int,
    max_attempts: int,
    reason: str,
    subsystem: str,
    role: str = "orchestrator",
) -> _EmitKwargs:
    """Return emit kwargs when a recoverable error triggers a retry."""
    payload = _RetryPayload(
        cycle_index=cycle_index,
        attempt=attempt,
        max_attempts=max_attempts,
        reason=reason,
        subsystem=subsystem,
    )
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "cycle_retry",
        "severity": "WARNING",
        "payload": payload.model_dump(),
    }


def cycle_start(
    *,
    cycle_index: int,
    phase: str | None = None,
    role: str = "orchestrator",
    subsystem: str = "lifecycle",
) -> _EmitKwargs:
    """Return emit kwargs for the start of a crawl cycle."""
    payload = _CyclePayload(cycle_index=cycle_index, phase=phase)
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "cycle_start",
        "severity": "INFO",
        "payload": payload.model_dump(),
    }


def cycle_end(
    *,
    cycle_index: int,
    phase: str | None = None,
    role: str = "orchestrator",
    subsystem: str = "lifecycle",
) -> _EmitKwargs:
    """Return emit kwargs for the end of a crawl cycle."""
    payload = _CyclePayload(cycle_index=cycle_index, phase=phase)
    return {
        "role": role,
        "subsystem": subsystem,
        "event_type": "cycle_end",
        "severity": "INFO",
        "payload": payload.model_dump(),
    }
