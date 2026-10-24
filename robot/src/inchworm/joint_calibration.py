"""AA-9: Per-joint servo calibration — servo-degree ↔ joint-radian conversion.

This module is the **single conversion point** between STS3215 servo positions
(0-360 degrees) and URDF joint angles (radians).

Background
----------
The STS3215 servo reports and accepts position in degrees (0-360) where the
servo's physical zero is at the factory-set mid-point.  The URDF joint zero
is the mechanical rest pose as measured during the calibration session.  These
two zeros differ by a per-joint offset (``zero_offset_deg``), and the direction
of increasing angle may differ (``sign = +1 or -1``).

The previous implementation hardcoded ``q_rad = (servo_deg - 180.0) * π/180``
which assumed every joint has its servo zero at 180 deg and the URDF and servo
share the same angular direction.  Neither assumption is verified.

Conversion formula
------------------
Given per-joint calibration ``(zero_offset_deg, sign)``::

    joint_rad = sign * (servo_deg - zero_offset_deg) * π / 180
    servo_deg = (joint_rad / (sign * π / 180)) + zero_offset_deg
              = sign * joint_rad * 180 / π + zero_offset_deg

Fail-closed rule
----------------
If ``zero_offset_deg`` is ``None`` or the calibration block is marked
``unverified: true``, :func:`servo_deg_to_joint_rad` and
:func:`joint_rad_to_servo_deg` raise :class:`CalibrationMissingError` when
called with ``live=True``.  In dry-run mode (``live=False``) the old
``zero_offset = 180, sign = +1`` fallback is used and a WARNING is emitted.

Safety invariants (AGENTS.md)
------------------------------
* This module never opens real serial ports or cameras.
* Dry-run is the default; live motion requires ``live=True``.
* Fail-closed: null or unverified calibration blocks live conversion.
* The gripper joint is never handled here (AA-17's domain).
"""

from __future__ import annotations

import logging
import math
from typing import Optional, Sequence, Tuple

from inchworm.motion_guard import JOINT_NAMES

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Default fallback (used in dry-run when calibration is missing)
# ---------------------------------------------------------------------------

_FALLBACK_ZERO_OFFSET_DEG: float = 180.0
_FALLBACK_SIGN: int = 1


# ---------------------------------------------------------------------------
# Typed exceptions
# ---------------------------------------------------------------------------


class CalibrationMissingError(RuntimeError):
    """Raised when live conversion is attempted with null or unverified calibration.

    The operator must run the calibration procedure, populate
    ``motion_guard.servo_calibration`` in ``hardware.yaml`` with
    ``zero_offset_deg`` and ``sign`` for every joint, set ``unverified: false``,
    and then re-run with ``live=True``.
    """

    def __init__(self, joint_name: str, arm_role: str) -> None:
        self.joint_name = joint_name
        self.arm_role = arm_role
        super().__init__(
            f"AA-9 joint calibration missing for {arm_role} arm joint '{joint_name}': "
            "zero_offset_deg is null or calibration block is marked unverified=True. "
            "Populate motion_guard.servo_calibration in hardware.yaml and set "
            "unverified: false before using live mode."
        )


# ---------------------------------------------------------------------------
# Per-joint calibration parameters
# ---------------------------------------------------------------------------


class JointCalibParams:
    """Resolved calibration parameters for one joint.

    Attributes
    ----------
    zero_offset_deg:
        Servo position (degrees, 0-360) that corresponds to joint_rad = 0.
        When ``None``, calibration has not been measured.
    sign:
        +1 if increasing servo_deg → increasing joint_rad,
        -1 if increasing servo_deg → decreasing joint_rad.
    unverified:
        ``True`` when this is a placeholder value, not a measured value.
        Live conversion is blocked when ``True``.
    """

    __slots__ = ("joint_name", "zero_offset_deg", "sign", "unverified")

    def __init__(
        self,
        joint_name: str,
        zero_offset_deg: Optional[float],
        sign: int,
        unverified: bool,
    ) -> None:
        if sign not in (1, -1):
            raise ValueError(
                f"sign must be +1 or -1 for joint '{joint_name}', got {sign!r}"
            )
        self.joint_name = joint_name
        self.zero_offset_deg = zero_offset_deg
        self.sign = sign
        self.unverified = unverified


# ---------------------------------------------------------------------------
# Arm-level calibration table
# ---------------------------------------------------------------------------


class ArmCalibration:
    """Per-joint calibration table for one arm.

    Parameters
    ----------
    arm_role:
        ``"left"`` or ``"right"`` — used in error messages and logs.
    params:
        Sequence of :class:`JointCalibParams`, one per joint in
        :data:`~inchworm.motion_guard.JOINT_NAMES` order.
    """

    def __init__(self, arm_role: str, params: Sequence[JointCalibParams]) -> None:
        if len(params) != len(JOINT_NAMES):
            raise ValueError(
                f"ArmCalibration for {arm_role!r} needs {len(JOINT_NAMES)} "
                f"JointCalibParams, got {len(params)}"
            )
        self._arm_role = arm_role
        self._params: Tuple[JointCalibParams, ...] = tuple(params)

    @property
    def arm_role(self) -> str:
        return self._arm_role

    def get(self, joint_index: int) -> JointCalibParams:
        """Return calibration for joint at *joint_index* (0-4)."""
        return self._params[joint_index]

    @property
    def is_live_ok(self) -> bool:
        """True only if every joint has a non-null, verified offset."""
        return all(
            (p.zero_offset_deg is not None and not p.unverified)
            for p in self._params
        )


# ---------------------------------------------------------------------------
# Conversion functions (used by GuardedArm)
# ---------------------------------------------------------------------------


def servo_deg_to_joint_rad(
    servo_deg: float,
    calib: JointCalibParams,
    arm_role: str,
    live: bool = False,
) -> float:
    """Convert a STS3215 servo position (degrees, 0-360) to a URDF joint angle (radians).

    Parameters
    ----------
    servo_deg:
        Goal or present position in degrees (STS3215 range 0-360).
    calib:
        Per-joint calibration parameters.
    arm_role:
        Arm role for error messages (``"left"`` or ``"right"``).
    live:
        If ``True`` and calibration is null or unverified, raises
        :class:`CalibrationMissingError`.  If ``False``, falls back to
        ``zero_offset=180, sign=+1`` with a WARNING.

    Returns
    -------
    float
        Joint angle in radians.

    Raises
    ------
    CalibrationMissingError
        If ``live=True`` and calibration is null or unverified.
    """
    if calib.zero_offset_deg is None or calib.unverified:
        if live:
            raise CalibrationMissingError(calib.joint_name, arm_role)
        _LOG.warning(
            "AA-9 %s arm joint '%s': calibration null/unverified; "
            "using fallback zero_offset=%.1f, sign=+1 (DRY-RUN only).",
            arm_role, calib.joint_name, _FALLBACK_ZERO_OFFSET_DEG,
        )
        return (servo_deg - _FALLBACK_ZERO_OFFSET_DEG) * (math.pi / 180.0)

    return calib.sign * (servo_deg - calib.zero_offset_deg) * (math.pi / 180.0)


def joint_rad_to_servo_deg(
    joint_rad: float,
    calib: JointCalibParams,
    arm_role: str,
    live: bool = False,
) -> float:
    """Convert a URDF joint angle (radians) to a STS3215 servo position (degrees, 0-360).

    Parameters
    ----------
    joint_rad:
        Joint angle in radians.
    calib:
        Per-joint calibration parameters.
    arm_role:
        Arm role for error messages.
    live:
        If ``True`` and calibration is null or unverified, raises
        :class:`CalibrationMissingError`.

    Returns
    -------
    float
        Servo goal position in degrees.  Not clamped; the caller
        (``GuardedArm.write_positions_rad``) clamps to [0, 360].

    Raises
    ------
    CalibrationMissingError
        If ``live=True`` and calibration is null or unverified.
    """
    if calib.zero_offset_deg is None or calib.unverified:
        if live:
            raise CalibrationMissingError(calib.joint_name, arm_role)
        _LOG.warning(
            "AA-9 %s arm joint '%s': calibration null/unverified; "
            "using fallback zero_offset=%.1f, sign=+1 (DRY-RUN only).",
            arm_role, calib.joint_name, _FALLBACK_ZERO_OFFSET_DEG,
        )
        return joint_rad * (180.0 / math.pi) + _FALLBACK_ZERO_OFFSET_DEG

    # sign * (servo_deg - zero_offset_deg) * π/180 = joint_rad
    # servo_deg = joint_rad / (sign * π/180) + zero_offset_deg
    #           = sign * joint_rad * 180/π + zero_offset_deg
    return calib.sign * joint_rad * (180.0 / math.pi) + calib.zero_offset_deg


# ---------------------------------------------------------------------------
# Factory: build ArmCalibration from config
# ---------------------------------------------------------------------------


def build_arm_calibration(cfg_block, arm_role: str) -> ArmCalibration:
    """Build an :class:`ArmCalibration` from a config block.

    Parameters
    ----------
    cfg_block:
        ``ArmServoCalibration`` config object from
        ``motion_guard.servo_calibration.left_arm`` or
        ``motion_guard.servo_calibration.right_arm``, or ``None`` if the
        calibration block is absent.
    arm_role:
        ``"left"`` or ``"right"``.

    Returns
    -------
    ArmCalibration
        All joints will have ``unverified=True`` when ``cfg_block`` is None
        or when the block itself is ``unverified: true``.
    """
    params: list[JointCalibParams] = []

    for i, joint_name in enumerate(JOINT_NAMES):
        if cfg_block is None:
            # No calibration block at all → null + unverified
            p = JointCalibParams(
                joint_name=joint_name,
                zero_offset_deg=None,
                sign=1,
                unverified=True,
            )
            _LOG.warning(
                "AA-9 %s arm: no servo_calibration block in config for joint '%s'; "
                "using null calibration — LIVE mode blocked.",
                arm_role, joint_name,
            )
        else:
            # Field names on the config object follow the pattern:
            # <joint>_zero_offset_deg, <joint>_sign, unverified
            field_offset = f"{joint_name}_zero_offset_deg"
            field_sign = f"{joint_name}_sign"
            zero_offset = getattr(cfg_block, field_offset, None)
            sign_val = getattr(cfg_block, field_sign, 1) or 1
            block_unverified = getattr(cfg_block, "unverified", True)

            if zero_offset is None or block_unverified:
                _LOG.warning(
                    "AA-9 %s arm: servo calibration for joint '%s' is null or unverified "
                    "(zero_offset_deg=%r, unverified=%r) — LIVE mode blocked.",
                    arm_role, joint_name, zero_offset, block_unverified,
                )

            p = JointCalibParams(
                joint_name=joint_name,
                zero_offset_deg=float(zero_offset) if zero_offset is not None else None,
                sign=int(sign_val),
                unverified=(zero_offset is None or block_unverified),
            )
        params.append(p)

    return ArmCalibration(arm_role=arm_role, params=params)
