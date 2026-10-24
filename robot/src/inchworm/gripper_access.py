"""Gripper-access capability token for AA-17.

This module defines the module-private sentinel ``_GRIPPER_TOKEN`` that gates
access to :meth:`inchworm.servo_bus.ArmServoBus.gripper_channel`.

**Import policy**: this module MUST be imported only by the AA-17 gripper lock
state machine (``inchworm/gripper_lock*.py``).  Any other import is a policy
violation detectable by the test in ``test_servo_bus.py``
(``TestGripperCapability::test_no_token_import_outside_aa17``).

Rationale: naming conventions alone (``_write_gripper_position_deg``) are
insufficient because any caller that knows the mangled name can call through.
A capability token forces an explicit import that is statically scannable.
The token is a module-level singleton object; equality and identity checks both
work because there is only one instance.
"""

from __future__ import annotations

# The sentinel is intentionally opaque — its type is not exported.
class _GripperTokenType:
    """Opaque singleton sentinel type.  Do not instantiate outside this module."""

    __slots__ = ()

    def __repr__(self) -> str:  # pragma: no cover
        return "<GripperToken>"


#: The one-and-only capability token.  Pass this to
#: :meth:`~inchworm.servo_bus.ArmServoBus.gripper_channel` to obtain a
#: :class:`~inchworm.servo_bus.GripperChannel`.
#:
#: **Importable only from** ``inchworm.gripper_access``.
#: Downstream code that imports it from anywhere else violates the policy.
_GRIPPER_TOKEN: _GripperTokenType = _GripperTokenType()
