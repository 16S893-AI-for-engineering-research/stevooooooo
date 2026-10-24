"""AST-based test: verify that write_position_deg and _write_goal_position
are only called from the allowed modules.

AA-9 AC#5 (orchestrator comment 2026-10-24):
"Add an AST test that no module outside motion_guard.py, torque.py
(e-stop hold-in-place only) and servo_bus.py calls write_position_deg."

Also adds joint_calibration.py to the allow-list because it is the new
calibration helper (it does not call write_position_deg, but is in the
allowed layer).

Forbidden callers:
  Any src/inchworm/*.py module OTHER than:
    - motion_guard.py      — GuardedArm is the gatekeeper
    - joint_calibration.py — calibration helper (no servo writes at all)
    - servo_bus.py         — implements write_position_deg / _write_goal_position
    - gripper_lock.py      — AA-17 state machine (via GripperChannel)
    - torque.py            — e-stop hold-in-place (re-commands PRESENT position only)

The test fails if any other module contains a direct call to
``write_position_deg`` or ``_write_goal_position``.
"""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Allowed modules (relative names within src/inchworm/)
# ---------------------------------------------------------------------------

ALLOWED_MODULES: frozenset[str] = frozenset(
    {
        "motion_guard.py",
        "joint_calibration.py",
        "servo_bus.py",
        "gripper_lock.py",
        "torque.py",
    }
)

# Attribute names that are the guarded write methods
FORBIDDEN_ATTR_NAMES: frozenset[str] = frozenset(
    {
        "write_position_deg",
        "_write_goal_position",
    }
)


# ---------------------------------------------------------------------------
# AST visitor
# ---------------------------------------------------------------------------


class _WriteCallVisitor(ast.NodeVisitor):
    """Collect all Attribute accesses of ``FORBIDDEN_ATTR_NAMES``."""

    def __init__(self) -> None:
        self.hits: list[tuple[str, int]] = []  # (attr_name, lineno)

    def visit_Attribute(self, node: ast.Attribute) -> None:  # noqa: N802
        if node.attr in FORBIDDEN_ATTR_NAMES:
            self.hits.append((node.attr, node.lineno))
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> None:  # noqa: N802
        # Also catch bare name calls (e.g. write_position_deg(...) if imported)
        if isinstance(node.func, ast.Name) and node.func.id in FORBIDDEN_ATTR_NAMES:
            self.hits.append((node.func.id, node.lineno))
        self.generic_visit(node)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------


def _inchworm_src_dir() -> Path:
    """Return the src/inchworm directory."""
    here = Path(__file__).resolve().parent
    src = here.parent / "src" / "inchworm"
    assert src.is_dir(), f"Expected src/inchworm at {src}"
    return src


def test_no_direct_write_position_calls_outside_allowed_modules() -> None:
    """AST check: no module outside the allow-list calls write_position_deg or
    _write_goal_position directly.

    This enforces that all position writes go through GuardedArm (AA-9 AC#5).

    Allowed modules (may call these methods directly):
    - motion_guard.py  — GuardedArm delegates to bus._write_goal_position
    - joint_calibration.py — no servo writes, but in the allowed layer
    - servo_bus.py     — defines and calls these methods internally
    - gripper_lock.py  — AA-17, uses GripperChannel path only
    - torque.py        — e-stop hold (re-commands PRESENT position only)

    All other src/inchworm/*.py modules must NOT call these methods.
    """
    src_dir = _inchworm_src_dir()
    violations: list[str] = []

    for py_file in sorted(src_dir.glob("*.py")):
        module_name = py_file.name
        if module_name in ALLOWED_MODULES:
            continue  # skip allowed modules

        source = py_file.read_text(encoding="utf-8")
        try:
            tree = ast.parse(source, filename=str(py_file))
        except SyntaxError as exc:
            violations.append(f"{module_name}: SyntaxError: {exc}")
            continue

        visitor = _WriteCallVisitor()
        visitor.visit(tree)

        for attr_name, lineno in visitor.hits:
            violations.append(
                f"{module_name}:{lineno} — forbidden call to '{attr_name}' "
                f"(must route through GuardedArm in motion_guard.py)"
            )

    if violations:
        formatted = "\n".join(f"  {v}" for v in violations)
        pytest.fail(
            textwrap.dedent(
                f"""\
                AA-9 AC#5 VIOLATION: {len(violations)} module(s) call write_position_deg
                or _write_goal_position outside the allowed layer.
                Route all writes through GuardedArm.write_position_deg.

                Violations:
                {formatted}

                Allowed modules (may call these methods):
                {', '.join(sorted(ALLOWED_MODULES))}
                """
            )
        )
