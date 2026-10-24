"""Orchestrator guard 2026-10-24: every `from inchworm.X import Y` in
robot/scripts/*.py (including imports inside functions, i.e. --live paths)
must resolve. Several hardware scripts had live paths calling APIs that do
not exist, which would only have surfaced during the owner's hardware session.
"""
import ast
import importlib
import pathlib

import pytest

SCRIPTS = sorted((pathlib.Path(__file__).resolve().parents[1] / "scripts").glob("*.py"))


# Known-broken live paths, being fixed by the "scripts live-path" lane.
# Remove each name from this set as it is fixed; the set must reach empty.
KNOWN_BROKEN = {
    "approach_repeatability.py", "calibrate_arm_base.py", "fk_verify.py",
    "fold_cycle.py", "hand_eye.py", "level_scan.py", "measure_limits.py",
    "record_seating_traces.py",
}


@pytest.mark.parametrize(
    "script",
    [pytest.param(s, marks=pytest.mark.xfail(strict=True, reason="live path uses non-existent API (tracked)"))
     if s.name in KNOWN_BROKEN else s for s in SCRIPTS],
    ids=lambda p: p.name,
)
def test_script_inchworm_imports_resolve(script):
    tree = ast.parse(script.read_text())
    missing = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith("inchworm"):
            mod = importlib.import_module(node.module)
            for alias in node.names:
                if alias.name != "*" and not hasattr(mod, alias.name):
                    missing.append(f"{node.module}.{alias.name}")
    assert not missing, f"{script.name} imports non-existent names: {missing}"
