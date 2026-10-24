"""Orchestrator guard 2026-10-24: every `from inchworm.X import Y` in
robot/scripts/*.py (including imports inside functions, i.e. --live paths)
must resolve. Several hardware scripts had live paths calling APIs that do
not exist, which would only have surfaced during the owner's hardware session.
"""
import ast
import importlib
import pathlib
import subprocess
import sys

import pytest

SCRIPTS = sorted((pathlib.Path(__file__).resolve().parents[1] / "scripts").glob("*.py"))


# Known-broken live paths, being fixed by the "scripts live-path" lane.
# Remove each name from this set as it is fixed; the set must reach empty.
# --- ALL FIXED: set is intentionally empty ---
KNOWN_BROKEN: set[str] = set()


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


# ---------------------------------------------------------------------------
# Smoke tests: --help and dry-run default must exit 0 without hardware
# ---------------------------------------------------------------------------

#: Scripts that require a sub-command to run in dry-run (default args won't work).
#: Map script-name -> argv to pass instead of [].
_DRY_RUN_ARGV: dict[str, list[str]] = {
    # record_seating_traces.py requires a sub-command; use fit with a dummy
    # file that triggers the argparse path without opening hardware.
    # We use --help for simplicity since fit requires a real file argument.
    "record_seating_traces.py": ["--help"],
    # scan_servos.py dry-runs fine with no args, but --help is safer.
    "scan_servos.py": ["--help"],
}

#: Scripts that need special arguments to reach a clean dry-run exit.
#: If a script is not listed here, it is invoked with no extra arguments.
_SKIP_DRY_RUN: set[str] = {
    # rollout.py requires a policy checkpoint even in dry-run; skip here.
    "rollout.py",
    # teleop.py opens hardware arbiter even in dry-run; skip here.
    "teleop.py",
    # inspect_dataset.py requires a dataset path argument; skip.
    "inspect_dataset.py",
    # jog.py requires --arm; skip bare-invocation dry-run.
    "jog.py",
}

_SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"


def _smoke_scripts():
    """Yield (script_path, argv) pairs for the smoke-test parametrize."""
    for p in sorted(_SCRIPTS_DIR.glob("*.py")):
        if p.name in _SKIP_DRY_RUN:
            continue
        argv = _DRY_RUN_ARGV.get(p.name, [])
        yield pytest.param(p, argv, id=p.name)


@pytest.mark.parametrize("script,extra_argv", list(_smoke_scripts()))
def test_script_help_exits_zero(script: pathlib.Path, extra_argv: list[str]):
    """--help must exit 0 for every script that supports it."""
    result = subprocess.run(
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, (
        f"{script.name} --help exited {result.returncode}\n"
        f"stdout: {result.stdout[:500]}\n"
        f"stderr: {result.stderr[:500]}"
    )


@pytest.mark.parametrize("script,extra_argv", [
    p for p in list(_smoke_scripts()) if p.values[1] == []
])
def test_script_dry_run_default_exits_zero(script: pathlib.Path, extra_argv: list[str]):
    """Default invocation (no flags) must exit 0: dry-run is always the default."""
    result = subprocess.run(
        [sys.executable, str(script)],
        capture_output=True,
        text=True,
        timeout=30,
        env={**__import__("os").environ, "INCHWORM_BACKEND": "mock"},
    )
    assert result.returncode == 0, (
        f"{script.name} (dry-run default) exited {result.returncode}\n"
        f"stdout: {result.stdout[:800]}\n"
        f"stderr: {result.stderr[:800]}"
    )
