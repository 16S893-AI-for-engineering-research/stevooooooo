"""Tests for scripts/fault_inject_hw.py (AA-28 hardware fault-injection script).

Coverage
--------
* --help exits 0.
* Default invocation (dry-run, no flags) exits 0.
* Dry-run walks Procedure A (servo fault) and prints its steps.
* Dry-run walks Procedure B (USB camera loss) and prints its steps.
* FaultResponder is called with the correct RecoveryFaultKind for each procedure.
* The JSON report is written to the session artifacts directory.
* No real serial ports or cameras are opened in dry-run mode.

All tests use subprocess or function imports only — no real hardware.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_ROBOT_ROOT = Path(__file__).resolve().parents[1]
_SCRIPTS_DIR = _ROBOT_ROOT / "scripts"
_SCRIPT = _SCRIPTS_DIR / "fault_inject_hw.py"

sys.path.insert(0, str(_ROBOT_ROOT / "src"))


def _run_script(*args: str, timeout: int = 30) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(_SCRIPT), *args],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


# ---------------------------------------------------------------------------
# Subprocess smoke tests
# ---------------------------------------------------------------------------


class TestHelpAndDryRunSmoke:
    """--help and bare invocation (dry-run default) must exit 0."""

    def test_help_exits_zero(self):
        result = _run_script("--help")
        assert result.returncode == 0, (
            f"--help exited {result.returncode}\n"
            f"stdout: {result.stdout[:400]}\n"
            f"stderr: {result.stderr[:400]}"
        )
        assert "AA-28" in result.stdout or "fault" in result.stdout.lower()

    def test_dry_run_default_exits_zero(self):
        """Default invocation (no flags) must exit 0: dry-run is the default."""
        result = _run_script()
        assert result.returncode == 0, (
            f"dry-run exited {result.returncode}\n"
            f"stdout: {result.stdout[:600]}\n"
            f"stderr: {result.stderr[:600]}"
        )

    def test_dry_run_prints_procedure_a_steps(self):
        """Dry-run --procedure servo must print Procedure A operator steps."""
        result = _run_script("--procedure", "servo")
        assert result.returncode == 0
        combined = result.stdout + result.stderr
        assert "PROCEDURE A" in combined
        assert "STEP A1" in combined
        assert "unplug" in combined.lower()

    def test_dry_run_prints_procedure_b_steps(self):
        """Dry-run --procedure usb_camera must print Procedure B operator steps."""
        result = _run_script("--procedure", "usb_camera")
        assert result.returncode == 0
        combined = result.stdout + result.stderr
        assert "PROCEDURE B" in combined
        assert "STEP B1" in combined
        assert "USB" in combined

    def test_dry_run_both_procedures_walks_both(self):
        """Dry-run --procedure both must walk Procedure A and B."""
        result = _run_script("--procedure", "both")
        assert result.returncode == 0
        combined = result.stdout + result.stderr
        assert "PROCEDURE A" in combined
        assert "PROCEDURE B" in combined

    def test_dry_run_writes_json_report(self):
        """Dry-run must write a JSON report with the expected schema."""
        result = _run_script()
        assert result.returncode == 0
        combined = result.stdout + result.stderr
        # Extract report path from output lines
        report_path = None
        for line in combined.splitlines():
            if "fault_inject_report.json" in line:
                for tok in line.split():
                    if "fault_inject_report.json" in tok:
                        report_path = Path(tok)
                        break
            if report_path:
                break
        assert report_path is not None, (
            f"Report path not found in combined output.\n{combined[:600]}"
        )
        assert report_path.exists(), f"Report file not found: {report_path}"
        data = json.loads(report_path.read_text())
        assert data["aa_task"] == "AA-28"
        assert data["dry_run"] is True
        assert isinstance(data["outcomes"], list)
        assert len(data["outcomes"]) == 2
        assert all(o["succeeded"] for o in data["outcomes"])
        assert data["all_succeeded"] is True


# ---------------------------------------------------------------------------
# Unit tests: procedure functions imported directly
# ---------------------------------------------------------------------------

# Import the procedure functions and dataclasses from the script by
# executing it as a module registered under a stable name in sys.modules.
import importlib.util as _ilu
import types as _types


def _import_script_module():
    """Import fault_inject_hw.py as a proper sys.modules entry.

    Registers it under 'fault_inject_hw' so @dataclass can resolve the
    module dict (needed by Python 3.12's dataclass implementation).
    """
    mod_name = "fault_inject_hw"
    if mod_name in sys.modules:
        return sys.modules[mod_name]
    spec = _ilu.spec_from_file_location(mod_name, _SCRIPT)
    mod = _ilu.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


# Load once at collection time; failures here surface as collection errors.
_hw_mod = _import_script_module()


class TestProcedureFunctions:
    """Unit tests for run_procedure_a/b using mock FaultResponder."""

    def test_procedure_a_dry_run_calls_mid_advance_fault(self):
        """Procedure A dry-run calls FaultResponder with MID_ADVANCE_SERVO_FAULT."""
        from inchworm.recovery import RecoveryFaultKind, RecoveryResponse

        mock_resp = MagicMock()
        mock_resp.respond.return_value = (RecoveryResponse.ESTOP_HOLD_AND_HALT, True)

        outcome = _hw_mod.run_procedure_a_servo_fault(
            dry_run=True,
            responder=mock_resp,
        )

        mock_resp.respond.assert_called_once()
        call_kwargs = mock_resp.respond.call_args.kwargs
        assert call_kwargs["fault_kind"] == RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT
        assert call_kwargs["arm"] == "arm_left"
        assert outcome.succeeded is True
        assert outcome.dry_run is True
        assert outcome.fault_kind == RecoveryFaultKind.MID_ADVANCE_SERVO_FAULT.value
        assert outcome.actual_response == RecoveryResponse.ESTOP_HOLD_AND_HALT.value

    def test_procedure_b_dry_run_calls_usb_device_loss(self):
        """Procedure B dry-run calls FaultResponder with USB_DEVICE_LOSS."""
        from inchworm.recovery import RecoveryFaultKind, RecoveryResponse

        mock_resp = MagicMock()
        mock_resp.respond.return_value = (RecoveryResponse.ESTOP_HOLD_AND_HALT, True)

        outcome = _hw_mod.run_procedure_b_usb_camera_loss(
            dry_run=True,
            responder=mock_resp,
        )

        mock_resp.respond.assert_called_once()
        call_kwargs = mock_resp.respond.call_args.kwargs
        assert call_kwargs["fault_kind"] == RecoveryFaultKind.USB_DEVICE_LOSS
        assert outcome.succeeded is True
        assert outcome.dry_run is True
        assert outcome.fault_kind == RecoveryFaultKind.USB_DEVICE_LOSS.value
        assert outcome.actual_response == RecoveryResponse.ESTOP_HOLD_AND_HALT.value

    def test_procedure_a_expected_response_is_estop_hold_and_halt(self):
        from inchworm.recovery import RecoveryFaultKind, RecoveryResponse

        mock_resp = MagicMock()
        mock_resp.respond.return_value = (RecoveryResponse.ESTOP_HOLD_AND_HALT, True)
        outcome = _hw_mod.run_procedure_a_servo_fault(dry_run=True, responder=mock_resp)
        assert outcome.expected_response == RecoveryResponse.ESTOP_HOLD_AND_HALT.value

    def test_procedure_b_expected_response_is_estop_hold_and_halt(self):
        from inchworm.recovery import RecoveryFaultKind, RecoveryResponse

        mock_resp = MagicMock()
        mock_resp.respond.return_value = (RecoveryResponse.ESTOP_HOLD_AND_HALT, True)
        outcome = _hw_mod.run_procedure_b_usb_camera_loss(dry_run=True, responder=mock_resp)
        assert outcome.expected_response == RecoveryResponse.ESTOP_HOLD_AND_HALT.value

    def test_procedure_a_procedure_id(self):
        from inchworm.recovery import RecoveryResponse

        mock_resp = MagicMock()
        mock_resp.respond.return_value = (RecoveryResponse.ESTOP_HOLD_AND_HALT, True)
        outcome = _hw_mod.run_procedure_a_servo_fault(dry_run=True, responder=mock_resp)
        assert outcome.procedure_id == "A_servo_fault"

    def test_procedure_b_procedure_id(self):
        from inchworm.recovery import RecoveryResponse

        mock_resp = MagicMock()
        mock_resp.respond.return_value = (RecoveryResponse.ESTOP_HOLD_AND_HALT, True)
        outcome = _hw_mod.run_procedure_b_usb_camera_loss(dry_run=True, responder=mock_resp)
        assert outcome.procedure_id == "B_usb_camera_loss"


# ---------------------------------------------------------------------------
# Unit tests: main() dry-run with tmp_path session root
# ---------------------------------------------------------------------------


class TestMainDryRun:
    """main() dry-run with a tmp_path session root to capture artifacts."""

    def _run_main(self, tmp_path, *extra_args):
        from inchworm.session import new_session as _real_new_session

        session_dir_holder = []

        def _capturing(cfg, **kw):
            ctx = _real_new_session(cfg, session_root=tmp_path)
            session_dir_holder.append(ctx.artifact_dir)
            return ctx

        with patch.object(_hw_mod, "new_session", side_effect=_capturing):
            rc = _hw_mod.main(list(extra_args))
        return rc, session_dir_holder

    def test_main_dry_run_both_returns_zero(self, tmp_path):
        rc, _ = self._run_main(tmp_path, "--procedure", "both")
        assert rc == 0

    def test_main_dry_run_servo_only_returns_zero(self, tmp_path):
        rc, _ = self._run_main(tmp_path, "--procedure", "servo")
        assert rc == 0

    def test_main_dry_run_usb_camera_only_returns_zero(self, tmp_path):
        rc, _ = self._run_main(tmp_path, "--procedure", "usb_camera")
        assert rc == 0

    def test_main_dry_run_report_schema(self, tmp_path):
        """Report JSON has the required top-level keys and correct values."""
        rc, dirs = self._run_main(tmp_path)
        assert rc == 0
        assert dirs, "new_session was not called"
        report_path = dirs[0] / "fault_inject_report.json"
        assert report_path.exists(), f"Report not found at {report_path}"
        data = json.loads(report_path.read_text())
        assert data["script"] == "fault_inject_hw.py"
        assert data["aa_task"] == "AA-28"
        assert data["dry_run"] is True
        assert isinstance(data["outcomes"], list)
        assert len(data["outcomes"]) == 2
        for o in data["outcomes"]:
            assert o["succeeded"] is True
            assert o["expected_response"] == "estop_hold_and_halt"
            assert o["actual_response"] == "estop_hold_and_halt"

    def test_main_dry_run_all_succeeded_true(self, tmp_path):
        """FaultInjectReport.all_succeeded is True when both procedures pass."""
        rc, dirs = self._run_main(tmp_path, "--procedure", "both")
        assert rc == 0
        data = json.loads((dirs[0] / "fault_inject_report.json").read_text())
        assert data["all_succeeded"] is True
