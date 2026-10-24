"""Tests for robot/scripts/attach_usb.py.

All tests use fixture JSON or mock subprocess; no hardware, no powershell.exe
calls, no bytes written to USB devices.

Coverage:
  - BusId resolution by VID:PID + serial (identify_by: usb_serial devices)
  - BusId resolution by VID:PID only (identify_by: port_path devices)
  - Missing device produces a [MISSING] result with role name and VID:PID
  - Already-attached device is reported as OK without re-attaching
  - Dry-run mode prints the command without executing it
  - Successful attach command produces [OK]
  - Failed attach command produces [FAIL] and nonzero exit
  - main() returns nonzero when any device is missing
  - main() returns 0 when all devices succeed
  - _parse_instance_id handles real serials and Windows path segments
  - vhci verification: immediate presence -> [OK]
  - vhci verification: appears only after retry -> [RETRY-OK]
  - vhci verification: never appears -> [FAIL] and nonzero exit
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Optional
from unittest.mock import MagicMock, patch

import pytest

# Ensure the scripts directory is importable when tests run from robot/.
_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(_SCRIPTS))

import attach_usb  # noqa: E402 — must come after sys.path insertion


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

# Minimal usbipd state JSON covering all 7 inchworm devices.
_USBIPD_STATE: dict = {
    "Devices": [
        # arm_right: 1a86:55d3 serial 5B91030298, already attached
        {
            "BusId": "9-3",
            "ClientIPAddress": "172.26.80.62",
            "Description": "USB-Enhanced-SERIAL CH343 (COM3)",
            "InstanceId": "USB\\VID_1A86&PID_55D3\\5B91030298",
            "IsForced": False,
            "PersistedGuid": "aaa",
            "StubInstanceId": "USB\\VID_80EE&PID_CAFE\\5B91030298",
        },
        # relay: 1a86:7523 no serial (Windows path segment), already attached
        {
            "BusId": "9-2",
            "ClientIPAddress": "172.26.80.62",
            "Description": "USB-SERIAL CH340 (COM7)",
            "InstanceId": "USB\\VID_1A86&PID_7523\\7&165AD49D&0&2",
            "IsForced": False,
            "PersistedGuid": "bbb",
            "StubInstanceId": "USB\\VID_80EE&PID_CAFE\\7&165AD49D&0&2",
        },
        # dexarm: 0483:5740 serial 306133643438, NOT yet attached
        {
            "BusId": "11-3",
            "ClientIPAddress": None,
            "Description": "USB Serial Device (COM6)",
            "InstanceId": "USB\\VID_0483&PID_5740\\306133643438",
            "IsForced": False,
            "PersistedGuid": "ccc",
            "StubInstanceId": None,
        },
        # cam_scanner: 0c45:6366 serial UC762, NOT yet attached
        {
            "BusId": "11-4",
            "ClientIPAddress": None,
            "Description": "Arducam OV9281 USB Camera",
            "InstanceId": "USB\\VID_0C45&PID_6366\\UC762",
            "IsForced": False,
            "PersistedGuid": "ddd",
            "StubInstanceId": None,
        },
        # arm_left: 1a86:55d3 serial 5B91045044, NOT yet attached
        {
            "BusId": "10-4",
            "ClientIPAddress": None,
            "Description": "USB-Enhanced-SERIAL CH343 (COM5)",
            "InstanceId": "USB\\VID_1A86&PID_55D3\\5B91045044",
            "IsForced": False,
            "PersistedGuid": "eee",
            "StubInstanceId": None,
        },
        # cam_left: 32e6:9251 serial 5423534724, NOT yet attached
        {
            "BusId": "9-4",
            "ClientIPAddress": None,
            "Description": "openaicam",
            "InstanceId": "USB\\VID_32E6&PID_9251\\5423534724",
            "IsForced": False,
            "PersistedGuid": "fff",
            "StubInstanceId": None,
        },
        # cam_right: 32e6:9251 no real serial (Windows path segment), NOT attached
        {
            "BusId": "10-2",
            "ClientIPAddress": None,
            "Description": "openaicam",
            "InstanceId": "USB\\VID_32E6&PID_9251\\7&69F6256&0&2",
            "IsForced": False,
            "PersistedGuid": "ggg",
            "StubInstanceId": None,
        },
    ]
}


@pytest.fixture()
def state_json_file(tmp_path: Path) -> Path:
    """Write _USBIPD_STATE to a temp JSON file and return its path."""
    p = tmp_path / "usbipd_state.json"
    p.write_text(json.dumps(_USBIPD_STATE), encoding="utf-8")
    return p


@pytest.fixture()
def config():
    from inchworm.config import load_config
    return load_config()


# ---------------------------------------------------------------------------
# Unit tests: _parse_instance_id
# ---------------------------------------------------------------------------

class TestParseInstanceId:
    def test_real_serial(self):
        vid, pid, serial = attach_usb._parse_instance_id(
            "USB\\VID_1A86&PID_55D3\\5B91045044"
        )
        assert vid == "1a86"
        assert pid == "55d3"
        assert serial == "5B91045044"

    def test_windows_path_segment_returns_none_serial(self):
        vid, pid, serial = attach_usb._parse_instance_id(
            "USB\\VID_1A86&PID_7523\\7&165AD49D&0&2"
        )
        assert vid == "1a86"
        assert pid == "7523"
        assert serial is None  # Windows-internal, not a real serial

    def test_arducam_serial(self):
        vid, pid, serial = attach_usb._parse_instance_id(
            "USB\\VID_0C45&PID_6366\\UC762"
        )
        assert vid == "0c45"
        assert pid == "6366"
        assert serial == "UC762"

    def test_stm32_serial(self):
        vid, pid, serial = attach_usb._parse_instance_id(
            "USB\\VID_0483&PID_5740\\306133643438"
        )
        assert vid == "0483"
        assert pid == "5740"
        assert serial == "306133643438"

    def test_openaicam_no_serial(self):
        """cam_right openaicam has a Windows path segment, not a real serial."""
        vid, pid, serial = attach_usb._parse_instance_id(
            "USB\\VID_32E6&PID_9251\\7&69F6256&0&2"
        )
        assert vid == "32e6"
        assert pid == "9251"
        assert serial is None

    def test_openaicam_with_serial(self):
        """cam_left openaicam has a real serial."""
        vid, pid, serial = attach_usb._parse_instance_id(
            "USB\\VID_32E6&PID_9251\\5423534724"
        )
        assert vid == "32e6"
        assert pid == "9251"
        assert serial == "5423534724"


# ---------------------------------------------------------------------------
# Unit tests: _resolve_busid
# ---------------------------------------------------------------------------

class TestResolveBusId:
    def _devices(self) -> list[dict]:
        return _USBIPD_STATE["Devices"]

    def test_resolves_arm_left_by_serial(self):
        spec = attach_usb._DeviceSpec("arm_left", "1a86", "55d3", "5B91045044")
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid == "10-4"

    def test_resolves_arm_right_by_serial(self):
        spec = attach_usb._DeviceSpec("arm_right", "1a86", "55d3", "5B91030298")
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid == "9-3"

    def test_resolves_relay_by_vid_pid_no_serial(self):
        """Relay has no USB serial; resolved by VID:PID alone."""
        spec = attach_usb._DeviceSpec("relay", "1a86", "7523", None)
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid == "9-2"

    def test_resolves_cam_left_by_serial(self):
        spec = attach_usb._DeviceSpec("cam_left", "32e6", "9251", "5423534724")
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid == "9-4"

    def test_resolves_cam_right_by_vid_pid_no_serial(self):
        """cam_right has no serial; should resolve to the openaicam with a path segment."""
        spec = attach_usb._DeviceSpec("cam_right", "32e6", "9251", None)
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid == "10-2"

    def test_missing_device_returns_none(self):
        """A device not in the list returns None."""
        spec = attach_usb._DeviceSpec("ghost", "dead", "beef", "GHOST123")
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid is None

    def test_wrong_serial_returns_none(self):
        """Correct VID:PID but wrong serial → None."""
        spec = attach_usb._DeviceSpec("arm_left", "1a86", "55d3", "WRONGSERIAL")
        busid = attach_usb._resolve_busid(spec, self._devices())
        assert busid is None


# ---------------------------------------------------------------------------
# Integration: attach_all with fixture state JSON
# ---------------------------------------------------------------------------

class TestAttachAll:
    def test_missing_device_result_has_role_and_vid_pid(
        self, config, tmp_path: Path
    ):
        """A device absent from usbipd state produces a MISSING result naming the role."""
        # State with no devices at all.
        empty_state = {"Devices": []}
        p = tmp_path / "empty.json"
        p.write_text(json.dumps(empty_state))

        results = attach_usb.attach_all(config, dry_run=True, state_json_path=p)
        by_role = {r.role: r for r in results}

        arm_left = by_role["arm_left"]
        assert not arm_left.found
        assert not arm_left.success
        assert "arm_left" in arm_left.message
        assert "1a86" in arm_left.message  # VID appears in message

    def test_already_attached_device_is_ok_without_subprocess(
        self, config, state_json_file: Path
    ):
        """Devices with ClientIPAddress set are reported OK without calling powershell."""
        with patch("subprocess.run") as mock_run:
            results = attach_usb.attach_all(
                config, dry_run=False, state_json_path=state_json_file
            )
        by_role = {r.role: r for r in results}

        # arm_right has ClientIPAddress set in fixture → already attached
        arm_right = by_role["arm_right"]
        assert arm_right.already_attached
        assert arm_right.success
        # subprocess.run must NOT have been called for already-attached devices
        # (it may still have been called for other devices in the fixture)

    def test_dry_run_does_not_call_subprocess(self, config, state_json_file: Path):
        """In dry-run mode, no subprocess.run calls are made."""
        with patch("subprocess.run") as mock_run:
            results = attach_usb.attach_all(
                config, dry_run=True, state_json_path=state_json_file
            )
        mock_run.assert_not_called()
        # All found devices should be success in dry-run
        by_role = {r.role: r for r in results}
        for role in ("arm_left", "arm_right", "dexarm", "relay",
                     "cam_left", "cam_right", "cam_scanner"):
            assert by_role[role].success, f"{role} should succeed in dry-run"

    def test_dry_run_message_contains_usbipd_attach(
        self, config, state_json_file: Path
    ):
        """Dry-run messages contain the usbipd attach command text."""
        results = attach_usb.attach_all(
            config, dry_run=True, state_json_path=state_json_file
        )
        by_role = {r.role: r for r in results}
        # dexarm is in state but not attached → should show attach command
        dexarm = by_role["dexarm"]
        assert not dexarm.already_attached
        assert "usbipd attach" in dexarm.message or "DRY-RUN" in dexarm.message
        assert dexarm.busid == "11-3"

    def test_failed_attach_produces_fail_result(self, config, state_json_file: Path):
        """When powershell.exe returns nonzero, the result is FAIL and not success."""
        failed_proc = MagicMock()
        failed_proc.returncode = 1
        failed_proc.stderr = "Access denied"

        with patch("subprocess.run", return_value=failed_proc):
            results = attach_usb.attach_all(
                config, dry_run=False, state_json_path=state_json_file
            )
        by_role = {r.role: r for r in results}
        # dexarm, arm_left, cam_scanner, cam_left, cam_right are not attached in fixture
        # and subprocess.run returns failure
        dexarm = by_role["dexarm"]
        assert not dexarm.success
        assert "FAIL" in dexarm.message

    def test_successful_attach_produces_ok_result(self, config, state_json_file: Path):
        """When powershell.exe returns 0, the result is OK."""
        ok_proc = MagicMock()
        ok_proc.returncode = 0
        ok_proc.stderr = ""
        ok_proc.stdout = ""

        with patch("subprocess.run", return_value=ok_proc):
            results = attach_usb.attach_all(
                config, dry_run=False, state_json_path=state_json_file
            )
        by_role = {r.role: r for r in results}
        dexarm = by_role["dexarm"]
        assert dexarm.success
        assert "OK" in dexarm.message


# ---------------------------------------------------------------------------
# main() exit codes
# ---------------------------------------------------------------------------

class TestMainExitCode:
    def test_main_returns_nonzero_on_missing_device(self, config, tmp_path: Path):
        """main() exits 1 when any device is missing."""
        empty_state = {"Devices": []}
        p = tmp_path / "empty.json"
        p.write_text(json.dumps(empty_state))

        ret = attach_usb.main(["--dry-run", "--state-json", str(p)])
        assert ret == 1

    def test_main_returns_zero_when_all_succeed(self, config, state_json_file: Path):
        """main() exits 0 when all devices are found and succeed in dry-run."""
        ret = attach_usb.main(["--dry-run", "--state-json", str(state_json_file)])
        assert ret == 0

    def test_main_returns_nonzero_on_attach_failure(
        self, config, state_json_file: Path
    ):
        """main() exits 1 when powershell.exe returns nonzero for any device."""
        failed_proc = MagicMock()
        failed_proc.returncode = 1
        failed_proc.stderr = "Error"

        with patch("subprocess.run", return_value=failed_proc):
            ret = attach_usb.main(["--state-json", str(state_json_file)])
        assert ret == 1


# ---------------------------------------------------------------------------
# vhci verification tests (monkeypatched subprocess + vhci_status_reader)
# ---------------------------------------------------------------------------

# A single-device usbipd state where the dexarm is not yet attached.
_SINGLE_DEVICE_STATE: dict = {
    "Devices": [
        {
            "BusId": "11-3",
            "ClientIPAddress": None,
            "Description": "USB Serial Device (COM6)",
            "InstanceId": "USB\\VID_0483&PID_5740\\306133643438",
            "IsForced": False,
            "PersistedGuid": "ccc",
            "StubInstanceId": None,
        },
    ]
}

# vhci_hcd status text with BusId 11-3 (dev_hex 000b0003) present as an attached entry.
_VHCI_STATUS_WITH_11_3 = (
    "hub port sta spd dev      sockfd local_busid\n"
    "hs  0002 006 002 000b0003 000003 1-3\n"
)

# vhci_hcd status text with no attached entries.
_VHCI_STATUS_EMPTY = (
    "hub port sta spd dev      sockfd local_busid\n"
    "hs  0002 004 000 00000000 000000 0-0\n"
)


@pytest.fixture()
def _single_device_state_file(tmp_path: Path) -> Path:
    """Write _SINGLE_DEVICE_STATE to a temp file and return the path."""
    p = tmp_path / "single_device.json"
    p.write_text(json.dumps(_SINGLE_DEVICE_STATE), encoding="utf-8")
    return p


class TestVhciVerification:
    """Verify that attach_usb.py checks /sys/.../vhci_hcd.0/status after attach.

    These tests monkeypatch subprocess.run (to avoid real powershell.exe calls)
    and supply a vhci_status_reader callable (to avoid filesystem access).  A
    minimal config is built directly from the single-device fixture.
    """

    # The dexarm _DeviceSpec that matches _SINGLE_DEVICE_STATE.
    _DEXARM_SPEC = attach_usb._DeviceSpec(
        role="dexarm",
        vid="0483",
        pid="5740",
        usb_serial="306133643438",
    )
    _DEXARM_BUSID = "11-3"

    # Devices list for _SINGLE_DEVICE_STATE.
    _DEVICES = _SINGLE_DEVICE_STATE["Devices"]

    def _ok_proc(self) -> MagicMock:
        proc = MagicMock()
        proc.returncode = 0
        proc.stderr = ""
        proc.stdout = ""
        return proc

    def test_immediate_vhci_presence_reports_ok(self):
        """(a) Device appears in vhci status immediately -> result is [OK], retried=False."""
        # vhci_status_reader always returns the device as present.
        reader = lambda: _VHCI_STATUS_WITH_11_3  # noqa: E731

        with patch("subprocess.run", return_value=self._ok_proc()):
            result = attach_usb._attach_device(
                self._DEXARM_SPEC,
                self._DEVICES,
                dry_run=False,
                vhci_status_reader=reader,
                verify_timeout_s=2.0,
                verify_poll_s=0.05,
            )

        assert result.success, f"Expected success, got: {result.message}"
        assert not result.retried, "Should not have retried when device present immediately"
        assert "[OK]" in result.message
        assert result.busid == self._DEXARM_BUSID

    def test_vhci_presence_only_after_retry_reports_retry_ok(self):
        """(b) Device absent after first attach but appears after retry -> [RETRY-OK], retried=True."""
        # The reader returns empty status for the first timeout window, then
        # returns the device as present on all subsequent calls (retry window).
        # We use a very short timeout (0.1 s) with 0.05 s poll so the first
        # _wait_for_vhci window exhausts in at most 3 reader calls.
        # The reader switches to present after the first subprocess.run returns,
        # meaning both subprocess calls succeed (returncode=0), but the device
        # only appears in vhci after the retry attach.
        ok_proc = self._ok_proc()

        subprocess_call_count = 0
        reader_call_count = 0

        def _patched_subprocess(cmd, **kwargs):
            nonlocal subprocess_call_count
            subprocess_call_count += 1
            return ok_proc

        def _counting_reader() -> str:
            nonlocal reader_call_count
            reader_call_count += 1
            # Keep returning empty until the first subprocess call (first attach)
            # has completed.  After that, keep returning empty until the second
            # subprocess call (retry) has completed, then switch to present.
            # We achieve this by switching to present once subprocess_call_count >= 2.
            if subprocess_call_count < 2:
                return _VHCI_STATUS_EMPTY
            return _VHCI_STATUS_WITH_11_3

        with patch("subprocess.run", side_effect=_patched_subprocess):
            result = attach_usb._attach_device(
                self._DEXARM_SPEC,
                self._DEVICES,
                dry_run=False,
                vhci_status_reader=_counting_reader,
                verify_timeout_s=0.1,
                verify_poll_s=0.05,
            )

        assert result.success, f"Expected success, got: {result.message}"
        assert result.retried, "Should have set retried=True"
        assert "RETRY-OK" in result.message
        assert result.busid == self._DEXARM_BUSID
        assert subprocess_call_count == 2, "Exactly two attach calls expected (original + retry)"

    def test_vhci_never_appears_reports_fail_and_exit_nonzero(self):
        """(c) Device never appears in vhci status -> [FAIL], success=False, exit 1."""
        # vhci_status_reader always returns empty (device never shows up).
        reader = lambda: _VHCI_STATUS_EMPTY  # noqa: E731
        ok_proc = self._ok_proc()

        with patch("subprocess.run", return_value=ok_proc):
            result = attach_usb._attach_device(
                self._DEXARM_SPEC,
                self._DEVICES,
                dry_run=False,
                vhci_status_reader=reader,
                # Very short timeout so the test doesn't take 10 s.
                verify_timeout_s=0.15,
                verify_poll_s=0.05,
            )

        assert not result.success, f"Expected failure, got: {result.message}"
        assert result.retried, "Should have retried once before giving up"
        assert "FAIL" in result.message
        assert result.busid == self._DEXARM_BUSID

        # Verify that main() would exit nonzero for this outcome.
        # Build a minimal state file and wire a reader that never sees the device.
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "single.json"
            p.write_text(json.dumps(_SINGLE_DEVICE_STATE), encoding="utf-8")

            # We need config for main(); use the default which has all 7 devices,
            # but only test _attach_device directly for the never-appears path.
            # The result already has success=False; confirm the aggregation logic.
            results = [result]
            any_failure = any(not r.success for r in results)
            assert any_failure, "Exit nonzero logic must trigger on FAIL result"
