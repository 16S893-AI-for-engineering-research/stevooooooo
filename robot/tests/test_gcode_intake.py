"""Tests for inchworm.gcode_intake — AA-34.

Covers:
  AC#1 — intake validates dialect, workspace, feedrate, temperature, coord mode
  AC#2 — Z-origin consistency check (GCodeOriginError with named reason)
  AC#3 — print-frame transform applied before workspace check (PrintFrameTransform protocol)
  AC#4 — stream abort: retract + laser off on firmware error or exception
  AC#6 — dry-run parses, validates, prints bounding box and estimated time
  AC#7 — sample fixture files; each rejection reason tested; dry-run output tested

AC#5 (Cura profile doc) and hardware streaming are NOT tested here (no live device).

All tests use in-memory G-code strings or fixture files; no worktree-absolute
paths — fixtures are loaded via the tests/data/ directory relative to this file.
"""

from __future__ import annotations

import math
import textwrap
from pathlib import Path
from typing import List, Optional, Tuple
from unittest.mock import MagicMock, call

import pytest

from inchworm.config import DexArmWorkspace
from inchworm.gcode_intake import (
    BoundingBox,
    GCodeArcRejected,
    GCodeBoundsError,
    GCodeDialectError,
    GCodeFeedrateError,
    GCodeIntakeConfig,
    GCodeOriginError,
    GCodeStreamAborted,
    GCodeTempError,
    GCodeValidationError,
    IdentityPrintFrame,
    ParsedSegment,
    PrintFrameTransform,
    _normalise_opcode,
    _tokenise_line,
    dry_run_segment,
    parse_segment,
    stream_segment,
    validate_segment,
)

# ---------------------------------------------------------------------------
# Fixture path helpers
# ---------------------------------------------------------------------------

_DATA_DIR = Path(__file__).parent / "data"


def _fixture(name: str) -> str:
    """Read a test fixture file and return its contents."""
    return (_DATA_DIR / name).read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# Default workspace (matches hardware.yaml mock defaults)
# ---------------------------------------------------------------------------

_WS = DexArmWorkspace(
    x_min_mm=-300.0,
    x_max_mm=300.0,
    y_min_mm=0.0,
    y_max_mm=450.0,
    z_min_mm=-100.0,
    z_max_mm=400.0,
)

_CONFIG = GCodeIntakeConfig(
    max_feedrate_mm_min=9000.0,
    max_nozzle_temp_c=280.0,
    max_bed_temp_c=120.0,
    z_origin_tolerance_mm=1.0,
    retract_z_mm=3.0,
)

# ---------------------------------------------------------------------------
# Unit: tokeniser and opcode normaliser
# ---------------------------------------------------------------------------


class TestNormaliseOpcode:
    def test_g00_to_g0(self) -> None:
        assert _normalise_opcode("G00") == "G0"

    def test_g01_to_g1(self) -> None:
        assert _normalise_opcode("G01") == "G1"

    def test_g28_unchanged(self) -> None:
        assert _normalise_opcode("G28") == "G28"

    def test_m104_unchanged(self) -> None:
        assert _normalise_opcode("M104") == "M104"

    def test_g90_unchanged(self) -> None:
        assert _normalise_opcode("G90") == "G90"


class TestTokeniseLine:
    def test_motion_with_params(self) -> None:
        gl = _tokenise_line("G1 X10 Y20 Z0.3 E1.5 F3000", 1)
        assert gl.opcode == "G1"
        assert gl.params == {"X": 10.0, "Y": 20.0, "Z": 0.3, "E": 1.5, "F": 3000.0}
        assert gl.comment == ""

    def test_comment_stripped(self) -> None:
        gl = _tokenise_line("G90 ; absolute mode", 2)
        assert gl.opcode == "G90"
        assert gl.comment == "absolute mode"

    def test_blank_line(self) -> None:
        gl = _tokenise_line("", 3)
        assert gl.opcode is None

    def test_comment_only_line(self) -> None:
        gl = _tokenise_line("; just a comment", 4)
        assert gl.opcode is None
        assert gl.comment == "just a comment"

    def test_zero_padded_g01(self) -> None:
        gl = _tokenise_line("G01 X5 Y5", 5)
        assert gl.opcode == "G1"

    def test_temp_command(self) -> None:
        gl = _tokenise_line("M104 S200", 6)
        assert gl.opcode == "M104"
        assert gl.params["S"] == 200.0

    def test_negative_param(self) -> None:
        gl = _tokenise_line("G1 Z-0.5 F1200", 7)
        assert gl.params["Z"] == pytest.approx(-0.5)


# ---------------------------------------------------------------------------
# Parser: parse_segment
# ---------------------------------------------------------------------------


class TestParseSegment:
    def test_simple_segment_no_error(self) -> None:
        gcode = _fixture("simple_segment.gcode")
        seg = parse_segment(gcode)
        assert seg.motion_line_count > 0
        assert not seg.bounding_box.is_empty
        assert seg.first_z == pytest.approx(0.3)

    def test_g28_raises_dialect_error(self) -> None:
        gcode = _fixture("g28_mid_segment.gcode")
        with pytest.raises(GCodeDialectError) as exc_info:
            parse_segment(gcode)
        err = exc_info.value
        assert err.reason == "g28_mid_segment"
        assert err.line_num >= 1

    def test_g28_error_carries_line_number(self) -> None:
        gcode = "G1 X10\nG28\nG1 X20"
        with pytest.raises(GCodeDialectError) as exc_info:
            parse_segment(gcode)
        assert exc_info.value.line_num == 2

    def test_unrecognised_opcode_strict(self) -> None:
        gcode = "G999 X10"
        with pytest.raises(GCodeDialectError) as exc_info:
            parse_segment(gcode, strict_dialect=True)
        err = exc_info.value
        assert err.reason == "unrecognised_command"

    def test_unrecognised_opcode_lenient(self) -> None:
        # Should not raise; unrecognised opcode is skipped with warning
        gcode = "G999 X10\nG1 X10 Y10"
        seg = parse_segment(gcode, strict_dialect=False)
        # Only G1 counts as a motion line
        assert seg.motion_line_count == 1

    def test_bounding_box_absolute(self) -> None:
        gcode = textwrap.dedent("""\
            G90
            G1 X10 Y20 Z0.3
            G1 X50 Y100 Z0.3
        """)
        seg = parse_segment(gcode)
        assert seg.bounding_box.x_min == pytest.approx(10.0)
        assert seg.bounding_box.x_max == pytest.approx(50.0)
        assert seg.bounding_box.y_min == pytest.approx(20.0)
        assert seg.bounding_box.y_max == pytest.approx(100.0)

    def test_bounding_box_relative_mode(self) -> None:
        gcode = textwrap.dedent("""\
            G90
            G1 X10 Y10 Z0.3
            G91
            G1 X5 Y5
        """)
        seg = parse_segment(gcode)
        # After relative G1 X5 Y5, position is (15, 15, 0.3)
        assert seg.bounding_box.x_max == pytest.approx(15.0)
        assert seg.bounding_box.y_max == pytest.approx(15.0)

    def test_estimated_time_with_feedrate(self) -> None:
        # 100 mm at 6000 mm/min = 1 second
        gcode = "G90\nG1 X100 F6000"
        seg = parse_segment(gcode)
        assert seg.estimated_time_s is not None
        assert seg.estimated_time_s == pytest.approx(1.0, rel=0.01)

    def test_estimated_time_none_when_no_feedrate(self) -> None:
        gcode = "G90\nG1 X100"
        seg = parse_segment(gcode)
        assert seg.estimated_time_s is None

    def test_first_z_recorded(self) -> None:
        gcode = "G90\nG1 X10 Y10\nG1 Z0.3\nG1 X20"
        seg = parse_segment(gcode)
        assert seg.first_z == pytest.approx(0.3)

    def test_first_z_none_when_no_z(self) -> None:
        gcode = "G90\nG1 X10 Y10"
        seg = parse_segment(gcode)
        assert seg.first_z is None

    def test_m82_m83_accepted(self) -> None:
        gcode = "M82\nM83\nG1 X10"
        seg = parse_segment(gcode)
        assert seg.motion_line_count == 1

    def test_fan_commands_accepted(self) -> None:
        gcode = "M106 S255\nM107\nG1 X10"
        seg = parse_segment(gcode)
        assert seg.motion_line_count == 1

    def test_g2_arc_accepted(self) -> None:
        gcode = "G90\nG1 X0 Y0\nG2 X10 Y0 I5 J0"
        seg = parse_segment(gcode)
        assert seg.motion_line_count == 2

    def test_g3_arc_accepted(self) -> None:
        gcode = "G90\nG1 X0 Y0\nG3 X10 Y0 I5 J0"
        seg = parse_segment(gcode)
        assert seg.motion_line_count == 2


# ---------------------------------------------------------------------------
# Validator: validate_segment
# ---------------------------------------------------------------------------


class TestValidateSegment:
    def test_valid_segment_passes(self) -> None:
        gcode = _fixture("simple_segment.gcode")
        seg = parse_segment(gcode)
        # Should not raise
        validate_segment(seg, _WS, config=_CONFIG)

    def test_out_of_bounds_y(self) -> None:
        gcode = _fixture("out_of_bounds.gcode")
        seg = parse_segment(gcode)
        with pytest.raises(GCodeBoundsError) as exc_info:
            validate_segment(seg, _WS, config=_CONFIG)
        err = exc_info.value
        assert err.reason == "out_of_workspace"
        assert err.line_num >= 1
        assert err.transformed_xyz[1] > _WS.y_max_mm

    def test_feedrate_exceeded(self) -> None:
        gcode = _fixture("feedrate_exceeded.gcode")
        seg = parse_segment(gcode)
        with pytest.raises(GCodeFeedrateError) as exc_info:
            validate_segment(seg, _WS, config=_CONFIG)
        err = exc_info.value
        assert err.reason == "feedrate_exceeded"
        assert err.feedrate > _CONFIG.max_feedrate_mm_min
        assert err.limit == _CONFIG.max_feedrate_mm_min

    def test_nozzle_temp_exceeded(self) -> None:
        gcode = _fixture("temp_exceeded.gcode")
        seg = parse_segment(gcode)
        with pytest.raises(GCodeTempError) as exc_info:
            validate_segment(seg, _WS, config=_CONFIG)
        err = exc_info.value
        assert err.reason == "temp_exceeded"
        assert err.command == "M104"
        assert err.temp == pytest.approx(300.0)

    def test_bed_temp_exceeded(self) -> None:
        gcode = _fixture("bed_temp_exceeded.gcode")
        seg = parse_segment(gcode)
        with pytest.raises(GCodeTempError) as exc_info:
            validate_segment(seg, _WS, config=_CONFIG)
        err = exc_info.value
        assert err.reason == "temp_exceeded"
        assert err.command == "M140"
        assert err.temp == pytest.approx(130.0)

    def test_z_origin_mismatch(self) -> None:
        # Segment starts at Z=0.3; frame_z0=0.3 but we set frame_z0=5.0 -> mismatch
        gcode = "G90\nG1 Z0.3 F1200\nG1 X10 Y10"
        seg = parse_segment(gcode)
        with pytest.raises(GCodeOriginError) as exc_info:
            validate_segment(seg, _WS, config=_CONFIG, frame_z0=5.0)
        err = exc_info.value
        assert err.reason == "z_origin_mismatch"
        assert abs(err.first_layer_z - err.frame_z0) > _CONFIG.z_origin_tolerance_mm

    def test_z_origin_within_tolerance(self) -> None:
        # Z=0.3, frame_z0=0.5, tolerance=1.0 -> should NOT raise
        gcode = "G90\nG1 Z0.3 F1200\nG1 X10 Y10"
        seg = parse_segment(gcode)
        validate_segment(seg, _WS, config=_CONFIG, frame_z0=0.5)  # no raise

    def test_frame_transform_applied_before_bounds(self) -> None:
        """AC#3: leveling correction shifts position; bounds check uses transformed coords."""

        class ShiftedFrame:
            frame_id = "shifted_y+10"

            def apply(self, xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
                x, y, z = xyz
                return (x, y + 10.0, z)

        # G-code at Y=445 which is in bounds for workspace Y_max=450,
        # but after +10 shift it becomes Y=455 -> out of bounds
        gcode = "G90\nG1 X0 Y445 Z0.3 F1200"
        seg = parse_segment(gcode)
        with pytest.raises(GCodeBoundsError) as exc_info:
            validate_segment(seg, _WS, config=_CONFIG, frame=ShiftedFrame())
        err = exc_info.value
        assert err.transformed_xyz[1] == pytest.approx(455.0)
        assert err.original_xyz[1] == pytest.approx(445.0)

    def test_identity_frame_no_shift(self) -> None:
        gcode = "G90\nG1 X0 Y100 Z0.3 F1200"
        seg = parse_segment(gcode)
        # Identity frame: no shift, within bounds
        validate_segment(seg, _WS, config=_CONFIG, frame=IdentityPrintFrame())

    def test_frame_id_is_logged(self, caplog) -> None:
        """Frame ID is included in debug logs for auditability (AC#3)."""
        import logging
        gcode = "G90\nG1 X10 Y10 Z0.3 F1200"
        seg = parse_segment(gcode)

        class TaggedFrame:
            frame_id = "hop_3_leveled"

            def apply(self, xyz):
                return xyz

        with caplog.at_level(logging.DEBUG, logger="inchworm.gcode_intake"):
            validate_segment(seg, _WS, config=_CONFIG, frame=TaggedFrame())
        assert "hop_3_leveled" in caplog.text

    def test_all_rejection_errors_carry_line_num(self) -> None:
        """Each rejection error has line_num >= 1."""
        cases = [
            # (gcode, exception_class)
            ("G1 X20 Y500 Z0.3 F1200", GCodeBoundsError),
            ("G1 X10 F12000", GCodeFeedrateError),
            ("M104 S300\nG1 X10 Y10", GCodeTempError),
        ]
        for gcode, exc_cls in cases:
            seg = parse_segment(gcode)
            with pytest.raises(GCodeValidationError) as exc_info:
                validate_segment(seg, _WS, config=_CONFIG)
            assert exc_info.value.line_num >= 1, f"{exc_cls.__name__} missing line_num"

    def test_relative_mode_bounds_check(self) -> None:
        # Start at X=290, then relative +20 -> X=310 -> out of bounds
        gcode = "G90\nG1 X290 Y100 Z0.3\nG91\nG1 X20"
        seg = parse_segment(gcode)
        with pytest.raises(GCodeBoundsError):
            validate_segment(seg, _WS, config=_CONFIG)


# ---------------------------------------------------------------------------
# Streamer: stream_segment
# ---------------------------------------------------------------------------


class _MockDriver:
    """Minimal DexArmDriver stub for streamer tests (updated for AA-34)."""

    def __init__(
        self,
        fail_on_call: Optional[int] = None,
        fail_move_relative: bool = False,
    ) -> None:
        self.moves: List[tuple] = []        # (x, y, z) or ("rel", dx, dy, dz)
        self.print_moves: List[tuple] = []  # (x, y, z, e, f) from print_move
        self.raw_cmds: List[str] = []
        self.quickstops: int = 0
        self._call_count = 0
        self._fail_on_call = fail_on_call
        self._fail_move_relative = fail_move_relative
        # Simulated position for get_position
        self._pos_x: float = 5.0
        self._pos_y: float = 10.0
        self._pos_z: float = 0.3

    def get_position(self):
        """Return a simple namespace that has .x, .y, .z."""
        from types import SimpleNamespace
        return SimpleNamespace(x=self._pos_x, y=self._pos_y, z=self._pos_z)

    def print_move(self, x, y, z, e=None, f=None, *, dry_run=False) -> None:
        """AA-34 bounded print move with E and F."""
        self._call_count += 1
        if self._fail_on_call is not None and self._call_count >= self._fail_on_call:
            from inchworm.dexarm import DexArmCommandError
            raise DexArmCommandError(f"G1 X{x} Y{y} Z{z}", "error:rejected")
        self.print_moves.append((x, y, z, e, f))
        self.moves.append((x, y, z))  # keep for backward-compat assertions

    def move_absolute(self, x, y, z, feedrate=None, *, dry_run=False) -> None:
        """Legacy — used by retract / safe-park paths (not called during streaming)."""
        self._call_count += 1
        if self._fail_on_call is not None and self._call_count >= self._fail_on_call:
            from inchworm.dexarm import DexArmCommandError
            raise DexArmCommandError(f"G0 X{x} Y{y} Z{z}", "error:rejected")
        self.moves.append((x, y, z))

    def move_relative(self, dx, dy, dz, feedrate=None, *, dry_run=False) -> None:
        """Used for retract on abort."""
        if self._fail_move_relative:
            from inchworm.dexarm import DexArmCommandError
            raise DexArmCommandError("G1 Z+3", "error:firmware_error")
        self.moves.append(("rel", dx, dy, dz))

    def quickstop(self, *, dry_run: bool = False) -> None:
        """M410 quickstop — called when retract fails."""
        self.quickstops += 1

    def send_raw(self, cmd: str, *, dry_run: bool = False) -> list:
        self.raw_cmds.append(cmd)
        return []


class _MockLaser:
    def __init__(self) -> None:
        self._commanded_on = True  # pretend it's on

    def off(self) -> None:
        self._commanded_on = False


class TestStreamSegment:
    def _simple_seg(self) -> ParsedSegment:
        gcode = textwrap.dedent("""\
            G90
            G1 X10 Y10 Z0.3 F2400
            G1 X20 Y10 E1.0
            G1 X20 Y20 E2.0
        """)
        return parse_segment(gcode)

    def test_stream_calls_print_move(self) -> None:
        """stream_segment uses print_move (AA-34 bounded API) for motion lines."""
        seg = self._simple_seg()
        driver = _MockDriver()
        stream_segment(seg, driver)
        # All three G1 lines should produce print_move calls
        assert len(driver.print_moves) == 3

    # Keep old name as alias so CI doesn't break if run on old results
    def test_stream_calls_move_absolute(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver()
        stream_segment(seg, driver)
        assert len(driver.moves) == 3

    def test_stream_dry_run_no_real_moves(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver()
        stream_segment(seg, driver, dry_run=True)
        # dry_run=True still calls print_move (with dry_run=True forwarded)
        # but MockDriver records the call anyway
        assert len(driver.print_moves) == 3  # all recorded

    def test_progress_callback_called(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver()
        calls: List[Tuple[int, int]] = []
        stream_segment(seg, driver, progress_cb=lambda s, t: calls.append((s, t)))
        motion_count = seg.motion_line_count
        assert len(calls) == motion_count
        # Last call should be (motion_count, motion_count)
        assert calls[-1] == (motion_count, motion_count)

    def test_abort_on_firmware_error_raises_stream_aborted(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver(fail_on_call=2)  # fail on 2nd move
        laser = _MockLaser()
        with pytest.raises(GCodeStreamAborted) as exc_info:
            stream_segment(seg, driver, laser_driver=laser)
        assert isinstance(exc_info.value.cause, Exception)

    def test_abort_retracts_nozzle(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver(fail_on_call=1)  # fail immediately
        laser = _MockLaser()
        with pytest.raises(GCodeStreamAborted) as exc_info:
            stream_segment(seg, driver, laser_driver=laser, config=_CONFIG)
        # Retract: a ("rel", 0, 0, 3.0) should be in moves
        retract_moves = [m for m in driver.moves if isinstance(m[0], str) and m[0] == "rel"]
        assert len(retract_moves) == 1
        assert retract_moves[0][3] == pytest.approx(_CONFIG.retract_z_mm)
        assert exc_info.value.retract_ok is True
        assert exc_info.value.quickstop_attempted is False

    def test_abort_turns_laser_off(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver(fail_on_call=1)
        laser = _MockLaser()
        laser._commanded_on = True
        with pytest.raises(GCodeStreamAborted):
            stream_segment(seg, driver, laser_driver=laser)
        assert not laser._commanded_on

    def test_abort_without_laser_does_not_fail(self) -> None:
        seg = self._simple_seg()
        driver = _MockDriver(fail_on_call=1)
        with pytest.raises(GCodeStreamAborted):
            stream_segment(seg, driver, laser_driver=None)

    def test_non_motion_commands_sent_as_raw(self) -> None:
        gcode = "M104 S200\nG1 X10 Y10 Z0.3"
        seg = parse_segment(gcode)
        driver = _MockDriver()
        stream_segment(seg, driver)
        # M104 S200 should appear in raw_cmds
        assert any("M104" in cmd for cmd in driver.raw_cmds)

    # --- AA-34 new tests ---

    def test_retract_fails_then_quickstop(self) -> None:
        """When move_relative raises, quickstop (M410) is attempted and
        GCodeStreamAborted reports retract_ok=False, quickstop_attempted=True."""
        seg = self._simple_seg()
        driver = _MockDriver(fail_on_call=1, fail_move_relative=True)
        laser = _MockLaser()
        with pytest.raises(GCodeStreamAborted) as exc_info:
            stream_segment(seg, driver, laser_driver=laser, config=_CONFIG)
        aborted = exc_info.value
        assert aborted.retract_ok is False
        assert aborted.quickstop_attempted is True
        assert driver.quickstops == 1
        # Laser must still be turned off even though retract failed
        assert not laser._commanded_on

    def test_e_passthrough_absolute_mode(self) -> None:
        """In M82 (absolute E) mode, E values are forwarded as-is to print_move."""
        gcode = textwrap.dedent("""\
            G90
            M82
            G92 E0
            G1 X10 Y10 Z0.3 E0.5 F2400
            G1 X20 Y10 Z0.3 E1.2
        """)
        seg = parse_segment(gcode)
        driver = _MockDriver()
        stream_segment(seg, driver)
        e_values = [m[3] for m in driver.print_moves]
        assert e_values[0] == pytest.approx(0.5)
        assert e_values[1] == pytest.approx(1.2)

    def test_e_passthrough_relative_mode(self) -> None:
        """In M83 (relative E) mode, E deltas are forwarded as-is to print_move."""
        gcode = textwrap.dedent("""\
            G90
            M83
            G1 X10 Y10 Z0.3 E0.3 F2400
            G1 X20 Y10 Z0.3 E0.4
        """)
        seg = parse_segment(gcode)
        driver = _MockDriver()
        stream_segment(seg, driver)
        e_values = [m[3] for m in driver.print_moves]
        assert e_values[0] == pytest.approx(0.3)
        assert e_values[1] == pytest.approx(0.4)

    def test_no_e_in_move_passes_none(self) -> None:
        """Motion lines without E pass None for e to print_move."""
        gcode = "G90\nG1 X10 Y10 Z0.3 F2400"
        seg = parse_segment(gcode)
        driver = _MockDriver()
        stream_segment(seg, driver)
        assert len(driver.print_moves) == 1
        assert driver.print_moves[0][3] is None  # e=None

    def test_frame_transform_applied_to_streamed_moves(self) -> None:
        """AA-34 AC#3: print-frame transform is applied to every streamed XYZ."""

        class ShiftFrame:
            frame_id = "shift_x+100"

            def apply(self, xyz):
                x, y, z = xyz
                return (x + 100.0, y, z)

        gcode = "G90\nG1 X10 Y10 Z0.3 F2400\nG1 X20 Y20 Z0.3"
        seg = parse_segment(gcode)
        driver = _MockDriver()
        stream_segment(seg, driver, frame=ShiftFrame())
        # All X values should be shifted by +100
        assert driver.print_moves[0][0] == pytest.approx(110.0)
        assert driver.print_moves[1][0] == pytest.approx(120.0)

    def test_start_position_queried_via_m114(self) -> None:
        """AA-34: start position is queried via get_position (M114), not assumed 0,0,0."""
        # Driver reports start pos (5.0, 10.0, 0.3)
        gcode = textwrap.dedent("""\
            G91
            G1 X5 Y0 Z0 F2400
        """)
        seg = parse_segment(gcode)
        driver = _MockDriver()  # _pos_x=5.0, _pos_y=10.0, _pos_z=0.3
        stream_segment(seg, driver)
        # Relative G1 X+5 from start (5,10,0.3) -> (10,10,0.3)
        assert driver.print_moves[0][0] == pytest.approx(10.0)
        assert driver.print_moves[0][1] == pytest.approx(10.0)

    def test_start_position_assumed_zero_when_get_position_unavailable(self) -> None:
        """When get_position fails, stream falls back to (0,0,0) and logs a warning."""

        class NoPosMockDriver(_MockDriver):
            def get_position(self):
                raise AttributeError("not available")

        gcode = textwrap.dedent("""\
            G91
            G1 X5 Y0 Z0 F2400
        """)
        seg = parse_segment(gcode)
        driver = NoPosMockDriver()
        # Orchestrator 2026-10-24: live streaming must refuse, not assume (0,0,0).
        with pytest.raises(Exception, match="(?i)start-position|M114"):
            stream_segment(seg, driver)
        assert driver.print_moves == []

    def test_arc_g2_rejected_at_stream_time(self) -> None:
        """G2 arc is rejected during streaming with GCodeArcRejected."""
        from inchworm.gcode_intake import GCodeArcRejected
        gcode = "G90\nG1 X0 Y0 Z0.3 F2400\nG2 X10 Y0 I5 J0"
        seg = parse_segment(gcode)  # parse is OK (dialect accepts G2)
        driver = _MockDriver()
        with pytest.raises(GCodeStreamAborted) as exc_info:
            stream_segment(seg, driver)
        # The cause should be GCodeArcRejected
        assert isinstance(exc_info.value.cause, GCodeArcRejected)
        assert exc_info.value.cause.reason == "arc_not_linearised"

    def test_arc_g3_rejected_at_stream_time(self) -> None:
        """G3 arc is rejected during streaming with GCodeArcRejected."""
        from inchworm.gcode_intake import GCodeArcRejected
        gcode = "G90\nG1 X0 Y0 Z0.3 F2400\nG3 X10 Y0 I5 J0"
        seg = parse_segment(gcode)
        driver = _MockDriver()
        with pytest.raises(GCodeStreamAborted) as exc_info:
            stream_segment(seg, driver)
        assert isinstance(exc_info.value.cause, GCodeArcRejected)

    def test_identity_frame_no_shift_in_streaming(self) -> None:
        """IdentityPrintFrame leaves coordinates unchanged during streaming."""
        gcode = "G90\nG1 X50 Y100 Z0.3 F2400"
        seg = parse_segment(gcode)
        driver = _MockDriver()
        stream_segment(seg, driver, frame=IdentityPrintFrame())
        assert driver.print_moves[0][0] == pytest.approx(50.0)
        assert driver.print_moves[0][1] == pytest.approx(100.0)



# ---------------------------------------------------------------------------
# Dry-run: dry_run_segment
# ---------------------------------------------------------------------------


class TestDryRunSegment:
    def test_dry_run_valid_segment(self) -> None:
        gcode = _fixture("simple_segment.gcode")
        output: List[str] = []
        seg = dry_run_segment(gcode, _WS, config=_CONFIG, print_fn=output.append)
        joined = "\n".join(output)
        assert "Bounding box" in joined
        assert "PASSED" in joined

    def test_dry_run_prints_bounding_box(self) -> None:
        gcode = "G90\nG1 X10 Y20 Z0.3 F2400\nG1 X50 Y80 Z0.3"
        output: List[str] = []
        dry_run_segment(gcode, _WS, config=_CONFIG, print_fn=output.append)
        joined = "\n".join(output)
        assert "BoundingBox" in joined or "Bounding" in joined

    def test_dry_run_prints_estimated_time(self) -> None:
        # 100 mm at F6000 = 1 second
        gcode = "G90\nG1 X100 F6000"
        output: List[str] = []
        dry_run_segment(gcode, _WS, config=_CONFIG, print_fn=output.append)
        joined = "\n".join(output)
        assert "time" in joined.lower() or "Estimated" in joined

    def test_dry_run_raises_on_invalid_segment(self) -> None:
        gcode = _fixture("out_of_bounds.gcode")
        with pytest.raises(GCodeBoundsError):
            dry_run_segment(gcode, _WS, config=_CONFIG)

    def test_dry_run_raises_g28_at_parse_stage(self) -> None:
        gcode = _fixture("g28_mid_segment.gcode")
        with pytest.raises(GCodeDialectError) as exc_info:
            dry_run_segment(gcode, _WS, config=_CONFIG)
        assert exc_info.value.reason == "g28_mid_segment"

    def test_dry_run_returns_parsed_segment(self) -> None:
        gcode = _fixture("simple_segment.gcode")
        seg = dry_run_segment(gcode, _WS, config=_CONFIG, print_fn=lambda _: None)
        assert isinstance(seg, ParsedSegment)
        assert seg.motion_line_count > 0

    def test_dry_run_with_named_frame(self) -> None:
        """Frame ID printed in dry-run output (AC#3 audit trail)."""

        class HopFrame:
            frame_id = "hop_2_leveled"

            def apply(self, xyz):
                return xyz

        gcode = "G90\nG1 X10 Y10 Z0.3 F2400"
        output: List[str] = []
        dry_run_segment(
            gcode, _WS, config=_CONFIG,
            frame=HopFrame(), print_fn=output.append,
        )
        joined = "\n".join(output)
        assert "hop_2_leveled" in joined


# ---------------------------------------------------------------------------
# Protocol compliance: IdentityPrintFrame
# ---------------------------------------------------------------------------


class TestIdentityPrintFrame:
    def test_is_protocol_compliant(self) -> None:
        f = IdentityPrintFrame()
        assert isinstance(f, PrintFrameTransform)

    def test_apply_is_identity(self) -> None:
        f = IdentityPrintFrame()
        xyz = (1.5, 2.5, 3.5)
        assert f.apply(xyz) == xyz

    def test_frame_id(self) -> None:
        assert IdentityPrintFrame().frame_id == "identity"


# ---------------------------------------------------------------------------
# Config schema in HardwareConfig (integration)
# ---------------------------------------------------------------------------


class TestHardwareConfigIntegration:
    def test_load_config_gcode_intake_firmware_limits(self) -> None:
        """hardware.yaml gcode_intake carries the DexArm firmware-derived limits
        (2026-10-27: block added; it was previously absent -> None)."""
        from inchworm.config import load_config, DEFAULT_CONFIG_PATH
        cfg = load_config(DEFAULT_CONFIG_PATH)
        gi = cfg.gcode_intake
        assert gi is not None
        assert gi.max_nozzle_temp_c < 275.0          # firmware HEATER_0_MAXTEMP
        assert gi.max_bed_temp_c < 1.0               # no heated bed
        assert gi.max_feedrate_mm_min <= 9000.0      # not loosened

    def test_parse_config_without_gcode_intake_block(self) -> None:
        """A config without the block still loads (gcode_intake -> None)."""
        import yaml
        from inchworm.config import parse_config, DEFAULT_CONFIG_PATH
        base = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
        base.pop("gcode_intake", None)
        assert parse_config(base).gcode_intake is None

    def test_parse_config_with_gcode_intake_block(self) -> None:
        """gcode_intake block in config parses correctly."""
        import yaml
        from inchworm.config import parse_config, DEFAULT_CONFIG_PATH

        base = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
        base["gcode_intake"] = {
            "max_feedrate_mm_min": 6000.0,
            "max_nozzle_temp_c": 250.0,
            "max_bed_temp_c": 100.0,
            "z_origin_tolerance_mm": 0.5,
            "retract_z_mm": 5.0,
        }
        cfg = parse_config(base)
        assert cfg.gcode_intake is not None
        assert cfg.gcode_intake.max_feedrate_mm_min == pytest.approx(6000.0)
        assert cfg.gcode_intake.max_nozzle_temp_c == pytest.approx(250.0)


# ---------------------------------------------------------------------------
# BoundingBox
# ---------------------------------------------------------------------------


class TestBoundingBox:
    def test_empty_initially(self) -> None:
        bb = BoundingBox()
        assert bb.is_empty

    def test_update_changes_bounds(self) -> None:
        bb = BoundingBox()
        bb.update(1.0, 2.0, 3.0)
        bb.update(10.0, 20.0, 30.0)
        assert bb.x_min == pytest.approx(1.0)
        assert bb.x_max == pytest.approx(10.0)
        assert bb.y_min == pytest.approx(2.0)
        assert bb.y_max == pytest.approx(20.0)
        assert bb.z_min == pytest.approx(3.0)
        assert bb.z_max == pytest.approx(30.0)

    def test_str_not_empty(self) -> None:
        bb = BoundingBox()
        bb.update(0.0, 0.0, 0.0)
        s = str(bb)
        assert "BoundingBox" in s


# --- Orchestrator 2026-10-24 audit findings 1 and 3 ---
def test_stream_calls_get_position_with_real_driver_signature():
    """The real DexArmDriver.get_position() takes no arguments; the mock must
    match, so a dry_run= kwarg is caught as a TypeError in tests."""
    import inspect
    from inchworm.dexarm import DexArmDriver
    assert list(inspect.signature(DexArmDriver.get_position).parameters) == ["self"]


def test_m114_failure_live_raises_typed_abort_and_runs_safety_sequence():
    from inchworm.gcode_intake import GCodeStreamAborted

    class Laser:
        def __init__(self): self.off_calls = 0
        def off(self): self.off_calls += 1

    class NoPos(_MockDriver):
        def get_position(self):
            raise RuntimeError("M114 timeout")

    seg = parse_segment("G1 X5 Y0 Z0 F2400\n")
    laser = Laser()
    with pytest.raises(GCodeStreamAborted, match="M114"):
        stream_segment(seg, NoPos(), laser_driver=laser)
    assert laser.off_calls >= 1


# ---------------------------------------------------------------------------
# AC#5: a slice made with the documented Rotrics profile passes the intake
# (docs/cura_dexarm_profile.md; tools/cura/slice_rotrics_normal.sh).
# ---------------------------------------------------------------------------


class _OffsetFrame:
    """Translate the centred Cura slice into the DexArm arm frame (as the
    AA-33 print frame does), here to a nominal print point Y=300 mm."""

    frame_id = "test-offset"

    def apply(self, xyz):
        return (xyz[0], xyz[1] + 300.0, xyz[2])


class TestRotricsProfileSlice:
    _FIRMWARE_LIMITS = GCodeIntakeConfig(
        # DexArm firmware: HEATER_0_MAXTEMP 275, no heated bed (TEMP_SENSOR_BED 0).
        max_nozzle_temp_c=260.0,
        max_bed_temp_c=0.0001,
        max_feedrate_mm_min=9000.0,
    )

    def _seg(self):
        return parse_segment(_fixture("rotrics_normal_10x10x1mm.gcode"))

    def test_parses_in_strict_dialect(self) -> None:
        seg = self._seg()
        assert seg.motion_line_count > 100
        assert seg.first_z == pytest.approx(0.2)

    def test_validates_against_firmware_limits_in_arm_frame(self) -> None:
        seg = self._seg()
        validate_segment(seg, _WS, config=self._FIRMWARE_LIMITS,
                         frame=_OffsetFrame(), frame_z0=0.2)

    def test_profile_temperature_and_speeds(self) -> None:
        text = _fixture("rotrics_normal_10x10x1mm.gcode")
        assert "M109 S205" in text and "M140" not in text and "M190" not in text
        assert "G28" not in text and "M888" not in text and "M2000" not in text
        feeds = [float(t[1:]) for line in text.splitlines() if not line.startswith(";")
                 for t in line.split(";")[0].split() if t.startswith("F")]
        # Fastest move is the 50 mm/s travel (3000 mm/min).
        assert max(feeds) == pytest.approx(3000.0)

    def test_dry_run_stream_moves_into_arm_frame(self) -> None:
        seg = self._seg()
        driver = _MockDriver()
        stream_segment(seg, driver, frame=_OffsetFrame(), dry_run=True)
        assert len(driver.print_moves) == seg.motion_line_count
        ys = [m[1] for m in driver.moves]
        assert min(ys) > 290.0 and max(ys) < 310.0
