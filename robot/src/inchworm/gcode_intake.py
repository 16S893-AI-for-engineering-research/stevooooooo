"""Cura segment G-code intake — parser, validator, and streamer (AA-34).

This module owns the boundary between an externally-sliced segment G-code
file and the robot.  A segment must be validated against the DexArm workspace
and the current leveled print frame before any streaming begins.

Architecture overview
---------------------
* :class:`GCodeIntakeConfig`      — all tunable parameters (feedrate, temp,
                                    Z-origin tolerance) with documented defaults.
* :func:`parse_segment`           — line-by-line parser; returns a
                                    :class:`ParsedSegment` with bounding box,
                                    estimated print time, and per-line commands.
* :func:`validate_segment`        — validates a :class:`ParsedSegment` against
                                    workspace bounds (after the AA-33 print-frame
                                    transform), feedrate limits, temperature limits,
                                    and Z-origin consistency.
* :func:`stream_segment`          — streams a validated segment line-by-line
                                    through the :class:`~inchworm.dexarm.DexArmDriver`
                                    bounded API with progress callbacks; on firmware
                                    error or exception retracts the nozzle and
                                    ensures the laser is off.
* :func:`dry_run_segment`         — parse + validate + print bounding box and
                                    estimated time; no motion commanded.

PrintFrame protocol
-------------------
:class:`PrintFrameTransform` is a minimal structural Protocol (PEP 544) that
AA-33 ``print_frame.py`` must implement::

    class PrintFrameTransform(Protocol):
        frame_id: str
        def apply(self, xyz: tuple[float, float, float]) -> tuple[float, float, float]: ...

If ``inchworm.print_frame`` is not yet present another lane is writing it;
import it dynamically in callers.  :class:`IdentityPrintFrame` is provided
here for tests and dry-run use.

Allowed command set (Marlin/Cura dialect)
-----------------------------------------
* **Motion**: G0, G1 (linear), G2, G3 (arc — accepted at parse/validate time;
  **rejected at stream time** with :class:`GCodeArcRejected`) — with optional
  X, Y, Z, E, F, I, J, R params
* **Positioning mode**: G90 (absolute XYZ), G91 (relative XYZ)
* **Extruder mode**: M82 (absolute E), M83 (relative E)
* **Temperature**: M104, M109 (nozzle), M140, M190 (bed)
* **Fan**: M106 (on with S param), M107 (off)
* **Misc**: M84 (disable steppers), M73 (set percent), M0, M1 (pause/stop),
            M117 (LCD message), M900 (linear advance), M907 (motor current),
            G4 (dwell), G92 (set position/reset E), G21 (mm units),
            T0 (tool select), M110 (set line number), M201/M203/M204/M205
            (acceleration/jerk), M106 (fan speed)
* **Comments**: anything after ``;`` is ignored

Forbidden mid-segment
---------------------
* G28 (home) is **never** allowed in a segment file.  A Cura *end G-code*
  or *between-segment purge* that includes G28 will be rejected with
  :class:`GCodeDialectError`.

Named rejection reasons
-----------------------
All rejection errors carry a ``line_num`` (1-indexed) and a ``reason`` string:

* :class:`GCodeDialectError`   — forbidden command (G28) or unrecognised opcode
* :class:`GCodeBoundsError`    — position outside workspace after frame transform
* :class:`GCodeFeedrateError`  — feedrate exceeds configured limit
* :class:`GCodeTempError`      — temperature command exceeds configured limit
* :class:`GCodeOriginError`    — first-layer Z inconsistent with leveled frame Z0
* :class:`GCodeArcRejected`    — G2/G3 arc command encountered at stream time

Safety on abort
---------------
:func:`stream_segment` catches every exception from the DexArm driver and from
the progress callback.  On any error:

1. Retract nozzle: ``driver.move_relative(0, 0, +retract_z_mm)``.
2. If retract raises, fall back to ``driver.quickstop()`` (M410) and log it.
3. Laser off: call ``laser_driver.off()`` if a laser driver was supplied.
   This step is **always attempted** regardless of whether the retract or
   quickstop succeeded.
4. Re-raise the original exception wrapped in :class:`GCodeStreamAborted`,
   which carries ``retract_ok`` and ``quickstop_attempted`` attributes.

Units
-----
All positions are in **millimetres** (matching the DexArm driver convention).
Feedrate is in **mm/min**.  Temperatures are in **°C**.
"""

from __future__ import annotations

import logging
import math
import re
import time
from dataclasses import dataclass, field
from typing import (
    Callable,
    Iterator,
    List,
    Optional,
    Protocol,
    Sequence,
    Tuple,
    Union,
    runtime_checkable,
)

from inchworm.config import DexArmWorkspace

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass
class GCodeIntakeConfig:
    """Tunable parameters for the G-code intake.

    All values have safe defaults.  Override from hardware.yaml via the
    ``gcode_intake`` block parsed into :class:`~inchworm.config.GCodeIntakeConfigSchema`
    and passed here.

    Attributes
    ----------
    max_feedrate_mm_min:
        Maximum allowed feedrate in mm/min.  Moves with F values above this
        are rejected with :class:`GCodeFeedrateError`.
        Default 9000 mm/min (150 mm/s); tighten for the DexArm's real limits.
        unverified: true — owner must confirm hardware limit.
    max_nozzle_temp_c:
        Maximum allowed M104/M109 nozzle temperature in °C.
        Default 280 °C (conservative for most FDM materials).
        unverified: true — owner must confirm nozzle/hotend rating.
    max_bed_temp_c:
        Maximum allowed M140/M190 bed temperature in °C.
        Default 120 °C.
        unverified: true — owner must confirm bed heater rating.
    z_origin_tolerance_mm:
        Tolerance (mm) for Z-origin consistency check: the first motion layer's
        Z value must be within this distance of the leveled frame Z0.
        Default 1.0 mm; tighten after AA-33 leveling is calibrated.
        unverified: true.
    retract_z_mm:
        Distance (mm) to retract the nozzle on abort.  Default 3.0 mm.
    arc_segments:
        Number of linear segments to approximate G2/G3 arcs when computing
        bounding boxes.  Default 32.
    """

    max_feedrate_mm_min: float = 9000.0        # unverified: true
    max_nozzle_temp_c: float = 280.0           # unverified: true
    max_bed_temp_c: float = 120.0              # unverified: true
    z_origin_tolerance_mm: float = 1.0        # unverified: true
    retract_z_mm: float = 3.0
    arc_segments: int = 32


# ---------------------------------------------------------------------------
# PrintFrame protocol
# ---------------------------------------------------------------------------


@runtime_checkable
class PrintFrameTransform(Protocol):
    """Minimal protocol for an AA-33 print-frame transform.

    AA-33 ``print_frame.py`` must expose an object satisfying this protocol.
    The ``apply`` method transforms a position from the *G-code coordinate
    system* (segment-local) into the *DexArm arm frame* after applying the
    per-hop leveling correction.

    Attributes
    ----------
    frame_id:
        A string identifying the frame instance (e.g. ``"hop_3_leveled"``).
        Logged with the segment so the applied transform is auditable.
    """

    frame_id: str

    def apply(
        self, xyz: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Transform *xyz* from G-code space to DexArm arm frame.

        Parameters
        ----------
        xyz:
            (x, y, z) in mm, G-code coordinate system.

        Returns
        -------
        tuple[float, float, float]
            (x, y, z) in mm, DexArm arm frame.
        """
        ...


class IdentityPrintFrame:
    """No-op print-frame transform (identity, no leveling correction applied).

    Use this for dry-run and tests when AA-33 ``print_frame.py`` is not
    yet available.  In live mode prefer a real leveled frame from AA-33.
    """

    frame_id: str = "identity"

    def apply(
        self, xyz: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        return xyz


# ---------------------------------------------------------------------------
# Typed validation errors
# ---------------------------------------------------------------------------


class GCodeValidationError(ValueError):
    """Base class for all G-code validation errors.

    Attributes
    ----------
    line_num:
        1-indexed line number in the input file.
    line_text:
        The raw source line that triggered the error.
    reason:
        Short machine-readable rejection reason string.
    """

    def __init__(self, line_num: int, line_text: str, reason: str, message: str) -> None:
        super().__init__(f"line {line_num}: {reason}: {message}")
        self.line_num = line_num
        self.line_text = line_text
        self.reason = reason


class GCodeDialectError(GCodeValidationError):
    """Forbidden G-code command or unrecognised opcode.

    Reasons:
    - ``g28_mid_segment``: G28 (home) found anywhere in the segment file.
    - ``unrecognised_command``: opcode not in the allowed set.
    """


class GCodeBoundsError(GCodeValidationError):
    """Position is outside the DexArm workspace after applying the frame transform.

    Reason: ``out_of_workspace``.

    Attributes
    ----------
    original_xyz:
        The (x, y, z) position in G-code space before the transform.
    transformed_xyz:
        The (x, y, z) position after the print-frame transform, in arm frame.
    workspace:
        The configured :class:`~inchworm.config.DexArmWorkspace`.
    """

    def __init__(
        self,
        line_num: int,
        line_text: str,
        original_xyz: Tuple[float, float, float],
        transformed_xyz: Tuple[float, float, float],
        workspace: DexArmWorkspace,
    ) -> None:
        ox, oy, oz = original_xyz
        tx, ty, tz = transformed_xyz
        super().__init__(
            line_num,
            line_text,
            "out_of_workspace",
            f"G-code XYZ ({ox:.3f},{oy:.3f},{oz:.3f}) -> arm frame "
            f"({tx:.3f},{ty:.3f},{tz:.3f}) is outside workspace "
            f"X[{workspace.x_min_mm},{workspace.x_max_mm}] "
            f"Y[{workspace.y_min_mm},{workspace.y_max_mm}] "
            f"Z[{workspace.z_min_mm},{workspace.z_max_mm}]",
        )
        self.original_xyz = original_xyz
        self.transformed_xyz = transformed_xyz
        self.workspace = workspace


class GCodeFeedrateError(GCodeValidationError):
    """Feedrate exceeds the configured maximum.

    Reason: ``feedrate_exceeded``.
    """

    def __init__(
        self,
        line_num: int,
        line_text: str,
        feedrate: float,
        limit: float,
    ) -> None:
        super().__init__(
            line_num,
            line_text,
            "feedrate_exceeded",
            f"feedrate {feedrate:.1f} mm/min exceeds limit {limit:.1f} mm/min",
        )
        self.feedrate = feedrate
        self.limit = limit


class GCodeTempError(GCodeValidationError):
    """Temperature command exceeds the configured maximum.

    Reason: ``temp_exceeded``.
    """

    def __init__(
        self,
        line_num: int,
        line_text: str,
        command: str,
        temp: float,
        limit: float,
    ) -> None:
        super().__init__(
            line_num,
            line_text,
            "temp_exceeded",
            f"{command} temperature {temp:.1f}°C exceeds limit {limit:.1f}°C",
        )
        self.command = command
        self.temp = temp
        self.limit = limit


class GCodeOriginError(GCodeValidationError):
    """First-layer Z is inconsistent with the leveled frame Z0.

    Reason: ``z_origin_mismatch``.
    """

    def __init__(
        self,
        line_num: int,
        line_text: str,
        first_layer_z: float,
        frame_z0: float,
        tolerance: float,
    ) -> None:
        super().__init__(
            line_num,
            line_text,
            "z_origin_mismatch",
            f"first-layer Z {first_layer_z:.4f} mm differs from leveled "
            f"frame Z0 {frame_z0:.4f} mm by more than tolerance "
            f"{tolerance:.4f} mm",
        )
        self.first_layer_z = first_layer_z
        self.frame_z0 = frame_z0
        self.tolerance = tolerance


class GCodeStreamAborted(RuntimeError):
    """Raised by :func:`stream_segment` when streaming is aborted on error.

    The nozzle retract and laser-off actions have already been attempted
    before this exception is raised.  The ``cause`` attribute holds the
    original exception.

    Attributes
    ----------
    cause:
        The original exception that triggered the abort.
    retract_ok:
        ``True`` if the nozzle retract (move_relative +Z) succeeded.
        ``False`` if the retract itself raised (quickstop was attempted).
    quickstop_attempted:
        ``True`` if M410 quickstop was sent (because retract failed).
    """

    def __init__(
        self,
        message: str,
        cause: BaseException,
        retract_ok: bool = True,
        quickstop_attempted: bool = False,
    ) -> None:
        super().__init__(message)
        self.cause = cause
        self.retract_ok = retract_ok
        self.quickstop_attempted = quickstop_attempted


class GCodeArcRejected(GCodeValidationError):
    """Raised during streaming when a G2/G3 arc command is encountered.

    Arc commands (G2/G3) are accepted during *parsing* and *validation*
    (for bounding-box computation) but **rejected at stream time** because
    the DexArm firmware may not support arc interpolation reliably, and Cura
    can be configured to output only linear moves.

    Reason: ``arc_not_linearised``.

    **Cura configuration**: disable arc support in Cura
    (search for "Arc Welder" or "Arc" in Cura settings and ensure it is off).
    See ``docs/cura_dexarm_profile.md`` section §7 for details.
    """

    def __init__(self, line_num: int, line_text: str) -> None:
        super().__init__(
            line_num,
            line_text,
            "arc_not_linearised",
            "G2/G3 arc commands are rejected at stream time; configure Cura "
            "to disable arc output (Arc Welder off). "
            "See docs/cura_dexarm_profile.md §7.",
        )


# ---------------------------------------------------------------------------
# Internal G-code line representation
# ---------------------------------------------------------------------------


@dataclass
class GCodeLine:
    """One parsed G-code line.

    Attributes
    ----------
    line_num:
        1-indexed line number.
    raw:
        Raw text of the line.
    opcode:
        Upper-cased command code (e.g. ``"G1"``, ``"M104"``), or ``None``
        for blank/comment-only lines.
    params:
        Parsed letter→value map (e.g. ``{'X': 10.0, 'F': 3000.0}``).
        Letter keys are upper-case; values are floats.
    comment:
        Everything after the ``;``, stripped; ``""`` if none.
    """

    line_num: int
    raw: str
    opcode: Optional[str]
    params: dict  # str -> float
    comment: str


# ---------------------------------------------------------------------------
# Tokeniser
# ---------------------------------------------------------------------------

# Matches a single G/M/T word and optional leading line number (N...).
_OPCODE_RE = re.compile(r"(?:N\d+\s+)?([GMTF][0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)
# Matches parameter letters followed by an optional signed float.
_PARAM_RE = re.compile(r"([A-Z])(-?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

# Allowed opcodes (upper-case).  G2/G3 accepted but arc bounds are approximated.
_ALLOWED_OPCODES: frozenset[str] = frozenset(
    [
        # Motion
        "G0", "G1", "G2", "G3",
        # Positioning and extruder mode
        "G4", "G21", "G90", "G91", "G92",
        # Temperature
        "M104", "M109", "M140", "M190",
        # Fan
        "M106", "M107",
        # Misc
        "M73", "M82", "M83", "M84", "M85",
        "M0", "M1", "M117",
        "M201", "M203", "M204", "M205",
        "M900", "M907",
        "M110", "M115",
        # Tool select (T0)
        "T0", "T1",
        # G28 is explicitly handled as a hard reject; not in this set.
    ]
)

_NOZZLE_TEMP_OPCODES: frozenset[str] = frozenset(["M104", "M109"])
_BED_TEMP_OPCODES: frozenset[str] = frozenset(["M140", "M190"])
_MOTION_OPCODES: frozenset[str] = frozenset(["G0", "G1", "G2", "G3"])


def _tokenise_line(raw: str, line_num: int) -> GCodeLine:
    """Parse one raw G-code line into a :class:`GCodeLine`."""
    # Split at comment marker
    if ";" in raw:
        code_part, comment = raw.split(";", 1)
        comment = comment.strip()
    else:
        code_part = raw
        comment = ""

    code_part = code_part.strip()

    opcode: Optional[str] = None
    params: dict = {}

    if code_part:
        m = _OPCODE_RE.match(code_part)
        if m:
            opcode = m.group(1).upper()
            # Normalise zero-padded forms: G00 -> G0, G01 -> G1, M104 stays M104
            opcode = _normalise_opcode(opcode)
            # Parse params
            for pm in _PARAM_RE.finditer(code_part[m.end():]):
                key = pm.group(1).upper()
                try:
                    params[key] = float(pm.group(2))
                except ValueError:
                    pass

    return GCodeLine(
        line_num=line_num,
        raw=raw,
        opcode=opcode,
        params=params,
        comment=comment,
    )


def _normalise_opcode(opcode: str) -> str:
    """Normalise zero-padded G-codes: G00->G0, G01->G1, G02->G2, G03->G3."""
    if opcode.startswith("G") or opcode.startswith("M"):
        prefix = opcode[0]
        num_str = opcode[1:]
        try:
            num = int(num_str)
            return f"{prefix}{num}"
        except ValueError:
            return opcode
    return opcode


# ---------------------------------------------------------------------------
# ParsedSegment
# ---------------------------------------------------------------------------


@dataclass
class BoundingBox:
    """3-D bounding box in mm (G-code coordinate system, before transform)."""

    x_min: float = math.inf
    x_max: float = -math.inf
    y_min: float = math.inf
    y_max: float = -math.inf
    z_min: float = math.inf
    z_max: float = -math.inf

    def update(self, x: float, y: float, z: float) -> None:
        self.x_min = min(self.x_min, x)
        self.x_max = max(self.x_max, x)
        self.y_min = min(self.y_min, y)
        self.y_max = max(self.y_max, y)
        self.z_min = min(self.z_min, z)
        self.z_max = max(self.z_max, z)

    @property
    def is_empty(self) -> bool:
        return self.x_min == math.inf

    def __str__(self) -> str:
        if self.is_empty:
            return "BoundingBox(empty)"
        return (
            f"BoundingBox("
            f"X[{self.x_min:.3f},{self.x_max:.3f}] "
            f"Y[{self.y_min:.3f},{self.y_max:.3f}] "
            f"Z[{self.z_min:.3f},{self.z_max:.3f}]) mm"
        )


@dataclass
class ParsedSegment:
    """Result of parsing a G-code segment file.

    Attributes
    ----------
    lines:
        All parsed :class:`GCodeLine` objects (including blank/comment lines).
    bounding_box:
        Axis-aligned bounding box of all commanded positions, in G-code space.
    estimated_time_s:
        Estimated print time in seconds derived from move distances and
        feedrates.  ``None`` if no feedrate information was found.
    first_z:
        Z value of the first motion command with a Z parameter.  Used for
        Z-origin consistency checking.  ``None`` if no Z was found.
    motion_line_count:
        Number of motion (G0/G1/G2/G3) lines in the segment.
    """

    lines: List[GCodeLine]
    bounding_box: BoundingBox
    estimated_time_s: Optional[float]
    first_z: Optional[float]
    motion_line_count: int


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------


def parse_segment(
    source: Union[str, Sequence[str]],
    *,
    config: Optional[GCodeIntakeConfig] = None,
    strict_dialect: bool = True,
) -> ParsedSegment:
    """Parse a G-code segment and return a :class:`ParsedSegment`.

    This function does NOT validate workspace bounds or temperatures — that is
    done by :func:`validate_segment`.  It does reject obviously invalid dialect
    issues (G28 mid-segment, unrecognised opcodes) when *strict_dialect* is
    True (the default).

    Parameters
    ----------
    source:
        Either a multi-line string, a list of line strings, or any sequence
        of strings (one per G-code line).
    config:
        :class:`GCodeIntakeConfig` for arc approximation resolution.
        Uses module defaults when ``None``.
    strict_dialect:
        When ``True`` (default), raise :class:`GCodeDialectError` on G28 or
        unrecognised opcodes.  When ``False``, unrecognised opcodes are logged
        as warnings and G28 still raises.

    Returns
    -------
    ParsedSegment

    Raises
    ------
    GCodeDialectError
        If G28 is found anywhere in the file, or if *strict_dialect* is True
        and an unrecognised opcode is encountered.
    """
    if config is None:
        config = GCodeIntakeConfig()

    if isinstance(source, str):
        raw_lines: Sequence[str] = source.splitlines()
    else:
        raw_lines = list(source)

    parsed_lines: List[GCodeLine] = []
    bbox = BoundingBox()
    estimated_time_s: float = 0.0
    has_feedrate = False
    first_z: Optional[float] = None
    motion_count = 0

    # Coordinate mode state (default in Cura: absolute XYZ, absolute E)
    abs_xyz = True   # G90 = True, G91 = False
    cur_f: Optional[float] = None  # current feedrate mm/min

    # Current position for relative move tracking
    cur_x: float = 0.0
    cur_y: float = 0.0
    cur_z: float = 0.0

    for i, raw in enumerate(raw_lines):
        line_num = i + 1
        gl = _tokenise_line(raw, line_num)
        parsed_lines.append(gl)

        if gl.opcode is None:
            continue

        # ---- Hard reject: G28 anywhere ----
        if gl.opcode == "G28":
            raise GCodeDialectError(
                line_num,
                raw,
                "g28_mid_segment",
                "G28 (home) is not allowed in a segment file; "
                "it must not appear between segment start and end G-code",
            )

        # ---- Dialect check ----
        if gl.opcode not in _ALLOWED_OPCODES:
            if strict_dialect:
                raise GCodeDialectError(
                    line_num,
                    raw,
                    "unrecognised_command",
                    f"opcode {gl.opcode!r} is not in the allowed command set",
                )
            else:
                log.warning(
                    "gcode_intake: line %d: unrecognised opcode %r (skipping)",
                    line_num,
                    gl.opcode,
                )
                continue

        # ---- Mode tracking ----
        if gl.opcode == "G90":
            abs_xyz = True
        elif gl.opcode == "G91":
            abs_xyz = False

        # ---- Feedrate tracking ----
        if "F" in gl.params:
            cur_f = gl.params["F"]
            if cur_f > 0:
                has_feedrate = True

        # ---- Motion commands: bounding box + estimated time ----
        if gl.opcode in _MOTION_OPCODES:
            motion_count += 1

            # Resolve target position (absolute or relative)
            raw_x = gl.params.get("X")
            raw_y = gl.params.get("Y")
            raw_z = gl.params.get("Z")

            if abs_xyz:
                tx = raw_x if raw_x is not None else cur_x
                ty = raw_y if raw_y is not None else cur_y
                tz = raw_z if raw_z is not None else cur_z
            else:
                tx = cur_x + (raw_x or 0.0)
                ty = cur_y + (raw_y or 0.0)
                tz = cur_z + (raw_z or 0.0)

            # Record first Z for origin check
            if first_z is None and raw_z is not None:
                first_z = tz

            # Update bounding box
            if gl.opcode in ("G0", "G1"):
                bbox.update(tx, ty, tz)
            elif gl.opcode in ("G2", "G3"):
                # Approximate arc with line segments for bounding box
                _expand_bbox_arc(
                    bbox,
                    cur_x, cur_y,
                    tx, ty,
                    gl.params,
                    gl.opcode == "G3",
                    config.arc_segments,
                )
                bbox.update(tx, ty, tz)

            # Accumulate estimated time
            if cur_f is not None and cur_f > 0:
                dist = math.sqrt(
                    (tx - cur_x) ** 2 + (ty - cur_y) ** 2 + (tz - cur_z) ** 2
                )
                estimated_time_s += (dist / cur_f) * 60.0  # mm / (mm/min) * 60 = s

            cur_x, cur_y, cur_z = tx, ty, tz

    return ParsedSegment(
        lines=parsed_lines,
        bounding_box=bbox,
        estimated_time_s=estimated_time_s if has_feedrate else None,
        first_z=first_z,
        motion_line_count=motion_count,
    )


def _expand_bbox_arc(
    bbox: BoundingBox,
    sx: float, sy: float,
    ex: float, ey: float,
    params: dict,
    ccw: bool,
    n_segments: int,
) -> None:
    """Approximate a G2/G3 arc and update *bbox* with intermediate points.

    Uses the I/J centre-offset form; R form is not supported by this
    approximation (a log warning is emitted and start+end are used).
    """
    if "I" in params or "J" in params:
        ci = params.get("I", 0.0)
        cj = params.get("J", 0.0)
        cx = sx + ci
        cy = sy + cj
        # Start and end angles
        a0 = math.atan2(sy - cy, sx - cx)
        a1 = math.atan2(ey - cy, ex - cx)
        r = math.sqrt(ci ** 2 + cj ** 2)
        if r < 1e-6:
            return
        if ccw:
            if a1 <= a0:
                a1 += 2 * math.pi
        else:
            if a1 >= a0:
                a1 -= 2 * math.pi
        for k in range(n_segments + 1):
            t = k / n_segments
            a = a0 + t * (a1 - a0)
            bbox.update(cx + r * math.cos(a), cy + r * math.sin(a), 0.0)
    else:
        # R form: fall back to start and end only
        log.warning(
            "gcode_intake: G2/G3 with R parameter; "
            "arc bounding box may be underestimated"
        )


# ---------------------------------------------------------------------------
# Validator
# ---------------------------------------------------------------------------


def validate_segment(
    segment: ParsedSegment,
    workspace: DexArmWorkspace,
    *,
    config: Optional[GCodeIntakeConfig] = None,
    frame: Optional[PrintFrameTransform] = None,
    frame_z0: Optional[float] = None,
) -> None:
    """Validate a :class:`ParsedSegment` against workspace bounds, feedrate,
    temperature, and Z-origin consistency.

    Parameters
    ----------
    segment:
        A :class:`ParsedSegment` returned by :func:`parse_segment`.
    workspace:
        The :class:`~inchworm.config.DexArmWorkspace` to validate against.
    config:
        :class:`GCodeIntakeConfig` with limits.  Uses defaults when ``None``.
    frame:
        Print-frame transform to apply to each position before the workspace
        check.  When ``None``, :class:`IdentityPrintFrame` is used (no
        leveling correction — acceptable for dry-run or when AA-33 is not
        yet implemented).
    frame_z0:
        The leveled frame's Z0 value (mm, DexArm arm frame).  When supplied,
        the first-layer Z after transform is checked against this value.
        Skipped when ``None``.

    Raises
    ------
    GCodeBoundsError
        If any commanded position (after frame transform) falls outside
        the workspace.
    GCodeFeedrateError
        If any feedrate value exceeds ``config.max_feedrate_mm_min``.
    GCodeTempError
        If any temperature command exceeds the configured limit.
    GCodeOriginError
        If ``frame_z0`` is supplied and the first-layer Z after transform
        differs from *frame_z0* by more than ``config.z_origin_tolerance_mm``.
    """
    if config is None:
        config = GCodeIntakeConfig()
    if frame is None:
        frame = IdentityPrintFrame()

    log.debug(
        "gcode_intake: validating segment; frame_id=%s first_z=%s",
        frame.frame_id,
        segment.first_z,
    )

    abs_xyz = True
    cur_x = 0.0
    cur_y = 0.0
    cur_z = 0.0
    first_motion_z_checked = False

    for gl in segment.lines:
        if gl.opcode is None:
            continue

        # Mode tracking
        if gl.opcode == "G90":
            abs_xyz = True
        elif gl.opcode == "G91":
            abs_xyz = False

        # Feedrate check
        if "F" in gl.params:
            f = gl.params["F"]
            if f > config.max_feedrate_mm_min:
                raise GCodeFeedrateError(
                    gl.line_num, gl.raw, f, config.max_feedrate_mm_min
                )

        # Temperature checks
        if gl.opcode in _NOZZLE_TEMP_OPCODES and "S" in gl.params:
            t = gl.params["S"]
            if t > config.max_nozzle_temp_c:
                raise GCodeTempError(
                    gl.line_num, gl.raw, gl.opcode, t, config.max_nozzle_temp_c
                )
        if gl.opcode in _BED_TEMP_OPCODES and "S" in gl.params:
            t = gl.params["S"]
            if t > config.max_bed_temp_c:
                raise GCodeTempError(
                    gl.line_num, gl.raw, gl.opcode, t, config.max_bed_temp_c
                )

        # Motion: bounds check
        if gl.opcode in _MOTION_OPCODES:
            raw_x = gl.params.get("X")
            raw_y = gl.params.get("Y")
            raw_z = gl.params.get("Z")

            if abs_xyz:
                tx = raw_x if raw_x is not None else cur_x
                ty = raw_y if raw_y is not None else cur_y
                tz = raw_z if raw_z is not None else cur_z
            else:
                tx = cur_x + (raw_x or 0.0)
                ty = cur_y + (raw_y or 0.0)
                tz = cur_z + (raw_z or 0.0)

            # Apply print-frame transform
            tx_arm, ty_arm, tz_arm = frame.apply((tx, ty, tz))

            # Z-origin consistency check (first Z command only)
            if not first_motion_z_checked and raw_z is not None and frame_z0 is not None:
                first_motion_z_checked = True
                delta = abs(tz_arm - frame_z0)
                if delta > config.z_origin_tolerance_mm:
                    raise GCodeOriginError(
                        gl.line_num,
                        gl.raw,
                        tz_arm,
                        frame_z0,
                        config.z_origin_tolerance_mm,
                    )

            # Workspace bounds check (only if all params present or partially present)
            _check_workspace(
                gl.line_num, gl.raw,
                (tx, ty, tz),
                (tx_arm, ty_arm, tz_arm),
                workspace,
            )

            cur_x, cur_y, cur_z = tx, ty, tz

    log.debug(
        "gcode_intake: segment validated OK; frame_id=%s motion_lines=%d",
        frame.frame_id,
        segment.motion_line_count,
    )


def _check_workspace(
    line_num: int,
    raw: str,
    gcode_xyz: Tuple[float, float, float],
    arm_xyz: Tuple[float, float, float],
    workspace: DexArmWorkspace,
) -> None:
    """Raise :class:`GCodeBoundsError` if *arm_xyz* is outside *workspace*."""
    ax, ay, az = arm_xyz
    ws = workspace
    if (
        ax < ws.x_min_mm or ax > ws.x_max_mm
        or ay < ws.y_min_mm or ay > ws.y_max_mm
        or az < ws.z_min_mm or az > ws.z_max_mm
    ):
        raise GCodeBoundsError(line_num, raw, gcode_xyz, arm_xyz, workspace)


# ---------------------------------------------------------------------------
# Streamer
# ---------------------------------------------------------------------------

#: Retract distance in mm on abort.
_RETRACT_Z_MM = 3.0

# Progress callback: (lines_sent: int, total_lines: int) -> None
ProgressCallback = Callable[[int, int], None]


# --- AA-34 ---
def stream_segment(
    segment: ParsedSegment,
    driver: object,  # DexArmDriver — typed loosely to avoid circular import
    *,
    config: Optional[GCodeIntakeConfig] = None,
    frame: Optional[PrintFrameTransform] = None,
    laser_driver: Optional[object] = None,  # LaserDriver
    progress_cb: Optional[ProgressCallback] = None,
    dry_run: bool = False,
) -> None:
    """Stream a validated :class:`ParsedSegment` line-by-line to the DexArm.

    Streaming corrections (AA-34 orchestrator review 2026-10-24)
    -------------------------------------------------------------
    * **Start position**: M114 is sent before the first move; the reported
      position seeds the tracking state instead of assuming (0, 0, 0).
    * **Print-frame transform**: ``frame.apply(xyz)`` is called on every
      G0/G1 XYZ before the ``print_move`` call so the AA-33 leveling
      correction reaches the arm on every move.
    * **E passthrough**: M82/M83 extrusion mode is tracked; absolute E is
      forwarded verbatim; relative E is accumulated from the start-position
      query.  Both modes pass E through to ``DexArmDriver.print_move``.
    * **Arc rejection**: G2/G3 are rejected with :class:`GCodeArcRejected`.
      Configure Cura with arc support off (see ``docs/cura_dexarm_profile.md``).
    * **Abort path**: on any driver error the abort sequence is:
      1. ``driver.move_relative(0, 0, +retract_z)`` — retract nozzle.
      2. If retract fails, ``driver.quickstop()`` (M410) — halt motion.
      3. ``laser_driver.off()`` — always attempted regardless of retract outcome.
      :class:`GCodeStreamAborted` is raised with ``retract_ok`` and
      ``quickstop_attempted`` attributes recording which actions succeeded.

    Parameters
    ----------
    segment:
        A :class:`ParsedSegment` that has already been validated by
        :func:`validate_segment`.
    driver:
        An open :class:`~inchworm.dexarm.DexArmDriver` instance.
    config:
        :class:`GCodeIntakeConfig` for abort retract distance.  Uses module
        defaults when ``None``.
    frame:
        Print-frame transform (AA-33).  When ``None``, :class:`IdentityPrintFrame`
        is used (no leveling correction).  In live printing always supply a
        verified :class:`~inchworm.print_frame.PrintFrame` from AA-33.
    laser_driver:
        Optional :class:`~inchworm.laser.LaserDriver`.  Its ``off()`` method
        is called on abort.
    progress_cb:
        Optional callable invoked after each line is sent with
        ``(lines_sent, total_motion_lines)``.
    dry_run:
        When ``True``, all driver calls are forwarded with ``dry_run=True``
        (no bytes sent).

    Raises
    ------
    GCodeStreamAborted
        If any driver error or unexpected exception occurs.  Nozzle retract
        and laser-off have already been attempted.
    GCodeArcRejected
        If a G2 or G3 arc command is encountered during streaming.
    """
    if config is None:
        config = GCodeIntakeConfig()
    if frame is None:
        frame = IdentityPrintFrame()

    motion_lines = [gl for gl in segment.lines if gl.opcode in _MOTION_OPCODES]
    sendable_lines = [gl for gl in segment.lines if gl.opcode is not None]
    total = len(motion_lines)
    sent = 0

    log.info(
        "gcode_intake: starting stream; total_sendable=%d motion_lines=%d "
        "frame_id=%s dry_run=%s",
        len(sendable_lines),
        total,
        frame.frame_id,
        dry_run,
    )

    # --- Query start position via M114 (AA-34 fix: do not assume 0,0,0) ---
    # Orchestrator 2026-10-24: in LIVE mode an unknown start position must
    # fail closed. Relative moves from a guessed (0,0,0) could drive the
    # nozzle anywhere. The (0,0,0) seed is kept for dry-run only.
    try:
        # DexArmDriver.get_position() takes no arguments (audit finding 1).
        start_pos = driver.get_position() if not dry_run else None  # type: ignore[union-attr]
    except Exception as exc:
        if not dry_run:
            # Nothing has moved yet, but run the full abort sequence anyway
            # (retract/quickstop + laser off) and raise the typed error callers
            # catch (audit finding 3).
            retract_ok, quickstop = _abort_safe(driver, laser_driver, config, dry_run=False)
            raise GCodeStreamAborted(
                f"M114 start-position query failed; refusing to stream: {exc}",
                exc, retract_ok=retract_ok, quickstop_attempted=quickstop,
            ) from exc
        start_pos = None
    if start_pos is None and not dry_run:
        _err = RuntimeError("M114 returned no position")
        retract_ok, quickstop = _abort_safe(driver, laser_driver, config, dry_run=False)
        raise GCodeStreamAborted(
            "M114 returned no position; refusing to stream", _err,
            retract_ok=retract_ok, quickstop_attempted=quickstop,
        )

    # State tracking for streaming
    abs_xyz = True
    # Seed XYZ from M114 if available; fall back to 0,0,0 with a warning
    if start_pos is not None:
        cur_x: float = start_pos.x
        cur_y: float = start_pos.y
        cur_z: float = start_pos.z
    else:
        if not dry_run:
            log.warning(
                "gcode_intake: M114 start-position query failed; "
                "seeding position as (0, 0, 0)"
            )
        cur_x = 0.0
        cur_y = 0.0
        cur_z = 0.0

    # Extruder state: track M82 (absolute) / M83 (relative) mode
    abs_e = True         # M82 = absolute E (Cura default); M83 = relative
    cur_e: float = 0.0   # current absolute E position (seeded from 0 or G92)
    cur_f: Optional[float] = None

    gl: Optional[object] = None  # keep reference for error logging

    try:
        for gl in sendable_lines:
            # Mode tracking
            if gl.opcode == "G90":  # type: ignore[union-attr]
                abs_xyz = True
                driver.send_raw("G90", dry_run=dry_run)  # type: ignore[union-attr]
                continue
            elif gl.opcode == "G91":  # type: ignore[union-attr]
                abs_xyz = False
                driver.send_raw("G91", dry_run=dry_run)  # type: ignore[union-attr]
                continue

            # Extruder mode tracking (never send raw; tracked internally)
            if gl.opcode == "M82":  # type: ignore[union-attr]
                abs_e = True
                raw_cmd = _rebuild_gcode(gl)  # type: ignore[arg-type]
                driver.send_raw(raw_cmd, dry_run=dry_run)  # type: ignore[union-attr]
                continue
            elif gl.opcode == "M83":  # type: ignore[union-attr]
                abs_e = False
                raw_cmd = _rebuild_gcode(gl)  # type: ignore[arg-type]
                driver.send_raw(raw_cmd, dry_run=dry_run)  # type: ignore[union-attr]
                continue

            if "F" in gl.params:  # type: ignore[union-attr]
                cur_f = gl.params["F"]  # type: ignore[union-attr]

            if gl.opcode in _MOTION_OPCODES:  # type: ignore[union-attr]
                # --- Reject arcs at stream time ---
                if gl.opcode in ("G2", "G3"):  # type: ignore[union-attr]
                    raise GCodeArcRejected(gl.line_num, gl.raw)  # type: ignore[union-attr]

                raw_x = gl.params.get("X")  # type: ignore[union-attr]
                raw_y = gl.params.get("Y")  # type: ignore[union-attr]
                raw_z = gl.params.get("Z")  # type: ignore[union-attr]
                raw_e = gl.params.get("E")  # type: ignore[union-attr]

                # Resolve XYZ target
                if abs_xyz:
                    tx = raw_x if raw_x is not None else cur_x
                    ty = raw_y if raw_y is not None else cur_y
                    tz = raw_z if raw_z is not None else cur_z
                else:
                    tx = cur_x + (raw_x or 0.0)
                    ty = cur_y + (raw_y or 0.0)
                    tz = cur_z + (raw_z or 0.0)

                # --- Apply AA-33 print-frame transform ---
                tx_arm, ty_arm, tz_arm = frame.apply((tx, ty, tz))

                # Resolve E value (pass through; caller responsible for G92 reset)
                send_e: Optional[float] = None
                if raw_e is not None:
                    if abs_e:
                        # Absolute E: forward as-is; track for relative transitions
                        send_e = raw_e
                        cur_e = raw_e
                    else:
                        # Relative E: pass as delta (firmware is in M83 mode)
                        send_e = raw_e
                        cur_e += raw_e

                # Use print_move (bounds-checked G1 with E/F) — AA-34 bounded API
                driver.print_move(  # type: ignore[union-attr]
                    tx_arm, ty_arm, tz_arm,
                    e=send_e,
                    f=cur_f,
                    dry_run=dry_run,
                )
                cur_x, cur_y, cur_z = tx, ty, tz
                sent += 1
                if progress_cb is not None:
                    progress_cb(sent, total)
            else:
                # Non-motion command: send via send_raw
                raw_cmd = _rebuild_gcode(gl)  # type: ignore[arg-type]
                # G92 resets extruder position — track cur_e
                if gl.opcode == "G92" and "E" in gl.params:  # type: ignore[union-attr]
                    cur_e = gl.params["E"]  # type: ignore[union-attr]
                driver.send_raw(raw_cmd, dry_run=dry_run)  # type: ignore[union-attr]

    except Exception as exc:
        line_num = gl.line_num if gl is not None and hasattr(gl, "line_num") else 0  # type: ignore[union-attr]
        log.error(
            "gcode_intake: stream aborted at line %d: %s",
            line_num,
            exc,
        )
        retract_ok, quickstop_attempted = _abort_safe(
            driver, laser_driver, config, dry_run=dry_run
        )
        raise GCodeStreamAborted(
            f"streaming aborted: {exc}",
            cause=exc,
            retract_ok=retract_ok,
            quickstop_attempted=quickstop_attempted,
        ) from exc

    log.info(
        "gcode_intake: stream complete; motion_lines_sent=%d dry_run=%s",
        sent,
        dry_run,
    )
# --- AA-34 end ---


def _rebuild_gcode(gl: GCodeLine) -> str:
    """Reconstruct a sendable G-code string from a :class:`GCodeLine`."""
    parts = [gl.opcode]
    for k, v in gl.params.items():
        # Represent E as float, integer-valued floats without decimals
        if v == int(v):
            parts.append(f"{k}{int(v)}")
        else:
            parts.append(f"{k}{v:.4f}")
    return " ".join(parts)


def _abort_safe(
    driver: object,
    laser_driver: Optional[object],
    config: GCodeIntakeConfig,
    *,
    dry_run: bool = False,
) -> Tuple[bool, bool]:
    """Attempt nozzle retract and laser off as best-effort on abort.

    Abort sequence (AA-34):
    1. ``driver.move_relative(0, 0, +retract_z)`` — retract nozzle.
    2. If retract raises, ``driver.quickstop()`` (M410) — halt motion.
    3. ``laser_driver.off()`` — always attempted regardless of retract outcome.

    Returns
    -------
    tuple[bool, bool]
        ``(retract_ok, quickstop_attempted)`` — both logged; the caller
        records them in :class:`GCodeStreamAborted`.
    """
    retract_ok = False
    quickstop_attempted = False

    # Step 1: retract nozzle
    try:
        log.warning(
            "gcode_intake: abort retract: Z+%.1f mm dry_run=%s",
            config.retract_z_mm,
            dry_run,
        )
        driver.move_relative(  # type: ignore[union-attr]
            0.0, 0.0, config.retract_z_mm,
            dry_run=dry_run,
        )
        retract_ok = True
    except Exception as exc:
        # Step 2: retract failed — try M410 quickstop
        log.error(
            "gcode_intake: retract failed (%s); falling back to quickstop (M410)",
            exc,
        )
        quickstop_attempted = True
        try:
            driver.quickstop(dry_run=dry_run)  # type: ignore[union-attr]
            log.warning("gcode_intake: M410 quickstop sent")
        except Exception as qs_exc:
            log.error("gcode_intake: quickstop also failed: %s", qs_exc)

    # Step 3: laser off (always attempted)
    if laser_driver is not None:
        try:
            log.warning("gcode_intake: abort laser off")
            laser_driver.off()  # type: ignore[union-attr]
        except Exception as exc:
            log.error("gcode_intake: laser off failed: %s", exc)

    return retract_ok, quickstop_attempted


# ---------------------------------------------------------------------------
# Dry-run entry point
# ---------------------------------------------------------------------------


def dry_run_segment(
    source: Union[str, Sequence[str]],
    workspace: DexArmWorkspace,
    *,
    config: Optional[GCodeIntakeConfig] = None,
    frame: Optional[PrintFrameTransform] = None,
    frame_z0: Optional[float] = None,
    strict_dialect: bool = True,
    print_fn: Callable[[str], None] = print,
) -> ParsedSegment:
    """Parse, validate, and print bounding box without commanding any motion.

    This is the ``--dry-run`` entry point.  Call it with a file path wrapper::

        with open("segment.gcode") as f:
            dry_run_segment(f.readlines(), workspace, config=cfg, frame=my_frame)

    Parameters
    ----------
    source:
        G-code content (see :func:`parse_segment`).
    workspace:
        DexArm workspace for bounds validation.
    config:
        :class:`GCodeIntakeConfig`.  Uses defaults when ``None``.
    frame:
        Print-frame transform.  Defaults to :class:`IdentityPrintFrame`.
    frame_z0:
        Leveled frame Z0 (mm).  ``None`` = skip origin check.
    strict_dialect:
        Passed to :func:`parse_segment`.
    print_fn:
        Callable for output (default: builtin ``print``).  Override in tests.

    Returns
    -------
    ParsedSegment
        The parsed segment (even if validation would have failed; returns
        after validation error is re-raised).

    Raises
    ------
    GCodeValidationError
        Any validation rejection.
    """
    if config is None:
        config = GCodeIntakeConfig()

    print_fn("=== G-code dry-run ===")
    segment = parse_segment(source, config=config, strict_dialect=strict_dialect)
    print_fn(f"  Lines total       : {len(segment.lines)}")
    print_fn(f"  Motion lines      : {segment.motion_line_count}")
    print_fn(f"  Bounding box      : {segment.bounding_box}")

    if segment.estimated_time_s is not None:
        mins = int(segment.estimated_time_s // 60)
        secs = int(segment.estimated_time_s % 60)
        print_fn(f"  Estimated time    : {mins}m {secs}s")
    else:
        print_fn("  Estimated time    : unknown (no feedrate found)")

    print_fn(f"  First Z           : {segment.first_z}")

    validate_segment(segment, workspace, config=config, frame=frame, frame_z0=frame_z0)

    frame_id = frame.frame_id if frame is not None else "identity"
    print_fn(f"  Frame ID          : {frame_id}")
    print_fn("  Validation        : PASSED")
    return segment

