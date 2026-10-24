"""DexArm Marlin-style G-code driver (AA-12).

Drives the Rotrics DexArm over a :class:`~inchworm.backends.SerialTransport`
using a Marlin-compatible G-code dialect.  Key design choices:

* **Serialised command queue** — a :class:`threading.Lock` ensures that
  concurrent callers cannot interleave G-code lines on the wire.
* **Blocking acknowledgement** — every command waits for ``ok`` (or an
  error token) before returning; callers can rely on the arm being settled.
* **Pre-transmission bounds check** — workspace limits are enforced from
  :attr:`DexArmDriver.workspace` *before* any byte is sent.
* **Typed errors** — distinct exception classes for disconnects, protocol
  errors, bad commands, and bounds violations.
* **Dry-run mode** — every motion entry point honours a ``dry_run=True``
  parameter that logs the planned command without sending it.

Rotrics-specific commands (e.g. ``M888`` for module select) are noted in
docstrings as *unverified* because they cannot be validated without a live
device.

Telemetry is emitted via :class:`~inchworm.telemetry.SessionLogger` when one
is supplied; if none is given all telemetry calls are no-ops.

Wire encoding
--------------
Commands are sent as ASCII terminated by ``\\n`` (``0x0A``).  The firmware
replies with zero or more informational lines followed by ``ok\\n`` on
success or ``error:<reason>\\n`` on rejection.

Units
-----
All public-API position values are in **millimetres**.  Feedrate is in
**mm/min**.
"""

from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from inchworm.backends import SerialTransport
from inchworm.config import DexArmWorkspace

# ---------------------------------------------------------------------------
# Public typed errors
# ---------------------------------------------------------------------------


class DexArmError(RuntimeError):
    """Base class for all DexArm driver errors."""


class DexArmDisconnect(DexArmError):
    """Raised when the transport is closed or the device stops responding.

    Contains the partial response already received, if any.
    """

    def __init__(self, message: str, partial: str = "") -> None:
        super().__init__(message)
        self.partial = partial


class DexArmCommandError(DexArmError):
    """Raised when the firmware rejects a command.

    :attr:`command` is the G-code line that was sent.
    :attr:`firmware_response` is the raw reply from the firmware.
    """

    def __init__(self, command: str, firmware_response: str) -> None:
        super().__init__(
            f"firmware rejected {command!r}: {firmware_response!r}"
        )
        self.command = command
        self.firmware_response = firmware_response


class DexArmBoundsError(DexArmError):
    """Raised when a commanded position is outside the configured workspace.

    The move is *never transmitted* to the firmware.
    """

    def __init__(self, x: float, y: float, z: float, workspace: DexArmWorkspace) -> None:
        super().__init__(
            f"position ({x:.3f}, {y:.3f}, {z:.3f}) mm is outside workspace "
            f"X[{workspace.x_min_mm}, {workspace.x_max_mm}] "
            f"Y[{workspace.y_min_mm}, {workspace.y_max_mm}] "
            f"Z[{workspace.z_min_mm}, {workspace.z_max_mm}]"
        )
        self.x = x
        self.y = y
        self.z = z
        self.workspace = workspace


class DexArmNoBoundsError(DexArmError):
    """Raised when a motion command is attempted with no workspace configured.

    Bounds enforcement is **fail-closed**: the driver refuses to move without
    a configured workspace so that a missing or unconfigured workspace cannot
    silently allow unbounded motion.  Pass an explicit
    :class:`~inchworm.config.DexArmWorkspace` to the constructor to enable
    motion commands.

    The only exception is :meth:`DexArmDriver.home` (G28), which is permitted
    because homing is a recovery action that moves to the factory home position
    regardless of current state.
    """

    def __init__(self, command: str = "") -> None:
        detail = f" (command: {command!r})" if command else ""
        super().__init__(
            f"motion command refused: no workspace bounds configured{detail}. "
            "Construct DexArmDriver with a DexArmWorkspace to enable motion."
        )
        self.command = command


class DexArmMotionInRawError(DexArmError):
    """Raised when a motion G-code is passed to send_raw or select_module.

    Only the bounded motion API (move_absolute, move_relative, home, safe_park)
    may issue motion commands.  Direct raw G-code paths do not enforce workspace
    bounds, so they refuse motion codes entirely.

    Blocked codes: G0, G1, G2, G3, G28, G29 and their zero-padded forms
    (G00, G01, G02, G03), case-insensitive, including with leading line numbers
    (e.g. ``N10 G1 X10``).
    """

    def __init__(self, gcode_line: str, matched_code: str) -> None:
        super().__init__(
            f"motion G-code {matched_code!r} is not allowed in send_raw/select_module; "
            f"use the bounded motion API (move_absolute, move_relative, home). "
            f"Rejected line: {gcode_line!r}"
        )
        self.gcode_line = gcode_line
        self.matched_code = matched_code


# ---------------------------------------------------------------------------
# Position dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DexArmPosition:
    """XYZ position reported by M114 (mm, arm frame)."""

    x: float
    y: float
    z: float

    def __str__(self) -> str:
        return f"({self.x:.3f}, {self.y:.3f}, {self.z:.3f}) mm"


# ---------------------------------------------------------------------------
# Response parsing helpers
# ---------------------------------------------------------------------------

# Matches "X:nnn.nn Y:nnn.nn Z:nnn.nn" (M114 reply).
# The DexArm firmware may include E: and Count fields; we capture only X/Y/Z.
_M114_RE = re.compile(
    r"X:\s*(-?[\d.]+)\s+Y:\s*(-?[\d.]+)\s+Z:\s*(-?[\d.]+)",
    re.IGNORECASE,
)

# Motion G-codes that are blocked in send_raw and select_module.
# Blocked codes: G0, G1, G2, G3, G00, G01, G02, G03, G28, G29 (case-insensitive).
# Supports leading line numbers like "N10 G1 X10".
# Only the bounded motion API (move_absolute, move_relative, home, safe_park)
# may issue these codes; send_raw and select_module cannot bypass bounds.
# Matches any motion G-code anywhere in the line: G0-G3 (with optional
# leading zeros), G5 (Bezier), G28 (home), G29 (bed level), with any
# following character that is not a digit, so "G0X5", "G1X0;c" and
# "N10 G1 X1" are caught, while G4/G10/G12/G92 are not.
# Orchestrator fix 2026-10-24: the original pattern required whitespace
# after the code and missed Marlin's no-space form.
_MOTION_GCODE_RE = re.compile(
    r"(?<![A-Z0-9.])G0*(0|1|2|3|5|28|29)(?!\d)",
    re.IGNORECASE,
)


def _reject_motion_gcode(gcode_line: str) -> None:
    """Raise :class:`DexArmMotionInRawError` if *gcode_line* contains a motion G-code.

    Motion codes (G0, G1, G2, G3, G28, G29 and zero-padded forms) are blocked
    in :meth:`~DexArmDriver.send_raw` and :meth:`~DexArmDriver.select_module`
    because those paths do not enforce workspace bounds.  Use the bounded motion
    API (:meth:`~DexArmDriver.move_absolute`, :meth:`~DexArmDriver.move_relative`,
    :meth:`~DexArmDriver.home`) for all motion commands.
    """
    m = _MOTION_GCODE_RE.search(gcode_line)
    if m:
        raise DexArmMotionInRawError(gcode_line, f"G{m.group(1)}")


def _parse_m114(lines: Sequence[str]) -> Optional[DexArmPosition]:
    """Extract XYZ from M114 response lines, or return ``None`` on parse fail."""
    for line in lines:
        m = _M114_RE.search(line)
        if m:
            return DexArmPosition(float(m.group(1)), float(m.group(2)), float(m.group(3)))
    return None


def _is_ok(line: str) -> bool:
    return line.strip().lower() == "ok"


def _is_error(line: str) -> bool:
    return line.strip().lower().startswith("error")


# ---------------------------------------------------------------------------
# DexArmDriver
# ---------------------------------------------------------------------------

#: Default timeout in seconds waiting for a firmware acknowledgement.
DEFAULT_ACK_TIMEOUT_S: float = 15.0

#: Position to move to on safe_park().  Chosen to be within the default
#: workspace bounds but well clear of the print area.
#: Owner: adjust after first live session.
_PARK_X: float = 0.0
_PARK_Y: float = 300.0
_PARK_Z: float = 200.0
_PARK_FEEDRATE: float = 3000.0


class DexArmDriver:
    """Marlin-style G-code driver for the Rotrics DexArm.

    Parameters
    ----------
    transport:
        An open :class:`~inchworm.backends.SerialTransport` for the DexArm
        serial device (``"dexarm"``).
    workspace:
        Optional axis-aligned workspace limits.  When supplied, any move
        command whose target position falls outside is rejected with
        :class:`DexArmBoundsError` **before** the command is transmitted.
        Pass ``None`` to skip bounds checking.
    ack_timeout_s:
        Seconds to wait for ``ok`` from the firmware before raising
        :class:`DexArmDisconnect`.
    logger:
        Optional :class:`~inchworm.telemetry.SessionLogger`.  All telemetry
        is skipped when ``None``.
    """

    ROLE = "dexarm"
    SUBSYSTEM = "gcode_driver"

    def __init__(
        self,
        transport: SerialTransport,
        *,
        workspace: Optional[DexArmWorkspace] = None,
        ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S,
        logger: object = None,
    ) -> None:
        self._transport = transport
        self.workspace = workspace
        self._ack_timeout_s = ack_timeout_s
        self._log = logger
        # Serialises all wire access from concurrent threads.
        self._lock = threading.Lock()
        self._firmware_id: Optional[str] = None

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> str:
        """Perform handshake: send ``M115`` and return firmware identity string.

        Logs a WARNING when the configured workspace is marked as
        ``unverified: true`` — the bounds are a placeholder from the Rotrics
        spec and must be replaced with the measured as-built envelope before
        production use.

        Raises :class:`DexArmDisconnect` if the transport is not open or the
        firmware does not respond within ``ack_timeout_s``.
        """
        self._require_open()
        lines = self._send_receive("M115")
        identity = " ".join(
            line for line in lines if line and not _is_ok(line)
        ) or "unknown"
        self._firmware_id = identity
        self._emit("INFO", "dexarm_connected", {"firmware_id": identity})
        # Warn when workspace bounds are unverified (placeholder from spec).
        if self.workspace is not None and getattr(self.workspace, "unverified", False):
            self._emit(
                "WARNING",
                "dexarm_workspace_unverified",
                {
                    "message": (
                        "DexArm workspace bounds are unverified (placeholder from Rotrics spec). "
                        "Replace with measured as-built envelope before production use."
                    )
                },
            )
        return identity

    def disconnect(self) -> None:
        """Close the underlying transport."""
        if self._transport.is_open:
            self._transport.close()
        self._emit("INFO", "dexarm_disconnected", {})

    # ------------------------------------------------------------------
    # Position query
    # ------------------------------------------------------------------

    def get_position(self) -> DexArmPosition:
        """Query current position with M114.

        Returns a :class:`DexArmPosition` (mm, arm frame).

        Raises :class:`DexArmDisconnect` if transport is closed or no
        response arrives within ``ack_timeout_s``.
        """
        self._require_open()
        lines = self._send_receive("M114")
        pos = _parse_m114(lines)
        if pos is None:
            raise DexArmDisconnect(
                "M114 response did not contain position data",
                partial="\n".join(lines),
            )
        self._emit("DEBUG", "dexarm_position_queried", {"x": pos.x, "y": pos.y, "z": pos.z})
        return pos

    # ------------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------------

    def home(self, *, dry_run: bool = False) -> None:
        """Home all axes (G28).

        .. note::
            Homing is permitted even when no workspace is configured because
            G28 is a recovery action that moves to the factory home position
            (a known-safe state) regardless of the current position.  It does
            not require knowledge of the allowed envelope.

        Parameters
        ----------
        dry_run:
            When ``True``, log the command without sending it.
        """
        self._require_open()
        cmd = "G28"
        if dry_run:
            self._emit("INFO", "dexarm_dry_run", {"cmd": cmd, "reason": "home"})
            return
        self._send_receive(cmd)
        self._emit("INFO", "dexarm_homed", {})

    def move_absolute(
        self,
        x: float,
        y: float,
        z: float,
        feedrate: Optional[float] = None,
        *,
        dry_run: bool = False,
    ) -> None:
        """Move to an absolute position in mm (G90 + G0/G1).

        Bounds are checked **before** transmission.

        Parameters
        ----------
        x, y, z:
            Target position in mm (arm frame).
        feedrate:
            Optional move speed in mm/min.  When ``None``, no ``F`` parameter
            is appended and the firmware uses its current feedrate.
        dry_run:
            When ``True``, log the planned command without sending it.

        Raises
        ------
        DexArmBoundsError
            If the target falls outside the configured workspace.
        DexArmDisconnect
            If the transport is closed or times out.
        DexArmCommandError
            If the firmware rejects the command.
        """
        self._require_open()
        self._check_bounds(x, y, z)
        cmd = self._g0_cmd(x, y, z, feedrate)
        if dry_run:
            self._emit(
                "INFO", "dexarm_dry_run",
                {"cmd": cmd, "x": x, "y": y, "z": z, "feedrate": feedrate},
            )
            return
        # G90 = absolute positioning mode; G0/G1 = rapid/linear move.
        with self._lock:
            self._send_cmd("G90")
            self._recv_ack("G90")
            self._send_cmd(cmd)
            self._recv_ack(cmd)
        self._emit(
            "INFO", "dexarm_move_absolute",
            {"x": x, "y": y, "z": z, "feedrate": feedrate},
        )

    def move_relative(
        self,
        dx: float,
        dy: float,
        dz: float,
        feedrate: Optional[float] = None,
        *,
        dry_run: bool = False,
    ) -> None:
        """Move by a relative offset in mm (G91 + G0/G1).

        Bounds are checked using the *current* position plus the delta.
        The position is queried from the firmware just before the move.

        Parameters
        ----------
        dx, dy, dz:
            Offset in mm.
        feedrate:
            Optional move speed in mm/min.
        dry_run:
            When ``True``, query the current position and log the planned
            absolute target without sending motion commands.

        Raises
        ------
        DexArmBoundsError
            If the resolved target falls outside the workspace.
        """
        self._require_open()
        # Resolve the absolute target for bounds checking.
        # We need the lock held across the whole sequence to keep it atomic.
        with self._lock:
            cur_lines = self._send_receive_locked("M114")
            cur = _parse_m114(cur_lines)
            if cur is None:
                raise DexArmDisconnect(
                    "M114 before relative move returned no position data",
                    partial="\n".join(cur_lines),
                )
            tx, ty, tz = cur.x + dx, cur.y + dy, cur.z + dz
            self._check_bounds(tx, ty, tz)
            cmd = self._g0_cmd(tx, ty, tz, feedrate)
            if dry_run:
                self._emit(
                    "INFO", "dexarm_dry_run",
                    {"cmd": cmd, "dx": dx, "dy": dy, "dz": dz,
                     "resolved": {"x": tx, "y": ty, "z": tz}, "feedrate": feedrate},
                )
                return
            # Use absolute mode for the actual move (bounds already checked).
            self._send_cmd("G90")
            self._recv_ack("G90")
            self._send_cmd(cmd)
            self._recv_ack(cmd)
        self._emit(
            "INFO", "dexarm_move_relative",
            {"dx": dx, "dy": dy, "dz": dz, "feedrate": feedrate},
        )

    def set_feedrate(self, feedrate_mm_min: float, *, dry_run: bool = False) -> None:
        """Set the current feedrate with G1 F<rate> (no-move variant).

        Parameters
        ----------
        feedrate_mm_min:
            Speed in mm/min.
        dry_run:
            When ``True``, log without sending.
        """
        self._require_open()
        cmd = f"G1 F{feedrate_mm_min:.3f}"
        if dry_run:
            self._emit("INFO", "dexarm_dry_run", {"cmd": cmd, "feedrate": feedrate_mm_min})
            return
        self._send_receive(cmd)
        self._emit("INFO", "dexarm_feedrate_set", {"feedrate_mm_min": feedrate_mm_min})

    def safe_park(self, *, dry_run: bool = False) -> None:
        """Move the arm to the documented safe-park position and home.

        Safe park: X=0, Y=300, Z=200 mm at 3000 mm/min, then G28.
        These values are within the default workspace; owner should verify
        after the first live session.

        Parameters
        ----------
        dry_run:
            When ``True``, log the planned sequence without sending any commands.
        """
        self._require_open()
        if dry_run:
            self._emit(
                "INFO", "dexarm_dry_run",
                {
                    "reason": "safe_park",
                    "park": {"x": _PARK_X, "y": _PARK_Y, "z": _PARK_Z,
                              "feedrate_mm_min": _PARK_FEEDRATE},
                },
            )
            return
        self.move_absolute(_PARK_X, _PARK_Y, _PARK_Z, feedrate=_PARK_FEEDRATE)
        self.home()
        self._emit("INFO", "dexarm_safe_parked", {})

    # ------------------------------------------------------------------
    # Rotrics-specific commands (unverified without live device)
    # ------------------------------------------------------------------

    # --- AA-34 ---
    def print_move(
        self,
        x: float,
        y: float,
        z: float,
        e: Optional[float] = None,
        f: Optional[float] = None,
        *,
        dry_run: bool = False,
    ) -> None:
        """Issue a bounds-checked G1 print move carrying optional E (extrusion) and F (feedrate).

        This is the **only** path through which segment G-code motion reaches
        the DexArm during printing.  It enforces workspace bounds *before*
        any byte is sent, so a misconfigured print-frame transform or
        rogue G-code value cannot move the arm outside the safe envelope.

        Parameters
        ----------
        x, y, z:
            Target position in mm (arm frame, after print-frame transform).
        e:
            Extruder axis value in mm.  Passed through verbatim; the caller
            (``stream_segment``) is responsible for resolving absolute vs
            relative extrusion before this call.
        f:
            Feedrate in mm/min.  When ``None``, no ``F`` parameter is appended
            and the firmware uses its current feedrate.
        dry_run:
            When ``True``, log the planned command without sending it.

        Raises
        ------
        DexArmBoundsError
            If the target XYZ falls outside the configured workspace.
            **The move is never transmitted when this is raised.**
        DexArmNoBoundsError
            If no workspace is configured (fail-closed).
        DexArmDisconnect
            If the transport is closed or times out.
        DexArmCommandError
            If the firmware rejects the command.
        """
        self._require_open()
        self._check_bounds(x, y, z)
        # Build G1 command: "G1 X<x> Y<y> Z<z> [E<e>] [F<f>]"
        cmd = f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f}"
        if e is not None:
            cmd += f" E{e:.5f}"
        if f is not None:
            cmd += f" F{f:.3f}"
        if dry_run:
            self._emit(
                "INFO", "dexarm_dry_run",
                {"cmd": cmd, "x": x, "y": y, "z": z, "e": e, "feedrate": f},
            )
            return
        with self._lock:
            self._send_cmd("G90")   # ensure absolute XYZ mode
            self._recv_ack("G90")
            self._send_cmd(cmd)
            self._recv_ack(cmd)
        self._emit(
            "INFO", "dexarm_print_move",
            {"x": x, "y": y, "z": z, "e": e, "feedrate": f},
        )
    # --- AA-34 end ---

    # --- AA-10 ---
    def quickstop(self, *, dry_run: bool = False) -> None:
        """Send the Marlin M410 quickstop command to halt all motion immediately.

        M410 is the Marlin quickstop G-code that cancels the motion queue and
        stops the steppers as fast as possible (without homing). It is the
        DexArm emergency-stop command used by the AA-10 e-stop path.

        .. note::
            UNVERIFIED on live hardware. Marlin's M410 is standard but the
            Rotrics DexArm firmware may respond differently. The first live
            e-stop session must confirm the response and update this note.

        Parameters
        ----------
        dry_run:
            When ``True``, log without sending.
        """
        self._require_open()
        cmd = "M410"
        if dry_run:
            self._emit("INFO", "dexarm_dry_run", {"cmd": cmd, "reason": "quickstop"})
            return
        self._send_receive(cmd)
        self._emit("WARNING", "dexarm_quickstop", {"cmd": cmd})
    # --- AA-10 end ---

    def select_module(self, module_id: int, *, dry_run: bool = False) -> None:
        """Select the DexArm tool module with M888.

        .. warning::
            **UNVERIFIED** — M888 is a Rotrics extension and has not been
            tested against live hardware.  Do not rely on this command until
            a live session confirms the correct module IDs and protocol.

        Parameters
        ----------
        module_id:
            Rotrics module ID integer (0 = 3D print head per Rotrics docs,
            but this is unverified).
        dry_run:
            When ``True``, log without sending.
        """
        self._require_open()
        cmd = f"M888 P{module_id}"
        _reject_motion_gcode(cmd)
        if dry_run:
            self._emit(
                "INFO", "dexarm_dry_run",
                {"cmd": cmd, "module_id": module_id, "note": "unverified-rotrics-extension"},
            )
            return
        self._send_receive(cmd)
        self._emit(
            "WARNING", "dexarm_module_selected",
            {"module_id": module_id, "note": "M888 is unverified without live hardware"},
        )

    # ------------------------------------------------------------------
    # Raw command queue (public, serialised)
    # ------------------------------------------------------------------

    def send_raw(self, gcode_line: str, *, dry_run: bool = False) -> list[str]:
        """Send a single G-code line and return the firmware response lines.

        The command queue lock is held for the duration so that concurrent
        callers cannot interleave.  Use this for custom or future commands
        not yet wrapped by named methods.

        Parameters
        ----------
        gcode_line:
            A single G-code command, e.g. ``"G92 E0"``.  Do not include the
            trailing newline; it is added automatically.
        dry_run:
            When ``True``, log without sending and return an empty list.

        Returns
        -------
        list[str]
            All response lines before and including the final ``ok``.
        """
        self._require_open()
        _reject_motion_gcode(gcode_line)
        if dry_run:
            self._emit("INFO", "dexarm_dry_run", {"cmd": gcode_line})
            return []
        return self._send_receive(gcode_line)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_open(self) -> None:
        if not self._transport.is_open:
            raise DexArmDisconnect("DexArm transport is not open")

    def _check_bounds(self, x: float, y: float, z: float) -> None:
        # Fail-closed: refuse motion without a configured workspace so that a
        # missing or unconfigured workspace cannot silently allow unbounded motion.
        if self.workspace is None:
            raise DexArmNoBoundsError(f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f}")
        ws = self.workspace
        if (
            x < ws.x_min_mm or x > ws.x_max_mm
            or y < ws.y_min_mm or y > ws.y_max_mm
            or z < ws.z_min_mm or z > ws.z_max_mm
        ):
            raise DexArmBoundsError(x, y, z, ws)

    @staticmethod
    def _g0_cmd(
        x: float, y: float, z: float, feedrate: Optional[float]
    ) -> str:
        cmd = f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f}"
        if feedrate is not None:
            cmd += f" F{feedrate:.3f}"
        return cmd

    def _send_receive(self, cmd: str) -> list[str]:
        """Acquire the lock, send *cmd*, wait for ack, return response lines."""
        with self._lock:
            return self._send_receive_locked(cmd)

    def _send_receive_locked(self, cmd: str) -> list[str]:
        """Send *cmd* and wait for ack.  **Caller must hold** ``self._lock``."""
        self._send_cmd(cmd)
        return self._recv_ack(cmd)

    def _send_cmd(self, cmd: str) -> None:
        """Write ``cmd\\n`` to the transport."""
        if not self._transport.is_open:
            raise DexArmDisconnect("DexArm transport closed during send", partial="")
        line = (cmd.rstrip("\n") + "\n").encode("ascii")
        self._transport.write(line)

    def _recv_ack(self, cmd: str) -> list[str]:
        """Read lines until ``ok`` or ``error:``.  Returns all accumulated lines."""
        deadline = time.monotonic() + self._ack_timeout_s
        buf = bytearray()
        lines: list[str] = []

        while True:
            if time.monotonic() > deadline:
                raise DexArmDisconnect(
                    f"timeout waiting for ack on {cmd!r} after {self._ack_timeout_s}s",
                    partial="\n".join(lines),
                )
            if not self._transport.is_open:
                raise DexArmDisconnect(
                    f"transport closed while waiting for ack on {cmd!r}",
                    partial="\n".join(lines),
                )
            chunk = self._transport.read(256)
            if chunk:
                buf.extend(chunk)
            # Process complete lines
            while b"\n" in buf:
                nl = buf.index(b"\n")
                raw_line = buf[:nl]
                del buf[: nl + 1]
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    lines.append(line)
                if _is_ok(line):
                    return lines
                if _is_error(line):
                    raise DexArmCommandError(cmd, line)
            # Avoid busy-spinning when no data arrives.
            if not chunk:
                time.sleep(0.002)

    def _emit(self, severity: str, event_type: str, payload: dict) -> None:
        if self._log is None:
            return
        try:
            self._log.emit(
                role=self.ROLE,
                subsystem=self.SUBSYSTEM,
                event_type=event_type,
                severity=severity,  # type: ignore[arg-type]
                payload=payload,
            )
        except Exception:  # pragma: no cover
            pass


# ---------------------------------------------------------------------------
# Mock DexArm serial backend
# ---------------------------------------------------------------------------


class MockDexArmSerial:
    """Minimal mock serial transport that auto-responds to DexArm G-code.

    Install it as the ``transport`` argument to :class:`DexArmDriver` in tests.

    Features
    ---------
    * Responds to ``M115`` with a firmware identity line + ``ok``.
    * Responds to ``M114`` with an ``X:Y:Z:`` position line + ``ok``; the
      position tracks ``G0``/``G1`` commands so move tests see consistent state.
    * Responds to all other commands with ``ok``.
    * Supports :meth:`inject_error` to make the next command return
      ``error:<reason>``.
    * All bytes written are recorded in :attr:`written`.
    """

    FIRMWARE_ID = "FIRMWARE_NAME:Rotrics DexArm FIRMWARE_VERSION:1.0-mock"

    def __init__(self) -> None:
        self._open = True
        self._rx = bytearray()
        self.written: list[bytes] = []
        # Simulated position (mm).
        self._pos_x: float = 0.0
        self._pos_y: float = 300.0
        self._pos_z: float = 200.0
        # Next-response override (for error injection).
        self._error_next: Optional[str] = None

    # -- SerialTransport interface --

    @property
    def is_open(self) -> bool:
        return self._open

    def write(self, data: bytes) -> int:
        if not self._open:
            raise RuntimeError("MockDexArmSerial is closed")
        self.written.append(bytes(data))
        self._handle_command(data.decode("ascii", errors="replace").strip())
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def close(self) -> None:
        self._open = False

    # -- Test helpers --

    def inject_error(self, reason: str = "invalid command") -> None:
        """Cause the *next* command to respond with ``error:<reason>``."""
        self._error_next = reason

    def set_position(self, x: float, y: float, z: float) -> None:
        """Directly set the simulated arm position."""
        self._pos_x = x
        self._pos_y = y
        self._pos_z = z

    # -- Internal response generation --

    def _enqueue(self, text: str) -> None:
        self._rx.extend((text + "\n").encode("ascii"))

    def _handle_command(self, raw: str) -> None:
        cmd = raw.strip().upper()

        if self._error_next is not None:
            self._enqueue(f"error:{self._error_next}")
            self._error_next = None
            return

        if cmd.startswith("M115"):
            self._enqueue(self.FIRMWARE_ID)
            self._enqueue("ok")
        elif cmd.startswith("M114"):
            self._enqueue(
                f"X:{self._pos_x:.3f} Y:{self._pos_y:.3f} Z:{self._pos_z:.3f} E:0.000"
            )
            self._enqueue("ok")
        elif cmd.startswith("G28"):
            # Homing resets to a known position.
            self._pos_x = 0.0
            self._pos_y = 300.0
            self._pos_z = 200.0
            self._enqueue("ok")
        # --- AA-10 ---
        elif cmd.startswith("M410"):
            # Quickstop: halt all motion. Mock just acknowledges.
            self._enqueue("ok")
        # --- AA-10 end ---
        elif cmd.startswith("G90") or cmd.startswith("G91"):
            self._enqueue("ok")
        elif cmd.startswith("G0 ") or cmd.startswith("G1 "):
            self._parse_move(cmd)
            self._enqueue("ok")
        else:
            self._enqueue("ok")

    def _parse_move(self, cmd: str) -> None:
        """Update simulated position from a G0/G1 command string."""
        for axis, attr in (("X", "_pos_x"), ("Y", "_pos_y"), ("Z", "_pos_z")):
            m = re.search(rf"{axis}(-?[\d.]+)", cmd)
            if m:
                setattr(self, attr, float(m.group(1)))
