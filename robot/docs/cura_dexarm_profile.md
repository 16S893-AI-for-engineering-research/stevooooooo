# Cura DexArm Print Profile — Required Settings (AA-34)

This document lists the Cura slicer settings required to produce G-code
compatible with the inchworm robot's G-code intake module (`inchworm.gcode_intake`).

> **AC#5 status: UNCHECKED — needs owner-supervised hardware session.**
> The nozzle type, material profiles, and several machine limits marked
> "owner-to-confirm" below cannot be determined from software alone.
> Before slicing a real print segment, the owner must fill in every
> field marked **owner-to-confirm** and remove the placeholder.
> See the command at the bottom of this file.

---

## 1. Machine Settings

These values come from the Rotrics DexArm hardware specification (doc-1 §3).
All workspace values are **unverified** placeholders until a live session
measures the as-built envelope.

| Setting | Value | Notes |
|---------|-------|-------|
| Machine name | `Rotrics DexArm` | |
| G-code flavour | `Marlin` | Must be Marlin; RepRap or Smoothieware dialects are not accepted |
| Machine width (X) | `600 mm` (−300 to +300) | unverified: true — measure from DexArm workspace config |
| Machine depth (Y) | `450 mm` (0 to 450) | unverified: true |
| Machine height (Z) | `500 mm` (−100 to +400) | unverified: true |
| Origin at centre | `Yes` (X0,Y0 = arm centre) | Required so the segment origin matches the DexArm arm frame |
| Heated bed | **owner-to-confirm** | Depends on the DexArm print module variant; set True only if bed heater is present |
| Build volume shape | `Rectangular` | |

> **Note on workspace values**: The workspace values above are from the
> Rotrics specification.  The `hardware.yaml` `dexarm.workspace` block is the
> canonical source of truth at runtime.  If the owner updates `hardware.yaml`,
> update this table to match.

---

## 2. Extruder / Nozzle Settings

> **All values in this section are owner-to-confirm.**
> The inchworm robot uses the Rotrics DexArm 3D print module; nozzle size
> and material are chosen by the owner per-print.

| Setting | Placeholder | Owner action required |
|---------|------------|----------------------|
| Nozzle diameter | **owner-to-confirm** | Measure the installed nozzle (e.g. 0.4 mm, 0.6 mm) |
| Nozzle max temperature | **owner-to-confirm** | Check the hotend thermal limit; update `gcode_intake.max_nozzle_temp_c` in `hardware.yaml` |
| Compatible material diameter | **owner-to-confirm** | 1.75 mm or 2.85 mm depending on the print module variant |

---

## 3. Material / Print Settings

> **All values in this section are owner-to-confirm.**
> Default Cura profiles are tuned for desktop printers; the DexArm has
> different thermal mass and feedrate limits.

| Setting | Placeholder | Owner action required |
|---------|------------|----------------------|
| Print temperature | **owner-to-confirm** | Select for the loaded filament; must be ≤ `gcode_intake.max_nozzle_temp_c` |
| Bed temperature | **owner-to-confirm** | 0 if no heated bed; must be ≤ `gcode_intake.max_bed_temp_c` |
| Print speed | **owner-to-confirm** | Confirm max feedrate for the DexArm print module; update `gcode_intake.max_feedrate_mm_min` |
| Travel speed | **owner-to-confirm** | Confirm max travel feedrate; same limit applies |
| Layer height | **owner-to-confirm** | Depends on nozzle diameter (typically 0.1–0.3 × nozzle diameter) |
| Initial layer height | **owner-to-confirm** | Must match `z_origin_tolerance_mm` tuning |
| Retraction | **owner-to-confirm** | Enable if stringing; retraction distance and speed depend on the hotend |
| Cooling fan | **owner-to-confirm** | Enable if material requires it; M106/M107 are in the allowed command set |

---

## 4. Start G-code Constraints

The `inchworm.gcode_intake` parser **forbids G28 anywhere in a segment file**.
The start G-code must NOT include G28 (home all axes) because:

1. The DexArm is already positioned by the inchworm state machine before
   the segment is streamed.
2. G28 would move the arm to the firmware home position mid-print, crashing
   the nozzle into the part.

**Required start G-code** (minimum safe template):

```gcode
; --- AA-34 segment start G-code ---
; No G28 here. The arm is pre-positioned by the robot before streaming.
G21             ; Set units: millimetres
G90             ; Absolute XYZ positioning
M82             ; Absolute extruder mode
G92 E0          ; Reset extruder position
; M104 S{material_print_temperature_layer_0} ; Nozzle temp (if not pre-heated)
; M109 S{material_print_temperature_layer_0} ; Wait for nozzle (if not pre-heated)
```

> **Note**: Temperature commands (M104/M109/M140/M190) in the start G-code
> are validated against the configured limits.  Temperatures above
> `max_nozzle_temp_c` or `max_bed_temp_c` in `hardware.yaml` will be
> rejected by the intake validator.

---

## 5. End G-code Constraints

**Required end G-code** (minimum safe template):

```gcode
; --- AA-34 segment end G-code ---
; No G28 here either.
M104 S0         ; Nozzle heater off
M140 S0         ; Bed heater off (if present)
M84             ; Disable steppers
; The robot moves the DexArm to safe-park via the bounded API after streaming.
```

> Do NOT add G28 to the end G-code.  The inchworm state machine parks the
> arm via `DexArmDriver.safe_park()` after segment streaming completes.

---

## 6. Coordinate System / Origin

The DexArm arm frame has:
- **Origin at centre** (X0, Y0) in the XY plane.
- **Z0** = the leveled print frame Z0 (set by AA-33 per-hop leveling).

The slice must be made relative to this origin.  A segment sliced with a
different origin will fail the Z-origin consistency check (`GCodeOriginError`)
when the first-layer Z does not match the leveled frame Z0 within
`z_origin_tolerance_mm` (default 1.0 mm, unverified).

**Cura setting**: enable "Origin at centre" in Machine Settings.

---

## 7. Allowed G-code Commands

The intake parser accepts only the following Marlin opcodes.  Any other
opcode causes a `GCodeDialectError` (`unrecognised_command`).

| Code | Meaning | Notes |
|------|---------|-------|
| G0 / G1 | Rapid / linear move | Main print motion |
| G2 / G3 | Arc move (CW / CCW) | **Accepted at parse/validate time for bounding-box only; rejected at stream time.** Configure Cura with arc support disabled (Arc Welder off) so all arcs are linearised to G1 moves before slicing. See note below. |
| G4 | Dwell | |
| G21 | Set units to mm | |
| G90 / G91 | Absolute / relative XYZ | Both accepted; tracker keeps state |
| G92 | Set position / reset E | |
| M82 / M83 | Absolute / relative E | Both accepted |
| M84 / M85 | Disable / set inactivity steppers | |
| M104 / M109 | Set / wait nozzle temp | Limited to `max_nozzle_temp_c` |
| M140 / M190 | Set / wait bed temp | Limited to `max_bed_temp_c` |
| M106 / M107 | Fan on / off | |
| M73 | Set print progress | |
| M0 / M1 | Pause / stop | |
| M117 | LCD message | |
| M201 / M203 / M204 / M205 | Acceleration / jerk settings | |
| M900 | Linear advance | |
| M907 | Motor current | |
| M110 | Set line number | |
| M115 | Firmware info | |
| T0 / T1 | Tool select | |

**Explicitly forbidden at stream time**: G28 (home axes) — never allowed in a segment file; G2/G3 arc moves — accepted by the parser/validator but rejected when streaming (configure Cura to linearise arcs; see note below).

> **G2/G3 arc note (AA-34)**: The `stream_segment` function rejects G2/G3 commands with a `GCodeArcRejected` error (reason: `arc_not_linearised`). Arc moves are rejected because the DexArm firmware's arc interpolation is unverified and Cura can be configured without arc support. To disable arcs in Cura: in *Preferences → Configure Cura → Plugins*, ensure "Arc Welder" is not installed or is disabled; alternatively, search for "arc" in Cura print settings and set it to off. G2/G3 are still accepted during *validation* to check the bounding box; only streaming rejects them.
---

## 8. Feedrate Limits

The intake validator rejects any feedrate (F parameter) above
`gcode_intake.max_feedrate_mm_min` (default 9000 mm/min = 150 mm/s).

> This value is **unverified**.  The owner must confirm the DexArm 3D print
> module's real speed limit and update `gcode_intake.max_feedrate_mm_min`
> in `hardware.yaml`.

---

## 9. Reproducibility Checklist

To reproduce a segment slice:

1. Open Cura and select (or create) the "Rotrics DexArm" machine profile
   using the settings in §1.
2. Set nozzle diameter and material profile per §2–3.
3. Apply the start/end G-code from §4–5.
4. Confirm "Origin at centre" is enabled (§6).
5. Export as "Marlin" flavour G-code.
6. Run the dry-run validator before streaming:
   ```
   # owner-to-confirm command — run after AA-34 CLI is wired up
   # uv run python -m inchworm.gcode_intake --dry-run segment.gcode
   ```
7. On validation pass, confirm the bounding box and estimated time are
   within expected limits for the print segment.

---

## AC#5 Owner Action Required

**needs owner-supervised hardware session: confirm nozzle type, material
diameter, print temperature, bed temperature, and speed limits; update
`hardware.yaml` gcode_intake block; recheck this document.**

Specifically:
- Fill in every "owner-to-confirm" field in §2–3.
- Update `gcode_intake.max_nozzle_temp_c`, `max_bed_temp_c`,
  `max_feedrate_mm_min` in `hardware.yaml` to match the confirmed limits.
- Set `unverified: false` on the `dexarm.workspace` block in `hardware.yaml`
  once the workspace bounds are measured.
- Slice a test segment and confirm it passes the intake validator.
