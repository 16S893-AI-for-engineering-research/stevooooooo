# Cura DexArm Print Profile — Required Settings (AA-34)

These are the Cura slicer settings for the Rotrics DexArm 3D print module, as
published by Rotrics, plus the changes the inchworm robot needs so that the
G-code passes `inchworm.gcode_intake`.

Owner decision 2026-10-27: **use the settings Rotrics publishes.** Rotrics'
values are used unchanged except for the start/end G-code in §4–§5 and the
origin note in §6, which come from the robot's streaming model.

## 0. Sources (fetched 2026-10-27)

| Source | What it gives |
|---|---|
| Rotrics manual, *Generate 3D Printing G-code with Cura* (https://manual.rotrics.com/get-start/3d-printing/generate-3d-printing-g-code, page dated 2020-08-17) | Machine settings, nozzle 0.4 mm, filament 1.75 mm, start/end G-code |
| Rotrics official Cura profiles, `DexArm-3DPrint-Cura.zip` from the Rotrics Download Center (https://cdn.shopify.com/s/files/1/0255/0195/8216/files/DexArm-3DPrint-Cura.zip?v=1647501175, sha256 `95ec7537046b7405cbbe725b7bd759950bd4a766c7a5f11fe167dfce7b63f7ed`) | Draft / Normal / Fine quality profiles (not vendored: no licence stated; every value is in §2). Inner file sha256: Normal `86c55906…03d9`, Draft `d06ae57a…b081`, Fine `8b58e5e2…85b4` |
| Rotrics firmware, `Rotrics-Dev/Marlin_For_DexArm` `Marlin/Configuration.h` | `HEATER_0_MAXTEMP 275`, `TEMP_SENSOR_BED 0` (no heated bed), `DEFAULT_NOMINAL_FILAMENT_DIA 1.75`, `EXTRUDE_MINTEMP 170` |
| Rotrics firmware, `Marlin/src/gcode/control/M_dexarm.cpp` | `M888 P3` = 3D print module (saves to EEPROM and applies the module tool offset); `M2000` = straight-line G0 mode, `M2001` = fast (joint) G0 mode |

The Rotrics manual gives the 3D print build volume as 220 x 220 x 250 mm and
lists PLA, TPU and carbon-fibre filaments as supported.

## 1. Machine settings (Rotrics Cura guide)

Machine → Add printer → Custom FFF printer, named `Rotrics V1.0`:

| Setting | Value |
|---|---|
| X (width) | 150 mm (Rotrics build plate; use the real print area otherwise) |
| Y (depth) | 150 mm |
| Z (height) | 270 mm |
| Origin at center | **Yes** |
| Heated bed | No (firmware `TEMP_SENSOR_BED 0`) |
| G-code flavor | Marlin |
| Printhead X/Y min/max | 0 |
| Nozzle size | 0.4 mm |
| Compatible material diameter | 1.75 mm |

## 2. Quality profiles (Rotrics `.curaprofile` files)

Import with *Rotrics V1.0 → Manage printers → Profiles → Import*. The robot
uses **Normal-DexArm** by default.

| Setting | Draft | **Normal** | Fine |
|---|---|---|---|
| Layer height | 0.2 | **0.2** | 0.1 |
| Initial layer height | 0.2 | **0.2** | 0.2 |
| Wall thickness | 0.8 | **0.8** | 1.2 |
| Infill density / pattern | 20 % cubic | **20 % cubic** | 20 % cubic |
| Infill overlap / line distance | 30 % / 6.0 mm | **30 % / 6.0 mm** | 30 % / 6.0 mm |
| Print temperature (initial / final) | 205 °C | **205 °C** | 205 °C |
| Print speed | 35 mm/s | **30 mm/s** | 20 mm/s |
| Infill speed | 35 mm/s | **30 mm/s** | 20 mm/s |
| Wall speed (inner / outer) | 20 mm/s | **20 mm/s** | 20 mm/s |
| Initial layer speed | 20 mm/s | **20 mm/s** | 20 mm/s |
| Travel speed | 50 mm/s | **50 mm/s** | 50 mm/s |
| Retraction distance / speed | 6 mm / 30 mm/s | **6 mm / 30 mm/s** | 6 mm / 30 mm/s |
| Adhesion | skirt, 3 lines | **skirt, 3 lines** | skirt, 3 lines |
| Fan full at layer | 4 | **4** | 4 |
| Minimum layer time | 10 s | Cura default | Cura default |
| Jerk control | off | **off** | off |
| Combing | not in skin | **not in skin** | not in skin |

The profiles carry no material bed temperature (no heated bed). Material is
PLA at 205 °C, as in all three profiles. Change the temperature only for
another filament, and keep it ≤ `gcode_intake.max_nozzle_temp_c` (260 °C).

## 3. Arc moves

Keep Cura's Arc Welder plugin off (*Preferences → Configure Cura → Plugins*).
`stream_segment` rejects G2/G3 (`GCodeArcRejected`), because DexArm firmware
arc interpolation has not been verified on this robot.

## 4. Start G-code (robot deviation from Rotrics)

Rotrics' desktop start G-code is:

```gcode
M888 P3   ; select 3D print module (writes EEPROM, applies the tool offset)
G92 E0
M2000     ; straight-line G0 mode
```

For the robot, **remove `M888 P3` and `M2000` from the Cura start G-code**.
The robot sets the module and motion mode once, outside the segment, through
the bounded driver (`DexArmDriver.select_module(MODULE_3D_PRINT)`), for two
reasons:

- M888 rewrites EEPROM and changes the tool offset, so it does not belong in
  every streamed segment.
- `gcode_intake` streams every move through the bounded `print_move` (G1), so
  the G0 mode setting has no effect on segment moves.

The intake rejects unknown opcodes, so a slice that still contains them fails
validation with `unrecognised_command`. Never add G28 (homing): it is a hard
reject anywhere in a segment.

Use this start G-code:

```gcode
G21
G90
M82
G92 E0
M104 S{material_print_temperature_layer_0}
M109 S{material_print_temperature_layer_0}
```

Also turn off Cura's automatic temperature prepend (Machine Settings: untick
the automatic material print temperature insertion, or keep the template
above). Otherwise Cura adds `M105`, which is not on the allowed list.

## 5. End G-code

Rotrics' end G-code is `M106 S0 / M104 S0 / G91 / G0 Z20 / G90`. The robot
version uses only allowed opcodes and a bounded G1 lift:

```gcode
M107
M104 S0
G91
G1 Z20 F600
G90
```

Do not add G28. The state machine parks the DexArm through the bounded API.

## 6. Coordinate origin

The slice is centred (Origin at center = Yes), so Cura's X/Y are relative to
the print point. The AA-33 print frame (`frame=` in `validate_segment` /
`stream_segment`) maps them into the DexArm arm frame per hop. Validation
checks the transformed coordinates against `dexarm.workspace` in
`hardware.yaml`. The first layer Z must match the leveled Z0 within
`z_origin_tolerance_mm`.

## 7. Allowed G-code commands

The intake parser accepts only these opcodes; anything else raises
`GCodeDialectError` (`unrecognised_command`):

G0, G1, G2/G3 (validated for the bounding box only, rejected at stream time),
G4, G21, G90, G91, G92, M82, M83, M84, M85, M104, M109, M140, M190, M106, M107,
M73, M0, M1, M117, M201, M203, M204, M205, M900, M907, M110, M115, T0, T1.
G28 is always rejected.

## 8. Intake limits (hardware.yaml `gcode_intake`)

| Field | Value | Source |
|---|---|---|
| `max_nozzle_temp_c` | 260 | 15 °C under the firmware cutoff (`HEATER_0_MAXTEMP 275`) |
| `max_bed_temp_c` | 0.0001 (no bed) | `TEMP_SENSOR_BED 0`: any non-zero bed temperature is rejected |
| `max_feedrate_mm_min` | 9000 (150 mm/s) | Unchanged; the Rotrics profiles never exceed 50 mm/s (3000 mm/min) |

## 9. Reproducing a slice

Headless (used for the test fixture):

```bash
# Ubuntu 22.04: cura-engine 4.13 and cura (for fdmprinter.def.json)
tools/cura/slice_rotrics_normal.sh model.stl segment.gcode
```

`tests/data/rotrics_normal_10x10x1mm.gcode` was produced this way, with
CuraEngine 4.13.0, on 2026-10-27. `tests/test_gcode_intake.py::TestRotricsProfileSlice`
shows that it parses in strict dialect, validates against the limits above in
an arm-frame offset, and dry-run streams.

In the Cura GUI: §1 printer, download the zip (§0) and import `DexArm-Cura-Normal.curaprofile`,
then the start/end G-code from §4–§5, then slice and *Save to File*. Then:

```python
from inchworm.gcode_intake import dry_run_segment, GCodeIntakeConfig
# workspace = cfg.dexarm.workspace; frame = AA-33 print frame for the hop
dry_run_segment(open("segment.gcode").read(), workspace, config=..., frame=frame, frame_z0=...)
```

## 10. Still open (owner, at the hardware)

- Confirm the installed nozzle really is the stock 0.4 mm, and that the
  filament is 1.75 mm PLA.
- Do the first real print segment on the robot (AA-35 integrates printing).
