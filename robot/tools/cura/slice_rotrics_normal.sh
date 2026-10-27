#!/usr/bin/env bash
# AA-34: headless reproduction of the Rotrics "Normal-DexArm" Cura profile
# (robot/docs/cura_dexarm_profile.md) with CuraEngine 4.13 (Ubuntu 22.04
# packages cura-engine + cura for fdmprinter.def.json).
#
#   CURAENGINE=/usr/bin/CuraEngine \
#   FDMPRINTER=/usr/share/cura/resources/definitions/fdmprinter.def.json \
#   tools/cura/slice_rotrics_normal.sh model.stl out.gcode
#
# Start/end G-code are the inchworm segment templates (no G28, no M888/M2000,
# see the doc s4-s5), not the Rotrics desktop templates.
set -euo pipefail
in="$1"; out="$2"
CURAENGINE="${CURAENGINE:-CuraEngine}"
FDMPRINTER="${FDMPRINTER:-/usr/share/cura/resources/definitions/fdmprinter.def.json}"
start=$'G21\nG90\nM82\nG92 E0\nM104 S205\nM109 S205'
end=$'M107\nM104 S0\nG91\nG1 Z20 F600\nG90'
"$CURAENGINE" slice -j "$FDMPRINTER" \
  -s machine_width=150 -s machine_depth=150 -s machine_height=270 \
  -s machine_center_is_zero=true -s machine_heated_bed=false \
  -s machine_gcode_flavor="RepRap (Marlin/Sprinter)" \
  -s material_print_temp_prepend=false -s material_bed_temp_prepend=false \
  -s machine_start_gcode="$start" -s machine_end_gcode="$end" \
  -s adhesion_type=skirt -s jerk_enabled=false -s layer_height=0.2 -s layer_height_0=0.2 \
  -s retraction_combing=noskin \
  -e0 -s machine_nozzle_size=0.4 -s material_diameter=1.75 \
  -s material_print_temperature=205 -s material_initial_print_temperature=205 \
  -s material_final_print_temperature=205 -s material_print_temperature_layer_0=205 \
  -s retraction_amount=6 -s retraction_speed=30 -s retraction_retract_speed=30 -s retraction_prime_speed=30 \
  -s speed_print=30 -s speed_infill=30 -s speed_wall=20 -s speed_wall_0=20 -s speed_wall_x=20 \
  -s speed_topbottom=15 -s speed_roofing=15 -s speed_layer_0=20 -s speed_print_layer_0=20 \
  -s speed_travel=50 -s speed_travel_layer_0=33 -s skirt_brim_speed=20 \
  -s wall_thickness=0.8 -s infill_sparse_density=20 -s infill_pattern=cubic -s infill_overlap=30 \
  -s infill_line_distance=6.0 -s skirt_line_count=3 -s cool_fan_full_layer=4 \
  -l "$in" -o "$out"
