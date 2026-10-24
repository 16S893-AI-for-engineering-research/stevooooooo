; bed_temp_exceeded.gcode — segment with bed temperature above the limit (AA-34 tests)
; Default max_bed_temp_c=120; this sets 130 -> GCodeTempError

G21
G90
M82
M140 S130  ; 130°C exceeds max_bed_temp_c=120 -> GCodeTempError
G1 Z0.3 F1200
G1 X10 Y10 F3000
G1 X20 Y10 E1.0 F2400
