; feedrate_exceeded.gcode — segment with feedrate above the limit (AA-34 tests)
; Default max_feedrate_mm_min=9000; this uses 12000 -> GCodeFeedrateError

G21
G90
M82
G1 Z0.3 F1200
G1 X10 Y10 F12000  ; 12000 mm/min exceeds max_feedrate_mm_min=9000 -> GCodeFeedrateError
G1 X20 Y10 E1.0 F2400
