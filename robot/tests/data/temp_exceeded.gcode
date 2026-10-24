; temp_exceeded.gcode — segment with nozzle temperature above the limit (AA-34 tests)
; Default max_nozzle_temp_c=280; this sets 300 -> GCodeTempError

G21
G90
M82
M104 S300  ; 300°C exceeds max_nozzle_temp_c=280 -> GCodeTempError
G1 Z0.3 F1200
G1 X10 Y10 F3000
G1 X20 Y10 E1.0 F2400
