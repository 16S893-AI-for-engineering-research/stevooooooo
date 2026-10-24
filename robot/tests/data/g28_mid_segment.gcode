; g28_mid_segment.gcode — segment containing a forbidden G28 (AA-34 tests)
; G28 (home) must never appear in a segment file between start and end G-code

G21
G90
M82
G1 Z0.3 F1200
G1 X10 Y10 F3000
G28          ; FORBIDDEN: G28 mid-segment -> GCodeDialectError
G1 X20 Y10 E1.0 F2400
