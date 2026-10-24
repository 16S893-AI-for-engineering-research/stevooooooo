; out_of_bounds.gcode — segment that moves outside the DexArm workspace (AA-34 tests)
; The workspace is X[-300,300] Y[0,450] Z[-100,400]
; This segment goes to Y=500 which is outside Y_max=450

G21
G90
M82
G1 Z0.3 F1200
G1 X10 Y10 F3000
G1 X20 Y500 E1.0 F2400  ; Y=500 exceeds workspace Y_max=450 -> GCodeBoundsError
