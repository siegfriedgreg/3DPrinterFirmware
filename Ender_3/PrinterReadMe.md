Copy From RAW View Data!
=========================================================================================================================================================
=========================================================================================================================================================
=========================================================================================================================================================
;Start of custom Start G-Code

M117 HOMING
G28 ; HOMING

G92 E0 ; Reset Extruder
M117 SET HEAT
M104 S{material_print_temperature_layer_0} ; Set Extruder temperature
M109 S{material_print_temperature_layer_0} ; Wait for Extruder temperature

M420 S1 ; LOAD SAVED MESH

M117 MENUVERING
G1 X1.0 Y20  F4000.0 ; move to start-line position
G1 Z1.0 F1000; lower nozzle to bed
G1  F500 E5; Pre Prime for Purge Wipe

M117 PURGING!
G1 X0.1 Y120.0 Z0.3 F1500.0 E15 ; draw 1st line
G1 X0.4 Y120.0 Z0.3 F5000.0 ; move to side a little
G1 X0.4 Y300 Z0.3 F1500.0 E30 ; draw 2nd line but stop short
G92 E0 ; reset extruder
G1 Z1.0 F3000 ; move z up little to prevent scratching of surface

M117 MAGIC PRINT!

; End of custom Start G-Code

=========================================================================================================================================================
=========================================================================================================================================================
=========================================================================================================================================================

;Start of custom End G-code

G4 ; Wait

M220 S100 ; Reset Speed factor override percentage to default (100%)
M221 S100 ; Reset Extrude factor override percentage to default (100%)

G91 ; Set coordinates to relative

G1 F1800 E-4 ; Retract filament 4 mm to prevent oozing
G1 F3000 Z20 ; Move Z Axis up 20 mm to allow filament ooze freely

G90 ; Set coordinates to absolute
G1 X0 Y{machine_depth} F1000 ; Move Bed to the front
M106 S0 ; Turn off cooling fan
M104 S0 ; Turn off extruder
M140 S0 ; Turn off bed
M107 ; Turn off Fan

M84 X Y E ;Disable all steppers but Z

;End of custom End G-Code
