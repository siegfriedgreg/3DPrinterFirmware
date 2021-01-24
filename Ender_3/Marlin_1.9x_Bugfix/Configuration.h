//===========================================================================
//===========================================================================
//===========================================================================
//========================== START OF FILE ==================================
//===========================================================================
//===========================================================================
//===========================================================================

#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#define CONFIGURATION_H_VERSION 010109

//===========================================================================
#define STRING_CONFIG_H_AUTHOR "(GSieg_v4, Ender-3)" // Who made the changes.
//#define SHOW_BOOTSCREEN
//#define STRING_SPLASH_LINE1 SHORT_BUILD_VERSION // will be shown during bootup in line 1
//#define STRING_SPLASH_LINE2 WEBSITE_URL         // will be shown during bootup in line 2

// Enable to show the bitmap in Marlin/_Bootscreen.h on startup.
#define SHOW_CUSTOM_BOOTSCREEN

// Enable to show the bitmap in Marlin/_Statusscreen.h on the status screen.
#define CUSTOM_STATUS_SCREEN_IMAGE

// @section machine
#define SERIAL_PORT 0
#define BAUDRATE 115200

// Please choose the name from boards.h that matches your setup
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_MELZI_CREALITY
#endif

// Displayed in the LCD "Ready" message
#define CUSTOM_MACHINE_NAME "3nder-E"

#define EXTRUDERS 1
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75
#define POWER_SUPPLY 0

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_BED 1

// Dummy thermistor constant temperature readings, for use with 998 and 999
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100

#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// EXTRUDER HYSTERESIS
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// BED HYSTERESIS
#define TEMP_BED_RESIDENCY_TIME 10  // (seconds)
#define TEMP_BED_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_BED_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// HEATER MIN TEMP
#define HEATER_0_MINTEMP 5
#define BED_MINTEMP 5

// HEATER MAX TEMP
#define HEATER_0_MAXTEMP 300
#define BED_MAXTEMP 135

//===========================================================================
//============================= PID Settings ================================
//===========================================================================

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255     // Limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_K1 0.95      // Smoothing factor within any PID loop
#if ENABLED(PIDTEMP)
  #define PID_AUTOTUNE_MENU // Add PID Autotune to the LCD "Temperature" menu to run M303 and apply the result.
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  //#define PID_PARAMS_PER_HOTEND // Uses separate PID parameters for each extruder (useful for mismatched extruders)
                                  // Set/get with gcode: M301 E[extruder number, 0-2]
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.

  // If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
  // Creality Ender-3
  #define DEFAULT_Kp 21.73
  #define DEFAULT_Ki 1.54
  #define DEFAULT_Kd 76.55

#endif // PIDTEMP

//===========================================================================
//============================= PID > Bed Temperature Control ===============
//===========================================================================

#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

// @section extruder
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170

#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200

//===========================================================================
//======================== Thermal Runaway Protection =======================
//===========================================================================

#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed

//===========================================================================
//============================== Endstop Settings ===========================
//===========================================================================

#define USE_XMIN_PLUG
#define USE_YMIN_PLUG
#define USE_ZMIN_PLUG

// Enable pullup for all endstops to prevent a floating state
#define ENDSTOPPULLUPS
#if DISABLED(ENDSTOPPULLUPS)
  // Disable ENDSTOPPULLUPS to set pullups individually
  //#define ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX
  //#define ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
  //#define ENDSTOPPULLUP_ZMIN_PROBE
#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define Z_MIN_PROBE_ENDSTOP_INVERTING false // set to true to invert the logic of the probe.

/**
 * Stepper Drivers
 *
 * These settings allow Marlin to tune stepper driver timing and enable advanced options for
 * stepper drivers that support them. You may also override timing options in Configuration_adv.h.
 *
 * A4988 is assumed for unspecified drivers.
 *
 * Options: A4988, DRV8825, LV8729, L6470, TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2208, TMC2208_STANDALONE,
 *          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE
 * :['A4988', 'DRV8825', 'LV8729', 'L6470', 'TB6560', 'TB6600', 'TMC2100', 'TMC2130', 'TMC2130_STANDALONE', 'TMC2208', 'TMC2208_STANDALONE', 'TMC26X', 'TMC26X_STANDALONE', 'TMC2660', 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE']
 */
#define X_DRIVER_TYPE  TMC2208_STANDALONE
#define Y_DRIVER_TYPE  TMC2208_STANDALONE
#define Z_DRIVER_TYPE  TMC2208_STANDALONE
#define E0_DRIVER_TYPE TMC2208_STANDALONE


// Enable this feature if all enabled endstop pins are interrupt-capable.
// This will remove the need to poll the interrupt pins, saving many CPU cycles.
//#define ENDSTOP_INTERRUPTS_FEATURE

//=============================================================================
//============================== Movement Settings ============================
//=============================================================================

/**
 * Default Axis Steps Per Unit (steps/mm)
 * Override with M92
 *                                      X, Y, Z, E0 [, E1[, E2[, E3[, E4]]]]
 */
#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400, 143 }

/**
 * Default Max Feed Rate (mm/s)
 * Override with M203
 *                                      X, Y, Z, E0 [, E1[, E2[, E3[, E4]]]]
 */
#define DEFAULT_MAX_FEEDRATE          { 300, 300, 5, 25 }

/**
 * Default Max Acceleration (change/s) change = mm/s
 * (Maximum start speed for accelerated moves)
 * Override with M201
 *                                      X, Y, Z, E0 [, E1[, E2[, E3[, E4]]]]
 */
#define DEFAULT_MAX_ACCELERATION      { 300, 300, 100, 1000 }

/**
 * Default Acceleration (change/s) change = mm/s
 * Override with M204
 *
 *   M204 P    Acceleration
 *   M204 R    Retract Acceleration
 *   M204 T    Travel Acceleration
 */
#define DEFAULT_ACCELERATION          200    // X, Y, Z and E acceleration for printing moves
#define DEFAULT_RETRACT_ACCELERATION  200    // E acceleration for retracts
#define DEFAULT_TRAVEL_ACCELERATION   200    // X, Y, Z acceleration for travel (non printing) moves

/**
 * Default Jerk (mm/s)
 * Override with M205 X Y Z E
 *
 * "Jerk" specifies the minimum speed change that requires acceleration.
 * When changing speed and direction, if the difference is less than the
 * value set here, it may happen instantaneously.
 */
#define DEFAULT_XJERK                 8.0
#define DEFAULT_YJERK                 8.0
#define DEFAULT_ZJERK                 0.3
#define DEFAULT_EJERK                 5.0

//===========================================================================
//============================= Z Probe Options =============================
//===========================================================================
// @section probes

#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
#define BLTOUCH
#define SERVO0_PIN 27
#define X_PROBE_OFFSET_FROM_EXTRUDER -47  // X offset: -left  +right  [of the nozzle]
#define Y_PROBE_OFFSET_FROM_EXTRUDER -8  // Y offset: -front +behind [the nozzle]
#define Z_PROBE_OFFSET_FROM_EXTRUDER -1.55   // Z offset: -below +above  [the nozzle]

// Certain types of probes need to stay away from edges
#define MIN_PROBE_EDGE 10

// X and Y axis travel speed (mm/m) between probes
#define XY_PROBE_SPEED 4000

// Feedrate (mm/m) for the first approach when double-probing (MULTIPLE_PROBING == 2)
#define Z_PROBE_SPEED_FAST HOMING_FEEDRATE_Z

// Feedrate (mm/m) for the "accurate" probe of each point
#define Z_PROBE_SPEED_SLOW (Z_PROBE_SPEED_FAST / 2)

#define MULTIPLE_PROBING 2

#define Z_CLEARANCE_DEPLOY_PROBE   10 // Z Clearance for Deploy/Stow
#define Z_CLEARANCE_BETWEEN_PROBES  5 // Z Clearance between probe points
#define Z_CLEARANCE_MULTI_PROBE     5 // Z Clearance between multiple probes
//#define Z_AFTER_PROBING           5 // Z position after probing is done

#define Z_PROBE_LOW_POINT          -2 // Farthest distance below the trigger-point to go before stopping

// For M851 give a range for adjusting the Z probe offset
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{ 0:'Low', 1:'High' }
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis stepper immediately when it's not being used.
// WARNING: When motors turn off there is a chance of losing position accuracy!
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false

// @section extruder
#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER false // Keep only the active extruder enabled.
// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E0_DIR true
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false

// @section machine
// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR false

// @section homing

//#define NO_MOTION_BEFORE_HOMING  // Inhibit movement until all axes have been homed

//#define UNKNOWN_Z_NO_RAISE // Don't raise Z (lower the bed) if Z is "unknown." For beds that fall when Z is powered off.

//#define Z_HOMING_HEIGHT 4  // (in mm) Minimal z height before homing (G28) for Z clearance above the bed, clamps, ...
                             // Be sure you have this distance over your Z_MAX_POS in case.

// Direction of endstops when homing; 1=MAX, -1=MIN
// :[-1,1]
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// @section machine

// The size of the print bed
#define X_BED_SIZE 220
#define Y_BED_SIZE 220

// Travel limits (mm) after homing, corresponding to endstop positions.
#define X_MIN_POS -7.5
#define Y_MIN_POS -7.5
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 245

// Min software endstops constrain movement within minimum coordinate bounds
#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
  #define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  #define MIN_SOFTWARE_ENDSTOP_Z
#endif

// Max software endstops constrain movement within maximum coordinate bounds
#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
  #define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z
#endif

#if ENABLED(MIN_SOFTWARE_ENDSTOPS) || ENABLED(MAX_SOFTWARE_ENDSTOPS)
  //#define SOFT_ENDSTOPS_MENU_ITEM  // Enable/Disable software endstops from the LCD
#endif

//===========================================================================
//=============================== Bed Leveling ==============================
//===========================================================================
// @section calibrate

#define RESTORE_LEVELING_AFTER_G28
#define AUTO_BED_LEVELING_BILINEAR

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  // Gradually reduce leveling correction until a set height is reached,
  // at which point movement will be level to the machine's XY plane.
  // The height can be set with M420 Z<height>
  #define ENABLE_LEVELING_FADE_HEIGHT

  // For Cartesian machines, instead of dividing moves on mesh boundaries,
  // split up moves into short segments like a Delta. This follows the
  // contours of the bed more closely than edge-to-edge straight moves.
  #define SEGMENT_LEVELED_MOVES
  #define LEVELED_SEGMENT_LENGTH 5.0 // (mm) Length of all segments (except the last one)

  // Set the number of grid points per dimension.
  #define GRID_MAX_POINTS_X 3
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  // Set the boundaries for probing (where the probe can reach).
  //#define LEFT_PROBE_BED_POSITION MIN_PROBE_EDGE
  //#define RIGHT_PROBE_BED_POSITION (X_BED_SIZE - MIN_PROBE_EDGE)
  //#define FRONT_PROBE_BED_POSITION MIN_PROBE_EDGE
  //#define BACK_PROBE_BED_POSITION (Y_BED_SIZE - MIN_PROBE_EDGE)

#endif // BED_LEVELING

// @section homing
#define Z_SAFE_HOMING
#if ENABLED(Z_SAFE_HOMING)
  #define Z_SAFE_HOMING_X_POINT ((X_BED_SIZE) / 2)    // X point for Z homing when homing all axes (G28).
  #define Z_SAFE_HOMING_Y_POINT ((Y_BED_SIZE) / 2)    // Y point for Z homing when homing all axes (G28).
#endif

// Homing speeds (mm/m)
#define HOMING_FEEDRATE_XY (20*60)
#define HOMING_FEEDRATE_Z  (4*60)

//=============================================================================
//============================= Additional Features ===========================
//=============================================================================

// @section extras

#define EEPROM_SETTINGS // Enable for M500 and M501 commands
//#define DISABLE_M503    // Saves ~2700 bytes of PROGMEM. Disable for release!
#define EEPROM_CHITCHAT   // Give feedback on EEPROM commands. Disable to save PROGMEM.

#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages
#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.
#define BUSY_WHILE_HEATING            // Some hosts require "busy" messages even during heating

// Preheat Constants
#define PREHEAT_1_TEMP_HOTEND 185
#define PREHEAT_1_TEMP_BED     55
#define PREHEAT_1_FAN_SPEED   255 // Value from 0 to 255

#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED     55
#define PREHEAT_2_FAN_SPEED   255 // Value from 0 to 255

#define PRINTJOB_TIMER_AUTOSTART

//=============================================================================
//============================= LCD and SD support ============================
//=============================================================================

// @section lcd

#define LCD_LANGUAGE en
#define DISPLAY_CHARSET_HD44780 WESTERN
#define SDSUPPORT
//#define NO_LCD_MENUS
#define SLIM_LCD_MENUS

//=============================================================================
//=======================   LCD / Controller Selection  =======================
//=========================      (Graphical LCDs)      ========================
//=============================================================================

// Factory display for Creality CR-10
// https://www.aliexpress.com/item/Universal-LCD-12864-3D-Printer-Display-Screen-With-Encoder-For-CR-10-CR-7-Model/32833148327.html
//
// This is RAMPS-compatible using a single 10-pin connector.
// (For CR-10 owners who want to replace the Melzi Creality board but retain the display)
//
#define CR10_STOCKDISPLAY

//=============================================================================
//=============================== Extra Features ==============================
//=============================================================================

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// Delay (in milliseconds) before the next move will start, to give the servo time to reach its target angle.
// 300ms is a good value but you can try less delay.
// If the servo can't reach the requested position, increase it.
#define SERVO_DELAY { 300 }

// Only power servos during movement, otherwise leave off to prevent jitter
//#define DEACTIVATE_SERVOS_AFTER_MOVE

#endif // CONFIGURATION_H

//===========================================================================
//===========================================================================
//===========================================================================
//=========================== END OF FILE ===================================
//===========================================================================
//===========================================================================
//===========================================================================
