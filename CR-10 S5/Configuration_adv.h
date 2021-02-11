/**
 *                     Configuration_adv.h
 * 
 * CR-10s S5 with BTT SKR V1.4 Turbo, 5x TMC 2208 v3.0, BL Touch v3.1,
 * Keenovo Heat Bed, Micro-Swiss Direct Drive and HotEnd, NeoPixel LEDS.
 *
 */

#pragma once
#define CONFIGURATION_ADV_H_VERSION 020008

// Up To 25 could be defined, model dependent.
//#define CUSTOM_USER_MENUS
#if ENABLED(CUSTOM_USER_MENUS)
  //#define CUSTOM_USER_MENU_TITLE "Custom Commands"
  #define USER_SCRIPT_DONE "M117 User Script Done"
  #define USER_SCRIPT_AUDIBLE_FEEDBACK
  //#define USER_SCRIPT_RETURN  // Return to status screen after a script

  #define USER_DESC_1 "Home & UBL Info"
  #define USER_GCODE_1 "G28\nG29 W"

  #define USER_DESC_2 "Preheat for " PREHEAT_1_LABEL
  #define USER_GCODE_2 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)

  #define USER_DESC_3 "Preheat for " PREHEAT_2_LABEL
  #define USER_GCODE_3 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)

  #define USER_DESC_4 "Heat Bed/Home/Level"
  #define USER_GCODE_4 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"

  #define USER_DESC_5 "Home & Info"
  #define USER_GCODE_5 "G28\nM503"
#endif


//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
#define THERMOCOUPLE_MAX_ERRORS 15

#if TEMP_SENSOR_0 == 1000
  #define HOTEND0_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor
  #define HOTEND0_RESISTANCE_25C_OHMS  100000  // Resistance at 25C
  #define HOTEND0_BETA                 3950    // Beta value
#endif

#if TEMP_SENSOR_1 == 1000
  #define HOTEND1_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor
  #define HOTEND1_RESISTANCE_25C_OHMS  100000  // Resistance at 25C
  #define HOTEND1_BETA                 3950    // Beta value
#endif

#if TEMP_SENSOR_BED == 1000
  #define BED_PULLUP_RESISTOR_OHMS     4700    // Pullup resistor
  #define BED_RESISTANCE_25C_OHMS      100000  // Resistance at 25C
  #define BED_BETA                     3950    // Beta value
#endif

//
// Heated Bed Bang-Bang options
//
#if DISABLED(PIDTEMPBED)
  #define BED_CHECK_INTERVAL 5000   // (ms) Interval between checks in bang-bang control
  #if ENABLED(BED_LIMIT_SWITCHING)
    #define BED_HYSTERESIS 2        // (°C) Only set the relevant heater state when ABS(T-target) > BED_HYSTERESIS
  #endif
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  #define THERMAL_PROTECTION_PERIOD 40        // Seconds
  #define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

  //#define ADAPTIVE_FAN_SLOWING              // Slow part cooling fan if temperature drops
  #if BOTH(ADAPTIVE_FAN_SLOWING, PIDTEMP)
    //#define NO_FAN_SLOWING_IN_PID_TUNING    // Don't slow fan speed during M303
  #endif

  #define WATCH_TEMP_PERIOD  40               // Seconds
  #define WATCH_TEMP_INCREASE 2               // Degrees Celsius
#endif

#if ENABLED(THERMAL_PROTECTION_BED)
  #define THERMAL_PROTECTION_BED_PERIOD        40 // Seconds
  #define THERMAL_PROTECTION_BED_HYSTERESIS     4 // Degrees Celsius

  /**
   * As described above, except for the bed (M140/M190/M303).
   */
  #define WATCH_BED_TEMP_PERIOD               600 // Seconds
  #define WATCH_BED_TEMP_INCREASE               2 // Degrees Celsius
#endif

#if ENABLED(PIDTEMP)
  // Add an experimental additional term to the heater power, proportional to the extrusion speed.
  // A well-chosen Kc value should add just enough power to melt the increased material volume.
  //#define PID_EXTRUSION_SCALING
  #if ENABLED(PID_EXTRUSION_SCALING)
    #define DEFAULT_Kc (100) // heating power = Kc * e_speed
    #define LPQ_MAX_LEN 50
  #endif

  //#define PID_FAN_SCALING
  #if ENABLED(PID_FAN_SCALING)
    //#define PID_FAN_SCALING_ALTERNATIVE_DEFINITION
    #if ENABLED(PID_FAN_SCALING_ALTERNATIVE_DEFINITION)
      // The alternative definition is used for an easier configuration.
      // Just figure out Kf at fullspeed (255) and PID_FAN_SCALING_MIN_SPEED.
      // DEFAULT_Kf and PID_FAN_SCALING_LIN_FACTOR are calculated accordingly.

      #define PID_FAN_SCALING_AT_FULL_SPEED 13.0        //=PID_FAN_SCALING_LIN_FACTOR*255+DEFAULT_Kf
      #define PID_FAN_SCALING_AT_MIN_SPEED   6.0        //=PID_FAN_SCALING_LIN_FACTOR*PID_FAN_SCALING_MIN_SPEED+DEFAULT_Kf
      #define PID_FAN_SCALING_MIN_SPEED     10.0        // Minimum fan speed at which to enable PID_FAN_SCALING

      #define DEFAULT_Kf (255.0*PID_FAN_SCALING_AT_MIN_SPEED-PID_FAN_SCALING_AT_FULL_SPEED*PID_FAN_SCALING_MIN_SPEED)/(255.0-PID_FAN_SCALING_MIN_SPEED)
      #define PID_FAN_SCALING_LIN_FACTOR (PID_FAN_SCALING_AT_FULL_SPEED-DEFAULT_Kf)/255.0

    #else
      #define PID_FAN_SCALING_LIN_FACTOR (0)             // Power loss due to cooling = Kf * (fan_speed)
      #define DEFAULT_Kf 10                              // A constant value added to the PID-tuner
      #define PID_FAN_SCALING_MIN_SPEED 10               // Minimum fan speed at which to enable PID_FAN_SCALING
    #endif
  #endif
#endif

#define AUTOTEMP
#if ENABLED(AUTOTEMP)
  #define AUTOTEMP_OLDWEIGHT    0.98
  // Turn on AUTOTEMP on M104/M109 by default using proportions set here
  //#define AUTOTEMP_PROPORTIONAL
  #if ENABLED(AUTOTEMP_PROPORTIONAL)
    #define AUTOTEMP_MIN_P      0 // (°C) Added to the target temperature
    #define AUTOTEMP_MAX_P      5 // (°C) Added to the target temperature
    #define AUTOTEMP_FACTOR_P   1 // Apply this F parameter by default (overridden by M104/M109 F)
  #endif
#endif

/**
 * Hotend Idle Timeout
 * Prevent filament in the nozzle from charring and causing a critical jam.
 */
//#define HOTEND_IDLE_TIMEOUT
#if ENABLED(HOTEND_IDLE_TIMEOUT)
  #define HOTEND_IDLE_TIMEOUT_SEC (5*60)    // (seconds) Time without extruder movement to trigger protection
  #define HOTEND_IDLE_MIN_TRIGGER   180     // (°C) Minimum temperature to enable hotend protection
  #define HOTEND_IDLE_NOZZLE_TARGET   0     // (°C) Safe temperature for the nozzle after timeout
  #define HOTEND_IDLE_BED_TARGET      0     // (°C) Safe temperature for the bed after timeout
#endif

// Calibration for AD595 / AD8495 sensor to adjust temperature measurements.
// The final temperature is calculated as (measuredTemp * GAIN) + OFFSET.
#define TEMP_SENSOR_AD595_OFFSET  0.0
#define TEMP_SENSOR_AD595_GAIN    1.0
#define TEMP_SENSOR_AD8495_OFFSET 0.0
#define TEMP_SENSOR_AD8495_GAIN   1.0

// EXTRUDER COOLING FAN PIN SETTINGS. -1 For Off
#define E0_AUTO_FAN_PIN -1
#define E1_AUTO_FAN_PIN -1
#define E2_AUTO_FAN_PIN -1
#define E3_AUTO_FAN_PIN -1
#define E4_AUTO_FAN_PIN -1
#define E5_AUTO_FAN_PIN -1
#define E6_AUTO_FAN_PIN -1
#define E7_AUTO_FAN_PIN -1
#define CHAMBER_AUTO_FAN_PIN -1

#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED 255   // 255 == full speed
#define CHAMBER_AUTO_FAN_TEMPERATURE 30
#define CHAMBER_AUTO_FAN_SPEED 255

// Part-Cooling Fan Multiplexer
#define FANMUX0_PIN -1
#define FANMUX1_PIN -1
#define FANMUX2_PIN -1

//
// For Z set the number of stepper drivers
//
#define NUM_Z_STEPPER_DRIVERS 2   // (1-4) Z options change based on how many

#if NUM_Z_STEPPER_DRIVERS > 1
  // Enable if Z motor direction signals are the opposite of Z1
  //#define INVERT_Z2_VS_Z_DIR
  //#define INVERT_Z3_VS_Z_DIR
  //#define INVERT_Z4_VS_Z_DIR

  //#define Z_MULTI_ENDSTOPS
  #if ENABLED(Z_MULTI_ENDSTOPS)
    #define Z2_USE_ENDSTOP          _XMAX_
    #define Z2_ENDSTOP_ADJUSTMENT   0
    #if NUM_Z_STEPPER_DRIVERS >= 3
      #define Z3_USE_ENDSTOP        _YMAX_
      #define Z3_ENDSTOP_ADJUSTMENT 0
    #endif
    #if NUM_Z_STEPPER_DRIVERS >= 4
      #define Z4_USE_ENDSTOP        _ZMAX_
      #define Z4_ENDSTOP_ADJUSTMENT 0
    #endif
  #endif
#endif

//#define SENSORLESS_BACKOFF_MM  { 2, 2 }     // (mm) Backoff from endstops before sensorless homing

#define HOMING_BUMP_MM      { 5, 5, 2 }       // (mm) Backoff from endstops after first bump
#define HOMING_BUMP_DIVISOR { 2, 2, 4 }       // Re-Bump Speed Divisor (Divides the Homing Feedrate)

//#define HOMING_BACKOFF_POST_MM { 2, 2, 2 }  // (mm) Backoff from endstops after homing

#define QUICK_HOME                            // If G28 contains XY do a diagonal move first
//#define HOME_Y_BEFORE_X                     // If G28 contains XY home Y before X
//#define HOME_Z_FIRST                        // Home Z first. Requires a Z-MIN endstop (not a probe).
//#define CODEPENDENT_XY_HOMING               // If X/Y can't home without homing Y/X first

#if ENABLED(BLTOUCH)
  
  //#define BLTOUCH_DELAY 500
  #define BLTOUCH_FORCE_SW_MODE
  #define BLTOUCH_SET_5V_MODE
  //#define BLTOUCH_FORCE_MODE_SET
  //#define BLTOUCH_HS_MODE
  //#define BLTOUCH_LCD_VOLTAGE_MENU

#endif // BLTOUCH

// Z Steppers Auto-Alignment
#define Z_STEPPER_AUTO_ALIGN
#if ENABLED(Z_STEPPER_AUTO_ALIGN)
  // Define probe X and Y positions for Z1, Z2 [, Z3 [, Z4]]
  // If not defined, probe limits will be used.
  // Override with 'M422 S<index> X<pos> Y<pos>'
  //#define Z_STEPPER_ALIGN_XY { {  10, 190 }, { 100,  10 }, { 190, 190 } }
  /**
   * Orientation for the automatically-calculated probe positions.
   * Override Z stepper align points with 'M422 S<index> X<pos> Y<pos>'
   *
   * 2 Steppers:  (0)     (1)
   *               |       |   2   |
   *               | 1   2 |       |
   *               |       |   1   |
   */
  #ifndef Z_STEPPER_ALIGN_XY
    //#define Z_STEPPERS_ORIENTATION 0
  #endif

  // Provide Z stepper positions for more rapid convergence in bed alignment.
  // Requires triple stepper drivers (i.e., set NUM_Z_STEPPER_DRIVERS to 3)
  //#define Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS
  #if ENABLED(Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS)
    // Define Stepper XY positions for Z1, Z2, Z3 corresponding to
    // the Z screw positions in the bed carriage.
    // Define one position per Z stepper in stepper driver order.
    #define Z_STEPPER_ALIGN_STEPPER_XY { { 210.7, 102.5 }, { 152.6, 220.0 }, { 94.5, 102.5 } }
  #else
    // Amplification factor. Used to scale the correction step up or down in case
    // the stepper (spindle) position is farther out than the test point.
    #define Z_STEPPER_ALIGN_AMP 1.0       // Use a value > 1.0 NOTE: This may cause instability!
  #endif

  // On a 300mm bed a 5% grade would give a misalignment of ~1.5cm
  #define G34_MAX_GRADE              4    // (%) Maximum incline that G34 will handle
  #define Z_STEPPER_ALIGN_ITERATIONS 3    // Number of iterations to apply during alignment
  #define Z_STEPPER_ALIGN_ACC        0.02 // Stop iterating early if the accuracy is better than this
  #define RESTORE_LEVELING_AFTER_G34      // Restore leveling after G34 is done?
  // After G34, re-home Z (G28 Z) or just calculate it from the last probe heights?
  // Re-homing might be more precise in reproducing the actual 'G28 Z' homing height, especially on an uneven bed.
  #define HOME_AFTER_G34
#endif

#define AXIS_RELATIVE_MODES { false, false, false, false }

// Add a Duplicate option for well-separated conjoined nozzles
//#define MULTI_NOZZLE_DUPLICATION

// By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

/**
 * Idle Stepper Shutdown
 * Set DISABLE_INACTIVE_? 'true' to shut down axis steppers after an idle period.
 * The Deactive Time can be overridden with M18 and M84. Set to 0 for No Timeout.
 */
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X true
#define DISABLE_INACTIVE_Y true
#define DISABLE_INACTIVE_Z true  // Set 'false' if the nozzle could fall onto your printed part!
#define DISABLE_INACTIVE_E true

// If the Nozzle or Bed falls when the Z stepper is disabled, set its resting position here.
//#define Z_AFTER_DEACTIVATE Z_HOME_POS

// Default Minimum Feedrates for printing and travel moves
#define DEFAULT_MINIMUMFEEDRATE       0.0     // (mm/s) Minimum feedrate. Set with M205 S.
#define DEFAULT_MINTRAVELFEEDRATE     0.0     // (mm/s) Minimum travel feedrate. Set with M205 T.

// Minimum time that a segment needs to take as the buffer gets emptied
#define DEFAULT_MINSEGMENTTIME        20000   // (µs) Set with M205 B.

// Slow down the machine if the lookahead buffer is (by default) half full.
// Increase the slowdown divisor for larger buffer sizes.
#define SLOWDOWN
#if ENABLED(SLOWDOWN)
  #define SLOWDOWN_DIVISOR 2
#endif

/**
 * XY Frequency limit
 * Reduce resonance by limiting the frequency of small zigzag infill moves.
 * See https://hydraraptor.blogspot.com/2010/12/frequency-limit.html
 * Use M201 F<freq> G<min%> to change limits at runtime.
 */
//#define XY_FREQUENCY_LIMIT      10 // (Hz) Maximum frequency of small zigzag infill moves. Set with M201 F<hertz>.
#ifdef XY_FREQUENCY_LIMIT
  #define XY_FREQUENCY_MIN_PERCENT 5 // (percent) Minimum FR percentage to apply. Set with M201 G<min%>.
#endif

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/s)

/**
 * Adaptive Step Smoothing increases the resolution of multi-axis moves, particularly at step frequencies
 * below 1kHz (for AVR) or 10kHz (for ARM), where aliasing between axes in multi-axis moves causes audible
 * vibration and surface artifacts. The algorithm adapts to provide the best possible step smoothing at the
 * lowest stepping frequencies.
 */
#define ADAPTIVE_STEP_SMOOTHING

// Microstep settings (Requires a board with pins named X_MS1, X_MS2, etc.)
//#define MICROSTEP_MODES { 16, 16, 16, 16, 16, 16 } // [1,2,4,8,16]

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#if EITHER(IS_ULTIPANEL, EXTENSIBLE_UI)
  #define MANUAL_FEEDRATE { 50*60, 50*60, 4*60, 2*60 } // (mm/min) Feedrates for manual moves along X, Y, Z, E from panel
  #define FINE_MANUAL_MOVE 0.025    // (mm) Smallest manual move (< 0.1mm) applying to Z on most machines
  #if IS_ULTIPANEL
    #define MANUAL_E_MOVES_RELATIVE // Display extruder move distance rather than "position"
    #define ULTIPANEL_FEEDMULTIPLY  // Encoder sets the feedrate multiplier on the Status Screen
  #endif
#endif
// Change values more rapidly when the encoder is rotated faster
#define ENCODER_RATE_MULTIPLIER
#if ENABLED(ENCODER_RATE_MULTIPLIER)
  #define ENCODER_10X_STEPS_PER_SEC   30  // (steps/s) Encoder rate for 10x speed
  #define ENCODER_100X_STEPS_PER_SEC  80  // (steps/s) Encoder rate for 100x speed
#endif
// Play a beep when the feedrate is changed from the Status Screen
//#define BEEP_ON_FEEDRATE_CHANGE
#if ENABLED(BEEP_ON_FEEDRATE_CHANGE)
  #define FEEDRATE_CHANGE_BEEP_DURATION   10
  #define FEEDRATE_CHANGE_BEEP_FREQUENCY 440
#endif
#if HAS_LCD_MENU

  // Add Probe Z Offset calibration to the Z Probe Offsets menu
  #if HAS_BED_PROBE
    //#define PROBE_OFFSET_WIZARD
    #if ENABLED(PROBE_OFFSET_WIZARD)
      //
      // Enable to init the Probe Z-Offset when starting the Wizard.
      // Use a height slightly above the estimated nozzle-to-probe Z offset.
      // For example, with an offset of -5, consider a starting height of -4.
      //
      //#define PROBE_OFFSET_WIZARD_START_Z -4.0

      // Set a convenient position to do the calibration (probing point and nozzle/bed-distance)
      //#define PROBE_OFFSET_WIZARD_XY_POS { X_CENTER, Y_CENTER }
    #endif
  #endif

  // Include a page of printer information in the LCD Main Menu
  #define LCD_INFO_MENU
  #if ENABLED(LCD_INFO_MENU)
    //#define LCD_PRINTER_INFO_IS_BOOTSCREEN // Show bootscreen(s) instead of Printer Info pages
  #endif

  // BACK menu items keep the highlight at the top
  //#define TURBO_BACK_MENU_ITEM

  // Add a mute option to the LCD menu
  //#define SOUND_MENU_ITEM

  /**
   * LED Control Menu
   * Add LED Control to the LCD menu
   */
  #define LED_CONTROL_MENU
  #if ENABLED(LED_CONTROL_MENU)
    #define LED_COLOR_PRESETS                 // Enable the Preset Color menu option
    //#define NEO2_COLOR_PRESETS              // Enable a second NeoPixel Preset Color menu option
    #if ENABLED(LED_COLOR_PRESETS)
      #define LED_USER_PRESET_RED        255  // User defined RED value
      #define LED_USER_PRESET_GREEN      128  // User defined GREEN value
      #define LED_USER_PRESET_BLUE         0  // User defined BLUE value
      #define LED_USER_PRESET_WHITE      255  // User defined WHITE value
      #define LED_USER_PRESET_BRIGHTNESS 255  // User defined intensity
      //#define LED_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup
    #endif
    #if ENABLED(NEO2_COLOR_PRESETS)
      #define NEO2_USER_PRESET_RED        255  // User defined RED value
      #define NEO2_USER_PRESET_GREEN      128  // User defined GREEN value
      #define NEO2_USER_PRESET_BLUE         0  // User defined BLUE value
      #define NEO2_USER_PRESET_WHITE      255  // User defined WHITE value
      #define NEO2_USER_PRESET_BRIGHTNESS 255  // User defined intensity
      //#define NEO2_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup for the second strip
    #endif
  #endif

#endif // HAS_LCD_MENU

#if HAS_DISPLAY
  // The timeout (in ms) to return to the status screen from sub-menus
  //#define LCD_TIMEOUT_TO_STATUS 15000

  #if ENABLED(SHOW_BOOTSCREEN)
    #define BOOTSCREEN_TIMEOUT 4000      // (ms) Total Duration to display the boot screen(s)
    #if EITHER(HAS_MARLINUI_U8GLIB, TFT_COLOR_UI)
      #define BOOT_MARLIN_LOGO_SMALL     // Show a smaller Marlin logo on the Boot Screen (saving lots of flash)
    #endif
  #endif

  // Scroll a longer status message into view
  #define STATUS_MESSAGE_SCROLLING

  // On the Info Screen, display XY with one decimal place when possible
  //#define LCD_DECIMAL_SMALL_XY

  // Add an 'M73' G-code to set the current percentage
  //#define LCD_SET_PROGRESS_MANUALLY

  // Show the E position (filament used) during printing
  //#define LCD_SHOW_E_TOTAL
#endif

#if EITHER(SDSUPPORT, LCD_SET_PROGRESS_MANUALLY) && ANY(HAS_MARLINUI_U8GLIB, HAS_MARLINUI_HD44780, IS_TFTGLCD_PANEL, EXTENSIBLE_UI)
  //#define SHOW_REMAINING_TIME       // Display estimated time to completion
  #if ENABLED(SHOW_REMAINING_TIME)
    //#define USE_M73_REMAINING_TIME  // Use remaining time from M73 command instead of estimation
    //#define ROTATE_PROGRESS_DISPLAY // Display (P)rogress, (E)lapsed, and (R)emaining time
  #endif

  #if EITHER(HAS_MARLINUI_U8GLIB, EXTENSIBLE_UI)
    //#define PRINT_PROGRESS_SHOW_DECIMALS // Show progress with decimal digits
  #endif

  #if EITHER(HAS_MARLINUI_HD44780, IS_TFTGLCD_PANEL)
    //#define LCD_PROGRESS_BAR            // Show a progress bar on HD44780 LCDs for SD printing
    #if ENABLED(LCD_PROGRESS_BAR)
      #define PROGRESS_BAR_BAR_TIME 2000  // (ms) Amount of time to show the bar
      #define PROGRESS_BAR_MSG_TIME 3000  // (ms) Amount of time to show the status message
      #define PROGRESS_MSG_EXPIRE   0     // (ms) Amount of time to retain the status message (0=forever)
      //#define PROGRESS_MSG_ONCE         // Show the message for MSG_TIME then clear it
      //#define LCD_PROGRESS_BAR_TEST     // Add a menu item to test the progress bar
    #endif
  #endif
#endif

#if ENABLED(SDSUPPORT)
  
  #define SD_PROCEDURE_DEPTH 1              // Increase if you need more nested M32 calls

  #define SD_FINISHED_STEPPERRELEASE true   // Disable steppers when SD Print is finished
  #define SD_FINISHED_RELEASECOMMAND "M84"  // Use "M84XYE" to keep Z enabled so your bed stays in place

  // Reverse SD sort to show "more recent" files first, according to the card's FAT.
  // Since the FAT gets out of order with usage, SDCARD_SORT_ALPHA is recommended.
  #define SDCARD_RATHERRECENTFIRST

  #define SD_MENU_CONFIRM_START             // Confirm the selected SD file before printing

  //#define NO_SD_AUTOSTART                 // Remove auto#.g file support completely to save some Flash, SRAM
  //#define MENU_ADDAUTOSTART               // Add a menu option to run auto#.g files

  //#define BROWSE_MEDIA_ON_INSERT          // Open the file browser when media is inserted

  #define EVENT_GCODE_SD_ABORT "G27"        // G-code to run on SD Abort Print (e.g., "G28XY" or "G27")

  #if ENABLED(PRINTER_EVENT_LEDS)
    #define PE_LEDS_COMPLETED_TIME  (30*60) // (seconds) Time to keep the LED "done" color before restoring normal illumination
  #endif

  // Continue after Power-Loss (Creality3D)
  #define POWER_LOSS_RECOVERY
  #if ENABLED(POWER_LOSS_RECOVERY)
    #define PLR_ENABLED_DEFAULT   false // Power Loss Recovery enabled by default. (Set with 'M413 Sn' & M500)
    //#define BACKUP_POWER_SUPPLY       // Backup power / UPS to move the steppers on power loss
    //#define POWER_LOSS_RECOVER_ZHOME  // Z homing is needed for proper recovery. 99.9% of the time this should be disabled!
    //#define POWER_LOSS_ZRAISE       2 // (mm) Z axis raise on resume (on power loss with UPS)
    //#define POWER_LOSS_PIN         44 // Pin to detect power loss. Set to -1 to disable default pin on boards without module.
    //#define POWER_LOSS_STATE     HIGH // State of pin indicating power loss
    //#define POWER_LOSS_PULLUP         // Set pullup / pulldown as appropriate for your sensor
    //#define POWER_LOSS_PULLDOWN
    //#define POWER_LOSS_PURGE_LEN   20 // (mm) Length of filament to purge on resume
    //#define POWER_LOSS_RETRACT_LEN 10 // (mm) Length of filament to retract on fail. Requires backup power.

    // Without a POWER_LOSS_PIN the following option helps reduce wear on the SD card,
    // especially with "vase mode" printing. Set too high and vases cannot be continued.
    #define POWER_LOSS_MIN_Z_CHANGE 0.05 // (mm) Minimum Z change before saving power-loss data
  #endif

  #define SDCARD_SORT_ALPHA

  // SD Card Sorting options
  #if ENABLED(SDCARD_SORT_ALPHA)
    #define SDSORT_LIMIT       40     // Maximum number of sorted items (10-256). Costs 27 bytes each.
    #define FOLDER_SORTING     -1     // -1=above  0=none  1=below
    #define SDSORT_GCODE       false  // Allow turning sorting on/off with LCD and M34 G-code.
    #define SDSORT_USES_RAM    true   // Pre-allocate a static array for faster pre-sorting.
    #define SDSORT_USES_STACK  true   // Prefer the stack for pre-sorting to give back some SRAM. (Negated by next 2 options.)
    #define SDSORT_CACHE_NAMES false  // Keep sorted items in RAM longer for speedy performance. Most expensive option.
    #define SDSORT_DYNAMIC_RAM false  // Use dynamic allocation (within SD menus). Least expensive option. Set SDSORT_LIMIT before use!
    #define SDSORT_CACHE_VFATS 2      // Maximum number of 13-byte VFAT entries to use for sorting.
                                      // Note: Only affects SCROLL_LONG_FILENAMES with SDSORT_CACHE_NAMES but not SDSORT_DYNAMIC_RAM.
  #endif

  // Allow international symbols in long filenames. To display correctly, the
  // LCD's font must contain the characters. Check your selected LCD language.
  //#define UTF_FILENAME_SUPPORT

  // This allows hosts to request long names for files and folders with M33
  //#define LONG_FILENAME_HOST_SUPPORT

  // Enable this option to scroll long filenames in the SD card menu
  #define SCROLL_LONG_FILENAMES

#endif // SDSUPPORT

#if HAS_MARLINUI_U8GLIB
  // Show SD percentage next to the progress bar
  #define DOGM_SD_PERCENT

  // Save many cycles by drawing a hollow frame or no frame on the Info Screen
  //#define XYZ_NO_FRAME
  #define XYZ_HOLLOW_FRAME

  // Enable to save many cycles by drawing a hollow frame on Menu Screens
  #define MENU_HOLLOW_FRAME

  // A bigger font is available for edit items. Costs 3120 bytes of PROGMEM.
  // Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.
  //#define USE_BIG_EDIT_FONT

  // A smaller font may be used on the Info Screen. Costs 2434 bytes of PROGMEM.
  // Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.
  //#define USE_SMALL_INFOFONT

  // Swap the CW/CCW indicators in the graphics overlay
  //#define OVERLAY_GFX_REVERSE

  #if ENABLED(U8GLIB_ST7920)
    // Enable this option and reduce the value to optimize screen updates.
    // The normal delay is 10µs. Use the lowest value that still gives a reliable display.
    //#define DOGM_SPI_DELAY_US 5

    //#define LIGHTWEIGHT_UI
    #if ENABLED(LIGHTWEIGHT_UI)
      #define STATUS_EXPIRE_SECONDS 20
    #endif
  #endif

  /**
   * Status (Info) Screen customizations
   * These options may affect code size and screen render time.
   * Custom status screens can forcibly override these settings.
   */
  //#define STATUS_COMBINE_HEATERS    // Use combined heater images instead of separate ones
  //#define STATUS_HOTEND_NUMBERLESS  // Use plain hotend icons instead of numbered ones (with 2+ hotends)
  #define STATUS_HOTEND_INVERTED      // Show solid nozzle bitmaps when heating (Requires STATUS_HOTEND_ANIM)
  #define STATUS_HOTEND_ANIM          // Use a second bitmap to indicate hotend heating
  #define STATUS_BED_ANIM             // Use a second bitmap to indicate bed heating
  #define STATUS_CHAMBER_ANIM         // Use a second bitmap to indicate chamber heating
  //#define STATUS_CUTTER_ANIM        // Use a second bitmap to indicate spindle / laser active
  //#define STATUS_ALT_BED_BITMAP     // Use the alternative bed bitmap
  //#define STATUS_ALT_FAN_BITMAP     // Use the alternative fan bitmap
  //#define STATUS_FAN_FRAMES 3       // :[0,1,2,3,4] Number of fan animation frames
  //#define STATUS_HEAT_PERCENT       // Show heating in a progress bar
  //#define BOOT_MARLIN_LOGO_ANIMATED // Animated Marlin logo. Costs ~‭3260 (or ~940) bytes of PROGMEM.

#endif // HAS_MARLINUI_U8GLIB

//
// Specify additional languages for the UI. Default specified by LCD_LANGUAGE.
//
#if ANY(DOGLCD, TFT_COLOR_UI, TOUCH_UI_FTDI_EVE)
  //#define LCD_LANGUAGE_2 fr
  //#define LCD_LANGUAGE_3 de
  //#define LCD_LANGUAGE_4 es
  //#define LCD_LANGUAGE_5 it
  #ifdef LCD_LANGUAGE_2
    //#define LCD_LANGUAGE_AUTO_SAVE // Automatically save language to EEPROM on change
  #endif
#endif

//
// Classic UI Options
//
#if TFT_SCALED_DOGLCD
  //#define TFT_MARLINUI_COLOR 0xFFFF // White
  //#define TFT_MARLINBG_COLOR 0x0000 // Black
  //#define TFT_DISABLED_COLOR 0x0003 // Almost black
  //#define TFT_BTCANCEL_COLOR 0xF800 // Red
  //#define TFT_BTARROWS_COLOR 0xDEE6 // 11011 110111 00110 Yellow
  //#define TFT_BTOKMENU_COLOR 0x145F // 00010 100010 11111 Cyan
#endif

//
// ADC Button Debounce
//
#if HAS_ADC_BUTTONS
  #define ADC_BUTTON_DEBOUNCE_DELAY 16  // Increase if buttons bounce or repeat too fast
#endif

// @section safety

#define USE_WATCHDOG
#if ENABLED(USE_WATCHDOG)
  //#define WATCHDOG_RESET_MANUAL
#endif

#define BABYSTEPPING
#if ENABLED(BABYSTEPPING)
  //#define INTEGRATED_BABYSTEPPING         // EXPERIMENTAL integration of babystepping into the Stepper ISR
  //#define BABYSTEP_WITHOUT_HOMING
  //#define BABYSTEP_ALWAYS_AVAILABLE       // Allow babystepping at all times (not just during movement).
  //#define BABYSTEP_XY                     // Also enable X/Y Babystepping. Not supported on DELTA!
  #define BABYSTEP_INVERT_Z false           // Change if Z babysteps should go the other way
  //#define BABYSTEP_MILLIMETER_UNITS       // Specify BABYSTEP_MULTIPLICATOR_(XY|Z) in mm instead of micro-steps
  #define BABYSTEP_MULTIPLICATOR_Z  1       // (steps or mm) Steps or millimeter distance for each Z babystep
  #define BABYSTEP_MULTIPLICATOR_XY 10      // (steps or mm) Steps or millimeter distance for each Z babystep

  #define DOUBLECLICK_FOR_Z_BABYSTEPPING    // Double-click on the Status Screen for Z Babystepping.
  #if ENABLED(DOUBLECLICK_FOR_Z_BABYSTEPPING)
    #define DOUBLECLICK_MAX_INTERVAL 1250   // Maximum interval between clicks, in milliseconds.
                                            // Note: Extra time may be added to mitigate controller latency.
    //#define MOVE_Z_WHEN_IDLE              // Jump to the move Z menu on doubleclick when printer is idle.
    #if ENABLED(MOVE_Z_WHEN_IDLE)
      #define MOVE_Z_IDLE_MULTIPLICATOR 1   // Multiply 1mm by this factor for the move step size.
    #endif
  #endif

  //#define BABYSTEP_DISPLAY_TOTAL          // Display total babysteps since last G28

  //#define BABYSTEP_ZPROBE_OFFSET          // Combine M851 Z and Babystepping
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    //#define BABYSTEP_HOTEND_Z_OFFSET      // For multiple hotends, babystep relative Z offsets
    //#define BABYSTEP_ZPROBE_GFX_OVERLAY   // Enable graphical overlay on Z-offset editor
  #endif
#endif

// @section extruder

// Linear Pressure Control v1.5
#define LIN_ADVANCE
#if ENABLED(LIN_ADVANCE)
  //#define EXTRA_LIN_ADVANCE_K // Enable for second linear advance constants
  #define LIN_ADVANCE_K 0.22    // Unit: mm compression per 1mm/s extruder speed
  //#define LA_DEBUG            // If enabled, this will generate debug information output over USB.
  #define EXPERIMENTAL_SCURVE // Enable this option to permit S-Curve Acceleration
#endif

/**
 * Points to probe for all 3-point Leveling procedures.
 * Override if the automatically selected points are inadequate.
 */
#if EITHER(AUTO_BED_LEVELING_3POINT, AUTO_BED_LEVELING_UBL)
  //#define PROBE_PT_1_X 15
  //#define PROBE_PT_1_Y 180
  //#define PROBE_PT_2_X 15
  //#define PROBE_PT_2_Y 20
  //#define PROBE_PT_3_X 170
  //#define PROBE_PT_3_Y 20
#endif

/**
 * Probing Margins
 *
 * Override PROBING_MARGIN for each side of the build plate
 * Useful to get probe points to exact positions on targets or
 * to allow leveling to avoid plate clamps on only specific
 * sides of the bed. With NOZZLE_AS_PROBE negative values are
 * allowed, to permit probing outside the bed.
 *
 * If you are replacing the prior *_PROBE_BED_POSITION options,
 * LEFT and FRONT values in most cases will map directly over
 * RIGHT and REAR would be the inverse such as
 * (X/Y_BED_SIZE - RIGHT/BACK_PROBE_BED_POSITION)
 *
 * This will allow all positions to match at compilation, however
 * should the probe position be modified with M851XY then the
 * probe points will follow. This prevents any change from causing
 * the probe to be unable to reach any points.
 */
#if PROBE_SELECTED && !IS_KINEMATIC
  //#define PROBING_MARGIN_LEFT PROBING_MARGIN
  //#define PROBING_MARGIN_RIGHT PROBING_MARGIN
  //#define PROBING_MARGIN_FRONT PROBING_MARGIN
  //#define PROBING_MARGIN_BACK PROBING_MARGIN
#endif

#if EITHER(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL)
  // Override the mesh area if the automatic (max) area is too large
  //#define MESH_MIN_X MESH_INSET
  //#define MESH_MIN_Y MESH_INSET
  //#define MESH_MAX_X X_BED_SIZE - (MESH_INSET)
  //#define MESH_MAX_Y Y_BED_SIZE - (MESH_INSET)
#endif

#if BOTH(AUTO_BED_LEVELING_UBL, EEPROM_SETTINGS)
  //#define OPTIMIZED_MESH_STORAGE  // Store mesh with less precision to save EEPROM space
#endif

/**
 * Repeatedly attempt G29 leveling until it succeeds.
 * Stop after G29_MAX_RETRIES attempts.
 */
//#define G29_RETRY_AND_RECOVER
#if ENABLED(G29_RETRY_AND_RECOVER)
  #define G29_MAX_RETRIES 3
  #define G29_HALT_ON_FAILURE
  /**
   * Specify the GCODE commands that will be executed when leveling succeeds,
   * between attempts, and after the maximum number of retries have been tried.
   */
  #define G29_SUCCESS_COMMANDS "M117 Bed leveling done."
  #define G29_RECOVER_COMMANDS "M117 Probe failed. Rewiping.\nG28\nG12 P0 S12 T0"
  #define G29_FAILURE_COMMANDS "M117 Bed leveling failed.\nG0 Z10\nM300 P25 S880\nM300 P50 S0\nM300 P25 S880\nM300 P50 S0\nM300 P25 S880\nM300 P50 S0\nG4 S1"

#endif

//
// G2/G3 Arc Support
//
#define ARC_SUPPORT                 // Disable this feature to save ~3226 bytes
#if ENABLED(ARC_SUPPORT)
  #define MM_PER_ARC_SEGMENT      1 // (mm) Length (or minimum length) of each arc segment
  //#define ARC_SEGMENTS_PER_R    1 // Max segment length, MM_PER = Min
  #define MIN_ARC_SEGMENTS       24 // Minimum number of segments in a complete circle
  //#define ARC_SEGMENTS_PER_SEC 50 // Use feedrate to choose segment length (with MM_PER_ARC_SEGMENT as the minimum)
  #define N_ARC_CORRECTION       25 // Number of interpolated segments between corrections
  //#define ARC_P_CIRCLES           // Enable the 'P' parameter to specify complete circles
  //#define CNC_WORKSPACE_PLANES    // Allow G2/G3 to operate in XY, ZX, or YZ planes
  //#define SF_ARC_FIX              // Enable only if using SkeinForge with "Arc Point" fillet procedure
#endif

// Support for G5 with XYZE destination and IJPQ offsets. Requires ~2666 bytes.
#define BEZIER_CURVE_SUPPORT

/**
 * Direct Stepping
 *
 * Comparable to the method used by Klipper, G6 direct stepping significantly
 * reduces motion calculations, increases top printing speeds, and results in
 * less step aliasing by calculating all motions in advance.
 * Preparing your G-code: https://github.com/colinrgodsey/step-daemon
 */
//#define DIRECT_STEPPING

/**
 * G38 Probe Target
 *
 * This option adds G38.2 and G38.3 (probe towards target)
 * and optionally G38.4 and G38.5 (probe away from target).
 * Set MULTIPLE_PROBING for G38 to probe more than once.
 */
//#define G38_PROBE_TARGET
#if ENABLED(G38_PROBE_TARGET)
  //#define G38_PROBE_AWAY        // Include G38.4 and G38.5 to probe away from target
  #define G38_MINIMUM_MOVE 0.0275 // (mm) Minimum distance that will produce a move.
#endif

// Moves (or segments) with fewer steps than this will be joined with the next move
#define MIN_STEPS_PER_SEGMENT 6

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section motion

// The number of linear moves that can be in the planner at once.
// The value of BLOCK_BUFFER_SIZE must be a power of 2 (e.g. 8, 16, 32)
#if BOTH(SDSUPPORT, DIRECT_STEPPING)
  #define BLOCK_BUFFER_SIZE  8
#elif ENABLED(SDSUPPORT)
  #define BLOCK_BUFFER_SIZE 16
#else
  #define BLOCK_BUFFER_SIZE 16
#endif

// @section serial

// The ASCII buffer for serial input
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

// Transmission to Host Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256]
#define TX_BUFFER_SIZE 0

// Host Receive Buffer Size
// Without XON/XOFF flow control (see SERIAL_XON_XOFF below) 32 bytes should be enough.
// To use flow control, set this buffer size to at least 1024 bytes.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048]
//#define RX_BUFFER_SIZE 1024

#if RX_BUFFER_SIZE >= 1024
  // Enable to have the controller send XON/XOFF control characters to
  // the host to signal the RX buffer is becoming full.
  //#define SERIAL_XON_XOFF
#endif

// Add M575 G-code to change the baud rate
//#define BAUD_RATE_GCODE

#if ENABLED(SDSUPPORT)
  // Enable this option to collect and display the maximum
  // RX queue usage after transferring a file to SD.
  //#define SERIAL_STATS_MAX_RX_QUEUED

  // Enable this option to collect and display the number
  // of dropped bytes after a file transfer to SD.
  //#define SERIAL_STATS_DROPPED_RX
#endif

/**
 * Emergency Command Parser
 *
 * Add a low-level parser to intercept certain commands as they
 * enter the serial receive buffer, so they cannot be blocked.
 * Currently handles M108, M112, M410, M876
 * NOTE: Not yet implemented for all platforms.
 */
#define EMERGENCY_PARSER

// Bad Serial-connections can miss a received command by sending an 'ok'
// Therefore some clients abort after 30 seconds in a timeout.
// Some other clients start sending commands while receiving a 'wait'.
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.
//#define NO_TIMEOUTS 1000 // Milliseconds

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.
#define ADVANCED_OK

// Printrun may have trouble receiving long strings all at once.
// This option inserts short delays between lines of serial output.
#define SERIAL_OVERRUN_PROTECTION

// For serial echo, the number of digits after the decimal point
//#define SERIAL_FLOAT_PRECISION 4

// @section extras

/**
 * Extra Fan Speed
 * Adds a secondary fan speed for each print-cooling fan.
 *   'M106 P<fan> T3-255' : Set a secondary speed for <fan>
 *   'M106 P<fan> T2'     : Use the set secondary speed
 *   'M106 P<fan> T1'     : Restore the previous fan speed
 */
//#define EXTRA_FAN_SPEED

/**
 * Advanced Pause
 * Experimental feature for filament change support and for parking the nozzle when paused.
 * Adds the GCode M600 for initiating filament change.
 * If PARK_HEAD_ON_PAUSE enabled, adds the GCode M125 to pause printing and park the nozzle.
 *
 * Requires an LCD display.
 * Requires NOZZLE_PARK_FEATURE.
 * This feature is required for the default FILAMENT_RUNOUT_SCRIPT.
 */
#define ADVANCED_PAUSE_FEATURE
#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #define PAUSE_PARK_RETRACT_FEEDRATE         35  // (mm/s) Initial retract feedrate.
  #define PAUSE_PARK_RETRACT_LENGTH           1.5 // (mm) Initial retract.
                                                  // This short retract is done immediately, before parking the nozzle.
  #define FILAMENT_CHANGE_UNLOAD_FEEDRATE     35  // (mm/s) Unload filament feedrate. This can be pretty fast.
  #define FILAMENT_CHANGE_UNLOAD_ACCEL        25  // (mm/s^2) Lower acceleration may allow a faster feedrate.
  #define FILAMENT_CHANGE_UNLOAD_LENGTH       90  // (mm) The length of filament for a complete unload.
                                                  //   For Bowden, the full length of the tube and nozzle.
                                                  //   For direct drive, the full length of the nozzle.
                                                  //   Set to 0 for manual unloading.
  #define FILAMENT_CHANGE_SLOW_LOAD_FEEDRATE   6  // (mm/s) Slow move when starting load.
  #define FILAMENT_CHANGE_SLOW_LOAD_LENGTH     0  // (mm) Slow length, to allow time to insert material.
                                                  // 0 to disable start loading and skip to fast load only
  #define FILAMENT_CHANGE_FAST_LOAD_FEEDRATE  35  // (mm/s) Load filament feedrate. This can be pretty fast.
  #define FILAMENT_CHANGE_FAST_LOAD_ACCEL     25  // (mm/s^2) Lower acceleration may allow a faster feedrate.
  #define FILAMENT_CHANGE_FAST_LOAD_LENGTH    90  // (mm) Load length of filament, from extruder gear to nozzle.
                                                  //   For Bowden, the full length of the tube and nozzle.
                                                  //   For direct drive, the full length of the nozzle.
  #define ADVANCED_PAUSE_CONTINUOUS_PURGE         // Purge continuously up to the purge length until interrupted.
  #define ADVANCED_PAUSE_PURGE_FEEDRATE        3  // (mm/s) Extrude feedrate (after loading). Should be slower than load feedrate.
  #define ADVANCED_PAUSE_PURGE_LENGTH         15  // (mm) Length to extrude after loading.
                                                  //   Set to 0 for manual extrusion.
                                                  //   Filament can be extruded repeatedly from the Filament Change menu
                                                  //   until extrusion is consistent, and to purge old filament.
  #define ADVANCED_PAUSE_RESUME_PRIME          2  // (mm) Extra distance to prime nozzle after returning from park.
  //#define ADVANCED_PAUSE_FANS_PAUSE             // Turn off print-cooling fans while the machine is paused.

                                                  // Filament Unload does a Retract, Delay, and Purge first:
  #define FILAMENT_UNLOAD_PURGE_RETRACT        5  // (mm) Unload initial retract length.
  #define FILAMENT_UNLOAD_PURGE_DELAY       5000  // (ms) Delay for the filament to cool after retract.
  #define FILAMENT_UNLOAD_PURGE_LENGTH         0  // (mm) An unretract is done, then this length is purged.
  #define FILAMENT_UNLOAD_PURGE_FEEDRATE      25  // (mm/s) feedrate to purge before unload

  #define PAUSE_PARK_NOZZLE_TIMEOUT           55  // (seconds) Time limit before the nozzle is turned off for safety.
  #define FILAMENT_CHANGE_ALERT_BEEPS          6  // Number of alert beeps to play when a response is needed.
  #define PAUSE_PARK_NO_STEPPER_TIMEOUT           // Enable for XYZ steppers to stay powered on during filament change.

  #define PARK_HEAD_ON_PAUSE                      // Park the nozzle during pause and filament change.
  //#define HOME_BEFORE_FILAMENT_CHANGE             // If needed, home before parking for filament change

  #define FILAMENT_LOAD_UNLOAD_GCODES           // Add M701/M702 Load/Unload G-codes, plus Load/Unload in the LCD Prepare menu.
  //#define FILAMENT_UNLOAD_ALL_EXTRUDERS         // Allow M702 to unload all extruders above a minimum target temp (as set by M302)
#endif

#if HAS_TRINAMIC_CONFIG

  #define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current

  /**
   * Interpolate microsteps to 256
   * Override for each driver with <driver>_INTERPOLATE settings below
   */
  #define INTERPOLATE      true

  #if AXIS_IS_TMC(X)
    #define X_CURRENT       700        // (mA) RMS current. Multiply by 1.414 for peak current.
    #define X_CURRENT_HOME  X_CURRENT  // (mA) RMS current for sensorless homing
    #define X_MICROSTEPS     32        // 0..256
    #define X_RSENSE          0.11
    #define X_CHAIN_POS      -1        // -1..0: Not chained. 1: MCU MOSI connected. 2: Next in chain, ...
    //#define X_INTERPOLATE  true      // Enable to override 'INTERPOLATE' for the X axis
  #endif

  #if AXIS_IS_TMC(Y)
    #define Y_CURRENT       800
    #define Y_CURRENT_HOME  Y_CURRENT
    #define Y_MICROSTEPS     32
    #define Y_RSENSE          0.11
    #define Y_CHAIN_POS      -1
    //#define Y_INTERPOLATE  true
  #endif

  #if AXIS_IS_TMC(Z)
    #define Z_CURRENT       700
    #define Z_CURRENT_HOME  Z_CURRENT
    #define Z_MICROSTEPS     32
    #define Z_RSENSE          0.11
    #define Z_CHAIN_POS      -1
    //#define Z_INTERPOLATE  true
  #endif

  #if AXIS_IS_TMC(Z2)
    #define Z2_CURRENT      700
    #define Z2_CURRENT_HOME Z2_CURRENT
    #define Z2_MICROSTEPS    32
    #define Z2_RSENSE         0.11
    #define Z2_CHAIN_POS     -1
    //#define Z2_INTERPOLATE true
  #endif

  #if AXIS_IS_TMC(E0)
    #define E0_CURRENT      700
    #define E0_MICROSTEPS    32
    #define E0_RSENSE         0.11
    #define E0_CHAIN_POS     -1
    //#define E0_INTERPOLATE true
  #endif

  /**
   * Software enable
   *
   * Use for drivers that do not use a dedicated enable pin, but rather handle the same
   * function through a communication line such as SPI or UART.
   */
  //#define SOFTWARE_DRIVER_ENABLE

  /**
   * TMC2130, TMC2160, TMC2208, TMC2209, TMC5130 and TMC5160 only
   * Use Trinamic's ultra quiet stepping mode.
   * When disabled, Marlin will use spreadCycle stepping mode.
   */
  #define STEALTHCHOP_XY
  #define STEALTHCHOP_Z
  #define STEALTHCHOP_E

  /**
   * Optimize spreadCycle chopper parameters by using predefined parameter sets
   * or with the help of an example included in the library.
   * Provided parameter sets are
   * CHOPPER_DEFAULT_12V
   * CHOPPER_DEFAULT_19V
   * CHOPPER_DEFAULT_24V
   * CHOPPER_DEFAULT_36V
   * CHOPPER_09STEP_24V   // 0.9 degree steppers (24V)
   * CHOPPER_PRUSAMK3_24V // Imported parameters from the official Průša firmware for MK3 (24V)
   * CHOPPER_MARLIN_119   // Old defaults from Marlin v1.1.9
   *
   * Define your own with:
   * { <off_time[1..15]>, <hysteresis_end[-3..12]>, hysteresis_start[1..8] }
   */
  #define CHOPPER_TIMING CHOPPER_DEFAULT_12V        // All axes (override below)
  //#define CHOPPER_TIMING_X  CHOPPER_DEFAULT_12V   // For X Axes (override below)
  //#define CHOPPER_TIMING_X2 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_Y  CHOPPER_DEFAULT_12V   // For Y Axes (override below)
  //#define CHOPPER_TIMING_Y2 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_Z  CHOPPER_DEFAULT_12V   // For Z Axes (override below)
  //#define CHOPPER_TIMING_Z2 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_Z3 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_Z4 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E  CHOPPER_DEFAULT_12V   // For Extruders (override below)
  //#define CHOPPER_TIMING_E1 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E2 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E3 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E4 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E5 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E6 CHOPPER_DEFAULT_12V
  //#define CHOPPER_TIMING_E7 CHOPPER_DEFAULT_12V

  /**
   * Monitor Trinamic drivers
   * for error conditions like overtemperature and short to ground.
   * To manage over-temp Marlin can decrease the driver current until the error condition clears.
   * Other detected conditions can be used to stop the current print.
   * Relevant G-codes:
   * M906 - Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given.
   * M911 - Report stepper driver overtemperature pre-warn condition.
   * M912 - Clear stepper driver overtemperature pre-warn condition flag.
   * M122 - Report driver parameters (Requires TMC_DEBUG)
   */
  #define MONITOR_DRIVER_STATUS
  #if ENABLED(MONITOR_DRIVER_STATUS)
    #define CURRENT_STEP_DOWN     50  // [mA]
    #define REPORT_CURRENT_CHANGE
    #define STOP_ON_ERROR
  #endif

  /**
   * TMC2130, TMC2160, TMC2208, TMC2209, TMC5130 and TMC5160 only
   * The driver will switch to spreadCycle when stepper speed is over HYBRID_THRESHOLD.
   * This mode allows for faster movements at the expense of higher noise levels.
   * STEALTHCHOP_(XY|Z|E) must be enabled to use HYBRID_THRESHOLD.
   * M913 X/Y/Z/E to live tune the setting
   */
  //#define HYBRID_THRESHOLD
  #define X_HYBRID_THRESHOLD     100  // [mm/s]
  #define X2_HYBRID_THRESHOLD    100
  #define Y_HYBRID_THRESHOLD     100
  #define Y2_HYBRID_THRESHOLD    100
  #define Z_HYBRID_THRESHOLD       3
  #define Z2_HYBRID_THRESHOLD      3
  #define Z3_HYBRID_THRESHOLD      3
  #define Z4_HYBRID_THRESHOLD      3
  #define E0_HYBRID_THRESHOLD     30
  #define E1_HYBRID_THRESHOLD     30
  #define E2_HYBRID_THRESHOLD     30
  #define E3_HYBRID_THRESHOLD     30
  #define E4_HYBRID_THRESHOLD     30
  #define E5_HYBRID_THRESHOLD     30
  #define E6_HYBRID_THRESHOLD     30
  #define E7_HYBRID_THRESHOLD     30

  /**
   * TMC Homing stepper phase.
   *
   * Improve homing repeatability by homing to stepper coil's nearest absolute
   * phase position. Trinamic drivers use a stepper phase table with 1024 values
   * spanning 4 full steps with 256 positions each (ergo, 1024 positions).
   * Full step positions (128, 384, 640, 896) have the highest holding torque.
   *
   * Values from 0..1023, -1 to disable homing phase for that axis.
   */
   //#define TMC_HOME_PHASE { 896, 896, 896 }

  /**
   * Beta feature!
   * Create a 50/50 square wave step pulse optimal for stepper drivers.
   */
  //#define SQUARE_WAVE_STEPPING

  /**
   * Enable M122 debugging command for TMC stepper drivers.
   * M122 S0/1 will enable continous reporting.
   */
  #define TMC_DEBUG

  /**
   * You can set your own advanced settings by filling in predefined functions.
   * A list of available functions can be found on the library github page
   * https://github.com/teemuatlut/TMCStepper
   *
   * Example:
   * #define TMC_ADV() { \
   *   stepperX.diag0_otpw(1); \
   *   stepperY.intpol(0); \
   * }
   */
  #define TMC_ADV() {  }

#endif // HAS_TRINAMIC_CONFIG

/**
 * Photo G-code
 * Add the M240 G-code to take a photo.
 * The photo can be triggered by a digital pin or a physical movement.
 */
#define PHOTO_GCODE
#if ENABLED(PHOTO_GCODE)
  // A position to move to (and raise Z) before taking the photo
  #define PHOTO_POSITION { X_MAX_POS - 5, Y_MAX_POS, 2 }  // { xpos, ypos, zraise } (M240 X Y Z)
  #define PHOTO_DELAY_MS   500                            // (ms) Duration to pause before moving back (M240 P)
  //#define PHOTO_RETRACT_MM   6.5                          // (mm) E retract/recover for the photo move (M240 R S)

  // Canon RC-1 or homebrew digital camera trigger
  // Data from: https://www.doc-diy.net/photo/rc-1_hacked/
  //#define PHOTOGRAPH_PIN 23

  // Canon Hack Development Kit
  // https://captain-slow.dk/2014/03/09/3d-printing-timelapses/
  //#define CHDK_PIN        4

  // Optional second move with delay to trigger the camera shutter
  //#define PHOTO_SWITCH_POSITION { X_MAX_POS, Y_MAX_POS }  // { xpos, ypos } (M240 I J)

  // Duration to hold the switch or keep CHDK_PIN high
  //#define PHOTO_SWITCH_MS   50 // (ms) (M240 D)

  /**
   * PHOTO_PULSES_US may need adjustment depending on board and camera model.
   * Pin must be running at 48.4kHz.
   * Be sure to use a PHOTOGRAPH_PIN which can rise and fall quick enough.
   * (e.g., MKS SBase temp sensor pin was too slow, so used P1.23 on J8.)
   *
   *  Example pulse data for Nikon: https://bit.ly/2FKD0Aq
   *                     IR Wiring: https://git.io/JvJf7
   */
  //#define PHOTO_PULSES_US { 2000, 27850, 400, 1580, 400, 3580, 400 }  // (µs) Durations for each 48.4kHz oscillation
  #ifdef PHOTO_PULSES_US
    #define PHOTO_PULSE_DELAY_US 13 // (µs) Approximate duration of each HIGH and LOW pulse in the oscillation
  #endif
#endif

/**
 * Auto-report temperatures with M155 S<seconds>
 */
#define AUTO_REPORT_TEMPERATURES

/**
 * Include capabilities in M115 output
 */
#define EXTENDED_CAPABILITIES_REPORT
#if ENABLED(EXTENDED_CAPABILITIES_REPORT)
  //#define M115_GEOMETRY_REPORT
#endif

/**
 * Disable all Volumetric extrusion options
 */
//#define NO_VOLUMETRICS

#if DISABLED(NO_VOLUMETRICS)
  /**
   * Volumetric extrusion default state
   * Activate to make volumetric extrusion the default method,
   * with DEFAULT_NOMINAL_FILAMENT_DIA as the default diameter.
   *
   * M200 D0 to disable, M200 Dn to set a new diameter (and enable volumetric).
   * M200 S0/S1 to disable/enable volumetric extrusion.
   */
  //#define VOLUMETRIC_DEFAULT_ON

  //#define VOLUMETRIC_EXTRUDER_LIMIT
  #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
    /**
     * Default volumetric extrusion limit in cubic mm per second (mm^3/sec).
     * This factory setting applies to all extruders.
     * Use 'M200 [T<extruder>] L<limit>' to override and 'M502' to reset.
     * A non-zero value activates Volume-based Extrusion Limiting.
     */
    #define DEFAULT_VOLUMETRIC_EXTRUDER_LIMIT 0.00      // (mm^3/sec)
  #endif
#endif

// Extra options for the M114 "Current Position" report
#define M114_DETAIL           // Use 'M114` for details to check planner calculations
//#define M114_REALTIME       // Real current position based on forward kinematics
//#define M114_LEGACY         // M114 used to synchronize on every call. Enable if needed.

//#define REPORT_FAN_CHANGE   // Report the new fan speed when changed by M106 (and others)

/**
 * Set the number of proportional font spaces required to fill up a typical character space.
 * This can help to better align the output of commands like `G29 O` Mesh Output.
 *
 * For clients that use a fixed-width font (like OctoPrint), leave this set to 1.0.
 * Otherwise, adjust according to your client and font.
 */
#define PROPORTIONAL_FONT_RATIO 1.0

/**
 * Spend 28 bytes of SRAM to optimize the GCode parser
 */
#define FASTER_GCODE_PARSER

#if ENABLED(FASTER_GCODE_PARSER)
  //#define GCODE_QUOTED_STRINGS  // Support for quoted string parameters
#endif

//#define MEATPACK                // Support for MeatPack G-code compression (https://github.com/scottmudge/OctoPrint-MeatPack)

//#define GCODE_CASE_INSENSITIVE  // Accept G-code sent to the firmware in lowercase

//#define REPETIER_GCODE_M360     // Add commands originally from Repetier FW

/**
 * Cancel Objects
 *
 * Implement M486 to allow Marlin to skip objects
 */
#define CANCEL_OBJECTS

/**
 * Advanced Print Counter settings
 */
#if ENABLED(PRINTCOUNTER)
  #define SERVICE_WARNING_BUZZES  3
  // Activate up to 3 service interval watchdogs
  //#define SERVICE_NAME_1      "Service S"
  //#define SERVICE_INTERVAL_1  100 // print hours
  //#define SERVICE_NAME_2      "Service L"
  //#define SERVICE_INTERVAL_2  200 // print hours
  //#define SERVICE_NAME_3      "Service 3"
  //#define SERVICE_INTERVAL_3    1 // print hours
#endif

// @section develop

//
// M100 Free Memory Watcher to debug memory usage
//
//#define M100_FREE_MEMORY_WATCHER

//
// M42 - Set pin states
//
//#define DIRECT_PIN_CONTROL

//
// M43 - display pin status, toggle pins, watch pins, watch endstops & toggle LED, test servo probe
//
//#define PINS_DEBUGGING

// Enable Marlin dev mode which adds some special commands
//#define MARLIN_DEV_MODE

/**
 * 
 *                                END
 *                                    OF
 *                                       FILE
 * 
 */
