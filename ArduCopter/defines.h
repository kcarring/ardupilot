// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

#include <AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// Autopilot Yaw Mode enumeration
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =             0,  // pilot controls the heading
    AUTO_YAW_LOOK_AT_NEXT_WP =  1,  // point towards next waypoint (no pilot input accepted)
    AUTO_YAW_ROI =              2,  // point towards a location held in roi_WP (no pilot input accepted)
    AUTO_YAW_LOOK_AT_HEADING =  3,  // point towards a particular angle (not pilot input accepted)
    AUTO_YAW_LOOK_AHEAD =       4,  // point in the direction the copter is moving
    AUTO_YAW_RESETTOARMEDYAW =  5,  // point towards heading at time motors were armed
};

// Ch6... Ch12 aux switch control
#define AUX_SWITCH_PWM_TRIGGER_HIGH 1800   // pwm value above which the ch7 or ch8 option will be invoked
#define AUX_SWITCH_PWM_TRIGGER_LOW  1200   // pwm value below which the ch7 or ch8 option will be disabled
#define CH6_PWM_TRIGGER_HIGH    1800
#define CH6_PWM_TRIGGER_LOW     1200

// values used by the ap.ch7_opt and ap.ch8_opt flags
#define AUX_SWITCH_LOW              0       // indicates auxiliar switch is in the low position (pwm <1200)
#define AUX_SWITCH_MIDDLE           1       // indicates auxiliar switch is in the middle position (pwm >1200, <1800)
#define AUX_SWITCH_HIGH             2       // indicates auxiliar switch is in the high position (pwm >1800)

// Aux Switch enumeration
enum aux_sw_func {
    AUXSW_DO_NOTHING =           0,     // 0, aux switch disabled
                                        // Enum values 1-9 saved for flight controls
    AUXSW_FLIP =                10,     // flip
    AUXSW_SIMPLE_MODE =         11,     // change to simple mode
    AUXSW_RTL =                 12,     // change to RTL flight mode
    AUXSW_SAVE_TRIM =           13,     // save current position as level
    AUXSW_SAVE_WP =             14,     // save mission waypoint or RTL if in auto mode
    AUXSW_CAMERA_TRIGGER =      15,     // trigger camera servo or relay
    AUXSW_GROUND_RANGEFINDER =  16,     // allow enabling or disabling ground rangefinder in flight which helps avoid surface tracking when you are far above the ground
    AUXSW_FENCE =               17,     // allow enabling or disabling fence in flight
    AUXSW_RESETTOARMEDYAW =     18,     // changes yaw to be same as when quad was armed
    AUXSW_SUPERSIMPLE_MODE =    19,     // change to simple mode in middle, super simple at top
    AUXSW_ACRO_TRAINER =        20,     // low = disabled, middle = leveled, high = leveled and limited
    AUXSW_SPRAYER =             21,     // enable/disable the crop sprayer
    AUXSW_AUTO =                22,     // change to auto flight mode
    AUXSW_AUTOTUNE =            23,     // auto tune
    AUXSW_LAND =                24,     // change to LAND flight mode
    AUXSW_EPM =                 25,     // Operate the EPM cargo gripper low=off, middle=neutral, high=on
    AUXSW_PARACHUTE_ENABLE =    26,     // Parachute enable/disable
    AUXSW_PARACHUTE_RELEASE =   27,     // Parachute release
    AUXSW_PARACHUTE_3POS =      28,     // Parachute disable, enable, release with 3 position switch
    AUXSW_MISSION_RESET =       29,     // Reset auto mission to start from first command
    AUXSW_ATTCON_FEEDFWD =      30,     // enable/disable the roll and pitch rate feed forward
    AUXSW_ATTCON_ACCEL_LIM =    31,     // enable/disable the roll, pitch and yaw accel limiting
    AUXSW_RETRACT_MOUNT =       32,     // Retract Mount
    AUXSW_RELAY =               33,     // Relay pin on/off (only supports first relay)
    AUXSW_LANDING_GEAR =        34,     // Landing gear controller
    AUXSW_MOTOR_ESTOP =         35,     // Emergency Stop Switch
    AUXSW_MOTOR_INTERLOCK =     36,     // Motor On/Off switch
};

// Frame types
#define UNDEFINED_FRAME 0
#define QUAD_FRAME 1
#define TRI_FRAME 2
#define HEXA_FRAME 3
#define Y6_FRAME 4
#define OCTA_FRAME 5
#define HELI_FRAME 6
#define OCTA_QUAD_FRAME 7
#define SINGLE_FRAME 8
#define COAX_FRAME 9

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

// HIL enumerations
#define HIL_MODE_DISABLED               0
#define HIL_MODE_SENSORS                1

// Auto Pilot Modes enumeration
enum autopilot_modes {
    STABILIZE =     0,  // 0, manual airframe angle with manual throttle
    ACRO =          1,  // 1, manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // 2, manual airframe angle with automatic throttle
    AUTO =          3,  // 3, fully automatic waypoint control
    GUIDED =        4,  // 4, fully automatic fly to coordinate or fly at velocity/direction
    LOITER =        5,  // 5, automatic horizontal acceleration with automatic throttle
    RTL =           6,  // 6, automatic return to launching point
    CIRCLE =        7,  // 7, automatic circular flight with automatic throttle
    LAND =          9,  // 9, automatic landing with horizontal position control
    OF_LOITER =    10,  // 10, automatic position control using optical flow sensor
    DRIFT =        11,  // 11, semi-automatic position, yaw and throttle control
    SPORT =        13,  // 13, manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // 14, automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // 15, automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // 16, automatic position hold with manual override, with automatic throttle
};

// Tuning enumeration
enum tuning_func {
    TUNING_NONE =                        0,     // Not Used
    TUNING_STABILIZE_ROLL_PITCH_KP =    101,    // stabilize roll/pitch angle controller's P term
    TUNING_STABILIZE_YAW_KP =           102,    // stabilize yaw heading controller's P term
    TUNING_RATE_ROLL_PITCH_KP =         103,    // body frame roll/pitch rate controller's P term
    TUNING_RATE_ROLL_PITCH_KI =         104,    // body frame roll/pitch rate controller's I term
    TUNING_RATE_ROLL_PITCH_KD =         105,    // body frame roll/pitch rate controller's D term
    TUNING_RATE_PITCH_KP =              106,    // body frame pitch rate controller's P term
    TUNING_RATE_PITCH_KI =              107,    // body frame pitch rate controller's I term
    TUNING_RATE_PITCH_KD =              108,    // body frame pitch rate controller's D term
    TUNING_RATE_PITCH_FF =              109,    // body frame pitch rate controller FF term (TradHeli Only)
    TUNING_RATE_ROLL_KP =               110,    // body frame roll rate controller's P term
    TUNING_RATE_ROLL_KI =               111,    // body frame roll rate controller's I term
    TUNING_RATE_ROLL_KD =               112,    // body frame roll rate controller's D term
    TUNING_RATE_ROLL_FF =               113,    // body frame roll rate controller FF term (TradHeli Only)
    TUNING_RATE_YAW_KP =                114,    // body frame yaw rate controller's P term
    TUNING_RATE_YAW_KI =                115,    // body frame yaw rate controller's I term
    TUNING_RATE_YAW_KD =                116,    // body frame yaw rate controller's D term
    TUNING_RATE_YAW_FF =                117,    // body frame yaw rate controller FF term (TradHeli Only)
    TUNING_RATE_RP_FILT =               118,    // roll and pitch rate input filter
    TUNING_RATE_YAW_FILT =              119,    // yaw rate input filter
    TUNING_ALTITUDE_HOLD_KP =           120,    // altitude hold controller's P term (alt error to desired rate)
    TUNING_THROTTLE_RATE_KP =           121,    // throttle rate controller's P term (desired rate to acceleration or motor output)
    TUNING_ACCEL_Z_KP =                 122,    // accel based throttle controller's P term
    TUNING_ACCEL_Z_KI =                 123,    // accel based throttle controller's I term
    TUNING_ACCEL_Z_KD =                 124,    // accel based throttle controller's D term
    TUNING_WP_SPEED =                   125,    // maximum speed to next way point (0 to 10m/s)
    TUNING_CIRCLE_RATE =                126,    // circle turn rate in degrees (hard coded to about 45 degrees in either direction)
    TUNING_SONAR_GAIN =                 127,    // sonar gain
    TUNING_LOITER_POSITION_KP =         128,    // loiter distance controller's P term (position error to speed)
    TUNING_VEL_XY_KP =                  129,    // loiter rate controller's P term (speed error to tilt angle)
    TUNING_VEL_XY_KI =                  130,    // loiter rate controller's I term (speed error to tilt angle)
    TUNING_ACRO_RP_KP =                 131,    // acro controller's P term.  converts pilot input to a desired roll, pitch or yaw rate
    TUNING_ACRO_YAW_KP =                132,    // acro controller's P term.  converts pilot input to a desired roll, pitch or yaw rate
    TUNING_AHRS_RP_KP =                 133,    // accelerometer effect on roll/pitch angle (0=low)
    TUNING_AHRS_YAW_KP =                134,    // ahrs's compass effect on yaw angle (0 = very low, 1 = very high)
    TUNING_DECLINATION =                135,    // compass declination in radians
    TUNING_RC_FEEL_RP =                 136,    // roll-pitch input smoothing
    TUNING_RATE_MOT_YAW_HEADROOM =      137,    // motors yaw headroom minimum
    TUNING_HELI_EXTERNAL_GYRO =         138,    // TradHeli specific external tail gyro gain
//  TUNING_EKF_VERTICAL_POS =           139,    // EKF's baro vs accel (higher rely on accels more, baro impact is reduced).  Range should be 0.2 ~ 4.0?  2.0 is default
//  TUNING_EKF_HORIZONTAL_POS =         140,    // EKF's gps vs accel (higher rely on accels more, gps impact is reduced).  Range should be 1.0 ~ 3.0?  1.5 is default
//  TUNING_EKF_ACCEL_NOISE =            141,    // EKF's accel noise (lower means trust accels more, gps & baro less).  Range should be 0.02 ~ 0.5  0.5 is default (but very robust at that level)
};

// Acro Trainer types
#define ACRO_TRAINER_DISABLED   0
#define ACRO_TRAINER_LEVELING   1
#define ACRO_TRAINER_LIMITED    2

// RC Feel roll/pitch definitions
#define RC_FEEL_RP_VERY_SOFT        0
#define RC_FEEL_RP_SOFT             25
#define RC_FEEL_RP_MEDIUM           50
#define RC_FEEL_RP_CRISP            75
#define RC_FEEL_RP_VERY_CRISP       100

// Yaw behaviours during missions - possible values for WP_YAW_BEHAVIOR parameter
#define WP_YAW_BEHAVIOR_NONE                          0   // auto pilot will never control yaw during missions or rtl (except for DO_CONDITIONAL_YAW command received)
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1   // auto pilot will face next waypoint or home during rtl
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2   // auto pilot will face next waypoint except when doing RTL at which time it will stay in it's last
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3   // auto pilot will look ahead during missions and rtl (primarily meant for traditional helicotpers)

// Auto modes
enum AutoMode {
    Auto_TakeOff,
    Auto_WP,
    Auto_Land,
    Auto_RTL,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline,
    Auto_NavGuided,
    Auto_Loiter
};

// Guided modes
enum GuidedMode {
    Guided_TakeOff,
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel
};

// RTL states
enum RTLState {
    InitialClimb,
    ReturnHome,
    LoiterAtHome,
    FinalDescent,
    Land
};

// Flip states
enum FlipState {
    Flip_Start,
    Flip_Roll,
    Flip_Pitch_A,
    Flip_Pitch_B,
    Flip_Recover,
    Flip_Abandon
};

// LAND state
#define LAND_STATE_FLY_TO_LOCATION  0
#define LAND_STATE_DESCENDING       1

//  Logging parameters
#define TYPE_AIRSTART_MSG               0x00
#define TYPE_GROUNDSTART_MSG            0x01
#define LOG_CONTROL_TUNING_MSG          0x04
#define LOG_NAV_TUNING_MSG              0x05
#define LOG_PERFORMANCE_MSG             0x06
#define LOG_STARTUP_MSG                 0x0A
#define LOG_OPTFLOW_MSG                 0x0C
#define LOG_EVENT_MSG                   0x0D
#define LOG_PID_MSG                     0x0E    // deprecated
#define LOG_INAV_MSG                    0x11    // deprecated
#define LOG_CAMERA_MSG_DEPRECATED       0x12    // deprecated
#define LOG_ERROR_MSG                   0x13
#define LOG_DATA_INT16_MSG              0x14
#define LOG_DATA_UINT16_MSG             0x15
#define LOG_DATA_INT32_MSG              0x16
#define LOG_DATA_UINT32_MSG             0x17
#define LOG_DATA_FLOAT_MSG              0x18
#define LOG_AUTOTUNE_MSG                0x19
#define LOG_AUTOTUNEDETAILS_MSG         0x1A
#define LOG_RATE_MSG                    0x1D
#define LOG_MOTBATT_MSG                 0x1E

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_RCIN                   (1<<6)
#define MASK_LOG_IMU                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CURRENT                (1<<9)
#define MASK_LOG_RCOUT                  (1<<10)
#define MASK_LOG_OPTFLOW                (1<<11)
#define MASK_LOG_PID                    (1<<12) // deprecated
#define MASK_LOG_COMPASS                (1<<13)
#define MASK_LOG_INAV                   (1<<14) // deprecated
#define MASK_LOG_CAMERA                 (1<<15)
#define MASK_LOG_WHEN_DISARMED          (1UL<<16)
#define MASK_LOG_MOTBATT                (1UL<<17)
#define MASK_LOG_ANY                    0xFFFF

// DATA - event logging
#define DATA_MAVLINK_FLOAT              1
#define DATA_MAVLINK_INT32              2
#define DATA_MAVLINK_INT16              3
#define DATA_MAVLINK_INT8               4
#define DATA_AP_STATE                   7
#define DATA_SYSTEM_TIME_SET            8
#define DATA_INIT_SIMPLE_BEARING        9
#define DATA_ARMED                      10
#define DATA_DISARMED                   11
#define DATA_AUTO_ARMED                 15
#define DATA_TAKEOFF                    16
#define DATA_LAND_COMPLETE_MAYBE        17
#define DATA_LAND_COMPLETE              18
#define DATA_NOT_LANDED                 28
#define DATA_LOST_GPS                   19
#define DATA_FLIP_START                 21
#define DATA_FLIP_END                   22
#define DATA_SET_HOME                   25
#define DATA_SET_SIMPLE_ON              26
#define DATA_SET_SIMPLE_OFF             27
#define DATA_SET_SUPERSIMPLE_ON         29
#define DATA_AUTOTUNE_INITIALISED       30
#define DATA_AUTOTUNE_OFF               31
#define DATA_AUTOTUNE_RESTART           32
#define DATA_AUTOTUNE_SUCCESS           33
#define DATA_AUTOTUNE_FAILED            34
#define DATA_AUTOTUNE_REACHED_LIMIT     35
#define DATA_AUTOTUNE_PILOT_TESTING     36
#define DATA_AUTOTUNE_SAVEDGAINS        37
#define DATA_SAVE_TRIM                  38
#define DATA_SAVEWP_ADD_WP              39
#define DATA_SAVEWP_CLEAR_MISSION_RTL   40
#define DATA_FENCE_ENABLE               41
#define DATA_FENCE_DISABLE              42
#define DATA_ACRO_TRAINER_DISABLED      43
#define DATA_ACRO_TRAINER_LEVELING      44
#define DATA_ACRO_TRAINER_LIMITED       45
#define DATA_EPM_GRAB                   46
#define DATA_EPM_RELEASE                47
#define DATA_EPM_NEUTRAL                48  // deprecated
#define DATA_PARACHUTE_DISABLED         49
#define DATA_PARACHUTE_ENABLED          50
#define DATA_PARACHUTE_RELEASED         51
#define DATA_LANDING_GEAR_DEPLOYED      52
#define DATA_LANDING_GEAR_RETRACTED     53

// Centi-degrees to radians
#define DEGX100 5729.57795f

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// Error message sub systems and error codes
#define ERROR_SUBSYSTEM_MAIN                1
#define ERROR_SUBSYSTEM_RADIO               2
#define ERROR_SUBSYSTEM_COMPASS             3
#define ERROR_SUBSYSTEM_OPTFLOW             4
#define ERROR_SUBSYSTEM_FAILSAFE_RADIO      5
#define ERROR_SUBSYSTEM_FAILSAFE_BATT       6
#define ERROR_SUBSYSTEM_FAILSAFE_GPS        7   // not used
#define ERROR_SUBSYSTEM_FAILSAFE_GCS        8
#define ERROR_SUBSYSTEM_FAILSAFE_FENCE      9
#define ERROR_SUBSYSTEM_FLIGHT_MODE         10
#define ERROR_SUBSYSTEM_GPS                 11  // not used
#define ERROR_SUBSYSTEM_CRASH_CHECK         12
#define ERROR_SUBSYSTEM_FLIP                13
#define ERROR_SUBSYSTEM_AUTOTUNE            14
#define ERROR_SUBSYSTEM_PARACHUTE           15
#define ERROR_SUBSYSTEM_EKFINAV_CHECK       16
#define ERROR_SUBSYSTEM_FAILSAFE_EKFINAV    17
#define ERROR_SUBSYSTEM_BARO                18
#define ERROR_SUBSYSTEM_CPU                 19
// general error codes
#define ERROR_CODE_ERROR_RESOLVED           0
#define ERROR_CODE_FAILED_TO_INITIALISE     1
// subsystem specific error codes -- radio
#define ERROR_CODE_RADIO_LATE_FRAME         2
// subsystem specific error codes -- failsafe_thr, batt, gps
#define ERROR_CODE_FAILSAFE_RESOLVED        0
#define ERROR_CODE_FAILSAFE_OCCURRED        1
// subsystem specific error codes -- compass
#define ERROR_CODE_COMPASS_FAILED_TO_READ   2
// subsystem specific error codes -- main
#define ERROR_CODE_MAIN_INS_DELAY           1
// subsystem specific error codes -- crash checker
#define ERROR_CODE_CRASH_CHECK_CRASH        1
#define ERROR_CODE_CRASH_CHECK_LOSS_OF_CONTROL 2
// subsystem specific error codes -- flip
#define ERROR_CODE_FLIP_ABANDONED           2
// subsystem specific error codes -- autotune
#define ERROR_CODE_AUTOTUNE_BAD_GAINS       2
// parachute failed to deploy because of low altitude
#define ERROR_CODE_PARACHUTE_TOO_LOW        2
// EKF check definitions
#define ERROR_CODE_EKFINAV_CHECK_BAD_VARIANCE       2
#define ERROR_CODE_EKFINAV_CHECK_VARIANCE_CLEARED   0
// Baro specific error codes
#define ERROR_CODE_BARO_GLITCH              2

// Arming Check Enable/Disable bits
#define ARMING_CHECK_NONE                   0x00
#define ARMING_CHECK_ALL                    0x01
#define ARMING_CHECK_BARO                   0x02
#define ARMING_CHECK_COMPASS                0x04
#define ARMING_CHECK_GPS                    0x08
#define ARMING_CHECK_INS                    0x10
#define ARMING_CHECK_PARAMETERS             0x20
#define ARMING_CHECK_RC                     0x40
#define ARMING_CHECK_VOLTAGE                0x80

// Radio failsafe definitions (FS_THR parameter)
#define FS_THR_DISABLED                    0
#define FS_THR_ENABLED_ALWAYS_RTL          1
#define FS_THR_ENABLED_CONTINUE_MISSION    2
#define FS_THR_ENABLED_ALWAYS_LAND         3

// Battery failsafe definitions (FS_BATT_ENABLE parameter)
#define FS_BATT_DISABLED                    0       // battery failsafe disabled
#define FS_BATT_LAND                        1       // switch to LAND mode on battery failsafe
#define FS_BATT_RTL                         2       // switch to RTL mode on battery failsafe

// GPS Failsafe definitions (FS_GPS_ENABLE parameter)
#define FS_GPS_DISABLED                     0       // GPS failsafe disabled
#define FS_GPS_LAND                         1       // switch to LAND mode on GPS Failsafe
#define FS_GPS_ALTHOLD                      2       // switch to ALTHOLD mode on GPS failsafe
#define FS_GPS_LAND_EVEN_STABILIZE          3       // switch to LAND mode on GPS failsafe even if in a manual flight mode like Stabilize

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<11)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<12)

#endif // _DEFINES_H
