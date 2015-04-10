/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_RCINPUT_H
#define AC_RCINPUT_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <RC_Channel.h>     // RC Channel Library

class RCInput
{
public:
    /// Constructor
    ///
    RCInput(void);

    // RC Input Function enumeration
    enum rc_in_func {
        NO_FUNCTION =                        0,  // rc input disabled
        CTRL_ROLL =                          1,  // Roll control input
        CTRL_PITCH =                         2,  // Pitch control input
        CTRL_THROTTLE =                      3,  // Throttle control input
        CTRL_YAW =                           4,  // Yaw control input
        SW_FLIGHT_MODE =                     5,  // Flight Mode switch
        SW_FLIP =                           10,  // flip
        SW_SIMPLE_MODE =                    11,  // change to simple mode
        SW_RTL =                            12,  // change to RTL flight mode
        SW_SAVE_TRIM =                      13,  // save current position as level
        SW_SAVE_WP =                        14,  // save mission waypoint or RTL if in auto mode
        SW_CAMERA_TRIGGER =                 15,  // trigger camera servo or relay
        SW_GROUND_RANGEFINDER =             16,  // allow enabling or disabling ground rangefinder in flight which helps avoid surface tracking when you are far above the ground
        SW_FENCE =                          17,  // allow enabling or disabling fence in flight
        SW_RESETTOARMEDYAW =                18,  // changes yaw to be same as when quad was armed
        SW_SUPERSIMPLE_MODE =               19,  // change to simple mode in middle, super simple at top
        SW_ACRO_TRAINER =                   20,  // low = disabled, middle = leveled, high = leveled and limited
        SW_SPRAYER =                        21,  // enable/disable the crop sprayer
        SW_AUTO =                           22,  // change to auto flight mode
        SW_AUTOTUNE =                       23,  // auto tune
        SW_LAND =                           24,  // change to LAND flight mode
        SW_EPM =                            25,  // Operate the EPM cargo gripper low=off, middle=neutral, high=on
        SW_PARACHUTE_ENABLE =               26,  // Parachute enable/disable
        SW_PARACHUTE_RELEASE =              27,  // Parachute release
        SW_PARACHUTE_3POS =                 28,  // Parachute disable, enable, release with 3 position switch
        SW_MISSION_RESET =                  29,  // Reset auto mission to start from first command
        SW_ATTCON_FEEDFWD =                 30,  // enable/disable the roll and pitch rate feed forward
        SW_ATTCON_ACCEL_LIM =               31,  // enable/disable the roll, pitch and yaw accel limiting
        SW_RETRACT_MOUNT =                  32,  // Retract Mount
        SW_RELAY =                          33,  // Relay pin on/off (only supports first relay)
        SW_LANDING_GEAR =                   34,  // Landing gear controller
        SW_MOTOR_ESTOP =                    35,  // Emergency Stop Switch
        SW_MOTOR_INTERLOCK =                36,  // Motor On/Off switch
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

    // RC Input Switch Flags:
    struct flags {
        uint8_t ch5; // ch5 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch6; // ch6 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch7; // ch7 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch8; // ch8 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch9; // ch9 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch10; // ch10 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch11; // ch11 switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t ch12; // ch12 switch : 0 is low or false, 1 is center or true, 2 is high
    } switch_flag;

    //Search for first channel with assigned function
    uint8_t find_rc_input_func(int16_t func) const;

    // check_if_rc_input_func_used - Check to see if any of the RC input switches are set to a given mode.
    bool check_if_rc_input_func_used(int16_t check_function);

    // check_duplicate_rc_input_func - Check to see if any RC input switch functions are duplicated
    bool check_duplicate_rc_input_func();

    // get_flight_mode_switch_position - Determine position of flight mode switch and return.
    uint8_t get_flight_mode_switch_position();

    // get_num_tuning_channels - return number of tuning channels found.
    uint8_t get_num_tuning_channels() {return _num_tuning_channels;}

    // get_tuning_value_1 - return tuning value for tuning channel 1.
    float get_tuning_value_1();
    // get_tuning_value_2 - return tuning value for tuning channel 2.
    float get_tuning_value_2();
    // get_tuning_value_3 - return tuning value for tuning channel 3.
    float get_tuning_value_3();

    // get_tuning_function_1 - return assigned tuning function 1.
    uint16_t get_tuning_function_1() {return _tuning_function[0];}
    // get_tuning_function_2 - return assigned tuning function 2.
    uint16_t get_tuning_function_2() {return _tuning_function[1];}
    // get_tuning_function_3 - return assigned tuning function 3.
    uint16_t get_tuning_function_3() {return _tuning_function[2];}

    // accessors to get primary flight control channel assignments
    uint8_t roll_chan() {return _roll_chan;}
    uint8_t pitch_chan() {return _pitch_chan;}
    uint8_t throttle_chan() {return _throttle_chan;}
    uint8_t yaw_chan() {return _yaw_chan;}
    uint8_t flight_mode_chan() {return _flight_mode_chan;}

    // accessors to get auxiliary control functions
    int16_t ch5_function() {return _ch5_function;}
    int16_t ch6_function() {return _ch6_function;}
    int16_t ch7_function() {return _ch7_function;}
    int16_t ch8_function() {return _ch8_function;}
    int16_t ch9_function() {return _ch9_function;}
    int16_t ch10_function() {return _ch10_function;}
    int16_t ch11_function() {return _ch11_function;}
    int16_t ch12_function() {return _ch12_function;}

    void set_primary_control_channels();

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Primary flight control channel assignments
    // These should be locked after arming
    uint8_t _roll_chan;
    uint8_t _pitch_chan;
    uint8_t _throttle_chan;
    uint8_t _yaw_chan;
    uint8_t _flight_mode_chan;
    uint8_t _tuning_chan[3];            // Array holding up to 3 tuning channel assignments
    uint8_t _num_tuning_channels;       // Number of tuning channels found

    uint16_t _tuning_function[3];

    AP_Int16 * _ch_functions[13];

    // channel mappings
    AP_Int16 _ch1_function;
    AP_Int16 _ch2_function;
    AP_Int16 _ch3_function;
    AP_Int16 _ch4_function;
    AP_Int16 _ch5_function;
    AP_Int16 _ch6_function;
    AP_Int16 _ch7_function;
    AP_Int16 _ch8_function;
    AP_Int16 _ch9_function;
    AP_Int16 _ch10_function;
    AP_Int16 _ch11_function;
    AP_Int16 _ch12_function;
    AP_Float _tuning_1_low;
    AP_Float _tuning_1_high;
    AP_Float _tuning_2_low;
    AP_Float _tuning_2_high;
    AP_Float _tuning_3_low;
    AP_Float _tuning_3_high;
};
#endif
