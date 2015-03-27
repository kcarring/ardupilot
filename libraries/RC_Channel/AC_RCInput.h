/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_RCINPUT_H
#define AC_RCINPUT_H

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>

class RCInput
{
public:
    /// Constructor
    ///
    RCInput();

// RC Input Function enumeration
enum rc_in_func {
    DO_NOTHING =              0,  // rc input disabled
                                  // Enum values 1-9 saved for flight controls
    SW_FLIP =                10,  // flip
    SW_SIMPLE_MODE =         11,  // change to simple mode
    SW_RTL =                 12,  // change to RTL flight mode
    SW_SAVE_TRIM =           13,  // save current position as level
    SW_SAVE_WP =             14,  // save mission waypoint or RTL if in auto mode
    SW_CAMERA_TRIGGER =      15,  // trigger camera servo or relay
    SW_GROUND_RANGEFINDER =  16,  // allow enabling or disabling ground rangefinder in flight which helps avoid surface tracking when you are far above the ground
    SW_FENCE =               17,  // allow enabling or disabling fence in flight
    SW_RESETTOARMEDYAW =     18,  // changes yaw to be same as when quad was armed
    SW_SUPERSIMPLE_MODE =    19,  // change to simple mode in middle, super simple at top
    SW_ACRO_TRAINER =        20,  // low = disabled, middle = leveled, high = leveled and limited
    SW_SPRAYER =             21,  // enable/disable the crop sprayer
    SW_AUTO =                22,  // change to auto flight mode
    SW_AUTOTUNE =            23,  // auto tune
    SW_LAND =                24,  // change to LAND flight mode
    SW_EPM =                 25,  // Operate the EPM cargo gripper low=off, middle=neutral, high=on
    SW_PARACHUTE_ENABLE =    26,  // Parachute enable/disable
    SW_PARACHUTE_RELEASE =   27,  // Parachute release
    SW_PARACHUTE_3POS =      28,  // Parachute disable, enable, release with 3 position switch
    SW_MISSION_RESET =       29,  // Reset auto mission to start from first command
    SW_ATTCON_FEEDFWD =      30,  // enable/disable the roll and pitch rate feed forward
    SW_ATTCON_ACCEL_LIM =    31,  // enable/disable the roll, pitch and yaw accel limiting
    SW_RETRACT_MOUNT =       32,  // Retract Mount
    SW_RELAY =               33,  // Relay pin on/off (only supports first relay)
    SW_LANDING_GEAR =        34,  // Landing gear controller
    SW_MOTOR_ESTOP =         35,  // Emergency Stop Switch
    SW_MOTOR_INTERLOCK =     36,  // Motor On/Off switch
};

    /// roll - return input channel number for roll / aileron input
    uint8_t roll() const { return _ch1_function; }

    /// pitch - return input channel number for pitch / elevator input
    uint8_t pitch() const { return _ch2_function; }

    /// throttle - return input channel number for throttle input
    uint8_t throttle() const { return _ch3_function; }

    /// yaw - return input channel number for yaw / rudder input
    uint8_t yaw() const { return _ch4_function; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // channel mappings
    AP_Int8 _ch1_function;
    AP_Int8 _ch2_function;
    AP_Int8 _ch3_function;
    AP_Int8 _ch4_function;
    AP_Int8 _ch5_function;
    AP_Int8 _ch6_function;
    AP_Int8 _ch7_function;
    AP_Int8 _ch8_function;
    AP_Int8 _ch9_function;
    AP_Int8 _ch10_function;
    AP_Int8 _ch11_function;
    AP_Int8 _ch12_function;
};
#endif
