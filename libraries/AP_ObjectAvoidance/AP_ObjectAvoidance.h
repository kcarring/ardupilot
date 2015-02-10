// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_OBJECTAVOIDANCE_H__
#define __AP_OBJECTAVOIDANCE_H__

// Dependencies
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_AHRS.h>
#include <AP_RangeFinder.h>     // Range finder library
#include <RC_Channel.h>         // RC Channel Library

#define OA_DISABLED 0
#define OA_ENABLED 1

#define SCANNER_STAB_TILT_DEFAULT       OA_DISABLED    // Default object scanner tilt stabilization function
#define SCANNER_STAB_PAN_DEFAULT        OA_DISABLED    // Default object scanner pan stabilization function
#define SCANNER_SWEEP_TILT_DEFUALT      OA_DISABLED    // Default object scanner tilt sweep function
#define SCANNER_SWEEP_PAN_DEFUALT       OA_DISABLED    // Default object scanner pan sweep function
#define SCANNER_SERVO_PWM_RANGE_DEFAULT 900         // Default object scanner servo range, 900 is typical for hobby servos moving 90 degrees9

#define BUFFER_DEFAULT                  2.0f        // Default safety buffer around copter
#define SCANNER_SWEEP_HZ_DEFAULT        1           // Default object scanner sweeping frequency

class AP_ObjectAvoidance
{
public:
    //Constructor
    AP_ObjectAvoidance(const AP_AHRS &ahrs, RangeFinder &object_scanner);

    // initialization procedure.
    void    init(float delta_sec);

    // should be called periodically
    void    update_objectscanner_position();

    // accessor to get the current distance measurement
    uint16_t    get_object_distance();

    // setter to turn function on and off;
    void    set_enabled(bool true_false) { _enabled = true_false; }

    // enabled - returns true if object scanner is enabled
    bool enabled() const { return _enabled; }

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:

    //members
    const AP_AHRS                   &_ahrs;                     // Rotation matrix from earth to plane.
    RangeFinder                     &_object_scanner;           // Object Scanning Lidar or Sonar.

    bool                            _sweep_phase;               // True if sweep_increment is between Pi and 2Pi

    float                           _dt;                        // time step of loop
    float                           _sweep_increment;           // radians
    float                           _tilt_angle_pwm_resolution; // resolution of tilt servo, degrees per PWM
    float                           _pan_angle_pwm_resolution;  // resolution of pan servo, degrees per PWM
    float                           _sweep_value;             // radians

    uint16_t                        _object_distance;           // closest object during current scan cycle
    uint16_t                        _last_object_distance;      // closest object during last scan cycle
    uint16_t                        _scanner_reading;           // current scanner reading

    // function to reset scanner distance capture
    void    reset_scanner_capture();

    // recalculate scalars
    void    calc_scalars();

    // EEPROM parameters
    AP_Int8                         _enabled;               // (1 = enabled, 0 = disabled)
    AP_Int8                         _stab_tilt;             // (1 = yes, 0 = no, -1 = reverse)
    AP_Int8                         _stab_pan;              // (1 = yes, 0 = no)

    AP_Float                        _tilt_angle_min;        // min angle limit of actuated surface in 0.01 degree units
    AP_Float                        _tilt_angle_max;        // max angle limit of actuated surface in 0.01 degree units
    AP_Float                        _tilt_angle_pwm_range;  // PWM range for Min-Max travel. Typical servos move 90 degrees over 900 PWM.
    AP_Int16                        _tilt_trim;             // trim value for tilt channel
    AP_Float                        _pan_angle_min;         // min angle limit of actuated surface in 0.01 degree units
    AP_Float                        _pan_angle_max;         // max angle limit of actuated surface in 0.01 degree units
    AP_Float                        _pan_angle_pwm_range;   // PWM range for Min-Max travel. Typical servos move 90 degrees over 900 PWM.
    AP_Int16                        _pan_trim;              // trim value for tilt channel
    AP_Float                        _sweep_tilt;            // tilt angle to be swept, <1 will disable sweep function
    AP_Float                        _sweep_pan;             // pan angle to be swept, <1 will disable sweep function
    AP_Int16                        _sweep_hz;              // speed to sweep servo

};

#endif // __AP_OBJECT_AVOIDANCE_H__
