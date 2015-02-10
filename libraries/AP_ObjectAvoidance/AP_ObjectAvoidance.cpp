// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_ObjectAvoidance.h>

const AP_Param::GroupInfo AP_ObjectAvoidance::var_info[] PROGMEM = {

    // @Param: STAB_TILT
    // @DisplayName: Stabilize object scanner's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled, -1:Enabled Reverse
    // @User: Standard
    AP_GROUPINFO("STAB_TILT", 1, AP_ObjectAvoidance, _stab_tilt,  SCANNER_STAB_TILT_DEFAULT),

    // @Param: STAB_PAN
    // @DisplayName: Stabilize object scanner pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_PAN",   2, AP_ObjectAvoidance, _stab_pan,  SCANNER_STAB_PAN_DEFAULT),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of object scanner.
    // @Units: Degrees
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 3, AP_ObjectAvoidance, _tilt_angle_min, -45.0f),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the object scanner
    // @Units: Degrees
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 4, AP_ObjectAvoidance, _tilt_angle_max, 45.0f),

    // @Param: TILT_PWM_RNG
    // @DisplayName: Tilt PWM Range
    // @Description: PWM Range for Object Scanner Tilt Servo
    // @Units: PWM
    // @Range: 0 1100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TILT_PWM_RNG", 5, AP_ObjectAvoidance, _tilt_angle_pwm, SCANNER_SERVO_PWM_RANGE_DEFAULT),

    // @Param: TRIM_TILT
    // @DisplayName: Tilt Trim
    // @Description: Center position for tilt servo
    // @Units: PWM
    // @Range: 1250 1750
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM_TILT", 6, AP_ObjectAvoidance, _tilt_trim, 1500),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of object scanner.
    // @Units: Degrees
    // @Range: -180 0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  7, AP_ObjectAvoidance, _pan_angle_min,  -45.0f),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the object scanner
    // @Units: Degrees
    // @Range: 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  8, AP_ObjectAvoidance, _pan_angle_max,  45.0f),

    // @Param: PAN_PWM_RNG
    // @DisplayName: Pan PWM Range
    // @Description: PWM Range for Object Scanner Pan Servo
    // @Units: PWM
    // @Range: 0 1100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PAN_PWM_RNG", 9, AP_ObjectAvoidance, _pan_angle_pwm, SCANNER_SERVO_PWM_RANGE_DEFAULT),

    // @Param: TRIM_PAN
    // @DisplayName: Pan Trim
    // @Description: Center position for pan servo
    // @Units: PWM
    // @Range: 1250 1750
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM_PAN", 10, AP_ObjectAvoidance, _pan_trim, 1500),

    // @Param: SWEEP_TILT
    // @DisplayName: Sweep object scanner tilt angle
    // @Description: enable tilt/pitch sweep relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SWEEP_TILT", 11, AP_ObjectAvoidance, _sweep_tilt,  SCANNER_SWEEP_TILT_DEFUALT),

    // @Param: SWEEP_PAN
    // @DisplayName: Sweep object scanner pan angle
    // @Description: enable pan/yaw ssweep relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SWEEP_PAN",   12, AP_ObjectAvoidance, _sweep_pan,  SCANNER_SWEEP_PAN_DEFUALT),

    // @Param: ANG_SWEEP_TILT
    // @DisplayName: Sweep tilt angle
    // @Description: Included angle to be swept in tilt
    // @Units: Degrees
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGSW_TILT", 13, AP_ObjectAvoidance, _ang_sweep_tilt, 15),

    // @Param: ANG_SWEEP_PAN
    // @DisplayName: Sweep pan angle
    // @Description: Included angle to be swept in pan
    // @Units: Degrees
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGSW_PAN", 14, AP_ObjectAvoidance, _ang_sweep_pan, 15),

    // @Param: SWEEP_HZ
    // @DisplayName: Sweep Frequency
    // @Description: Speed for a single sweep
    // @Units: Frequency
    // @Range: 0 400
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SWEEP_HZ", 15, AP_ObjectAvoidance, _sweep_hz, SCANNER_SWEEP_HZ_DEFAULT),

    // @Param: BUFFER_DIST
    // @DisplayName: Buffer Distance
    // @Description: Safety buffer around Copter
    // @Units: Meters
    // @Range: 0 20
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("BUFFER_DIST", 16, AP_ObjectAvoidance, _buffer_dist, BUFFER_DEFAULT),

    AP_GROUPEND
};

AP_ObjectAvoidance::AP_ObjectAvoidance(const AP_AHRS &ahrs, RangeFinder &object_scanner) :
    _ahrs(ahrs),
    _object_scanner(object_scanner),
    _tilt_angle(0.0f),
    _pan_angle(0.0f),
    _sweep_value(0.0f),
    _enabled(false)
{
	AP_Param::setup_object_defaults(this, var_info);
}

// Initialize
void AP_ObjectAvoidance::calc_scalars(float delta_sec)
{
    _dt = delta_sec;
    _sweep_increment = 2*M_PI * _dt *_sweep_hz * 2;
    _tilt_angle_pwm = ( _tilt_angle_max - _tilt_angle_min )/_tilt_angle_pwm;
    _pan_angle_pwm = ( _pan_angle_max - _pan_angle_min )/_pan_angle_pwm;
}

/// This one should be called periodically
void AP_ObjectAvoidance::update_objectscanner_position()
{
    _object_scanner.update();

    // exit immediately if scanner is disabled
    if (!_object_scanner.healthy()) {
        return;
    }

    _scanner_reading = _object_scanner.distance_cm();

    if (_scanner_reading < _object_distance){
        _object_distance = _scanner_reading;
    }

    _sweep_value += _sweep_increment;

    if (!_phase && _sweep_value > M_PI){
        _phase = true;
        reset_scanner_capture();
    }
    if (_sweep_value > 2*M_PI){
        _sweep_value = 0.0f;
        _phase = false;
        reset_scanner_capture();
    }

    // center servos and exit if scanning is not active.
    if (!_enabled){
        RC_Channel_aux::set_radio(RC_Channel_aux::k_objectscanner_pan, _pan_trim);
        RC_Channel_aux::set_radio(RC_Channel_aux::k_objectscanner_tilt, _tilt_trim);
        return;
    }

    if (_sweep_tilt){
        _tilt_angle = _ang_sweep_tilt * cos(_sweep_value);
    } else {
        _tilt_angle = 0;
    }

    _tilt_angle += _stab_tilt * (degrees(_ahrs.pitch))*100;

    if (_sweep_pan){
        _pan_angle = _ang_sweep_pan * sin(_sweep_value);
    } else {
        _pan_angle = 0;
    }

    RC_Channel_aux::set_radio(RC_Channel_aux::k_objectscnner_pan, _pan_trim + (_pan_angle/_pan_angle_pwm));
    RC_Channel_aux::set_radio(RC_Channel_aux::k_objectscanner_tilt, _tilt_trim + (_tilt_angle/_tilt_angle_pwm));
}

// function to reset scanner distance capture.
void    AP_ObjectAvoidance::reset_scanner_capture()
{
    _last_object_distance = _object_distance;
    _object_distance = _scanner_reading;
}

// accessor to get the current distance measurement
uint16_t AP_ObjectAvoidance::get_object_distance()
{
    return min(_object_distance, _last_object_distance);
}
