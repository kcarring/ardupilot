// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_ObjectScanner.h>

const AP_Param::GroupInfo AP_ObjectScanner::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: Object Scanner Enable
    // @Description: Enable object scanner
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLED", 0, AP_ObjectScanner, _enabled,  OS_DISABLED),

    // @Param: STAB_TILT
    // @DisplayName: Object Scanner Stabilize Tilt Angle
    // @Description: Enable tilt stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled, -1:Enabled Reverse
    // @User: Standard
    AP_GROUPINFO("STAB_TILT", 1, AP_ObjectScanner, _stab_tilt,  SCANNER_STAB_TILT_DEFAULT),

    // @Param: TARG_PAN
    // @DisplayName: Object Scanner Target Pan Angle
    // @Description: Enable pan targeting relative to flight path
    // @Values: 0:Disabled,1:Enabled, -1:Enabled Reverse
    // @User: Standard
    AP_GROUPINFO("TARG_PAN",   2, AP_ObjectScanner, _target_pan,  SCANNER_TARG_PAN_DEFAULT),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt angular position of object scanner.
    // @Units: Degrees
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 3, AP_ObjectScanner, _tilt_angle_min, -45.0f),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt angular position of the object scanner
    // @Units: Degrees
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 4, AP_ObjectScanner, _tilt_angle_max, 45.0f),

    // @Param: TILT_PWM_RNG
    // @DisplayName: Tilt PWM Range
    // @Description: PWM Range for Object Scanner Tilt Servo
    // @Units: PWM
    // @Range: 0 1100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TILT_PWM_RNG", 5, AP_ObjectScanner, _tilt_angle_pwm_range, SCANNER_SERVO_PWM_RANGE_DEFAULT),

    // @Param: TRIM_TILT
    // @DisplayName: Tilt Trim
    // @Description: Center position for tilt servo
    // @Units: PWM
    // @Range: 1250 1750
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM_TILT", 6, AP_ObjectScanner, _tilt_trim, 1500),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan angular position of object scanner
    // @Units: Degrees
    // @Range: -180 0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  7, AP_ObjectScanner, _pan_angle_min,  -45.0f),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan angular position of the object scanner
    // @Units: Degrees
    // @Range: 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  8, AP_ObjectScanner, _pan_angle_max,  45.0f),

    // @Param: PAN_PWM_RNG
    // @DisplayName: Pan PWM Range
    // @Description: PWM Range for Object Scanner Pan Servo
    // @Units: PWM
    // @Range: 0 1100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PAN_PWM_RNG", 9, AP_ObjectScanner, _pan_angle_pwm_range, SCANNER_SERVO_PWM_RANGE_DEFAULT),

    // @Param: TRIM_PAN
    // @DisplayName: Pan Trim
    // @Description: Center position for pan servo
    // @Units: PWM
    // @Range: 1250 1750
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM_PAN", 10, AP_ObjectScanner, _pan_trim, 1500),

    // @Param: SWEEP_HZ
    // @DisplayName: Sweep Frequency
    // @Description: Scans per second
    // @Units: Frequency
    // @Range: 0 400
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SWEEP_HZ", 11, AP_ObjectScanner, _sweep_hz, SCANNER_SWEEP_HZ_DEFAULT),

    // @Param: SWEEP_TILT
    // @DisplayName: Sweep scanner tilt angle
    // @Description: Angle to be swept in tilt, must be >1 to enable
    // @Units: Degrees
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SWEEP_TILT", 12, AP_ObjectScanner, _sweep_tilt, OS_DISABLED),

    // @Param: SWEEP_PAN
    // @DisplayName: Sweep scanner pan angle
    // @Description: Angle to be swept in pan, must be >1 to enable
    // @Units: Degrees
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SWEEP_PAN", 13, AP_ObjectScanner, _sweep_pan, OS_DISABLED),

    AP_GROUPEND
};

AP_ObjectScanner::AP_ObjectScanner(const AP_AHRS &ahrs, RangeFinder &object_rangefinder) :
    _ahrs(ahrs),
    _object_scanner(object_rangefinder),
    _dt(SCANNER_SWEEP_DT_DEFAULT),
    _sweep_phase(0),
    _sweep_value(0),
    _sweep_increment(0),
    _tilt_angle_pwm_resolution(SCANNER_SERVO_PWM_RESOLUTION_DEFAULT),
    _pan_angle_pwm_resolution(SCANNER_SERVO_PWM_RESOLUTION_DEFAULT),
    _object_distance(0),
    _last_object_distance(0),
    _scanner_reading(0)
{
	AP_Param::setup_object_defaults(this, var_info);
}

// Initialize
void AP_ObjectScanner::init(float delta_sec)
{
    _dt = delta_sec;
    calc_scalars();
}

// Initialize
void AP_ObjectScanner::calc_scalars()
{
    // avoid divide by zero
    if (_tilt_angle_pwm_range == 0){
        _tilt_angle_pwm_range = SCANNER_SERVO_PWM_RANGE_DEFAULT;
    }
    if (_pan_angle_pwm_range == 0){
        _pan_angle_pwm_range = SCANNER_SERVO_PWM_RANGE_DEFAULT;
    }

    _sweep_increment = 2*M_PI * _dt*_sweep_hz;
    _tilt_angle_pwm_resolution = ( _tilt_angle_max - _tilt_angle_min )/_tilt_angle_pwm_range;
    _pan_angle_pwm_resolution = ( _pan_angle_max - _pan_angle_min )/_pan_angle_pwm_range;
}

/// This one should be called periodically
void AP_ObjectScanner::update_objectscanner_position()
{

    float           tilt_sweep_angle;          // sweep action tilt angle
    float           pan_sweep_angle;           // sweep action pan angle
    float           tilt_stab_angle;           // stabilize tilt angle
    float           pan_targ_angle;            // stabilize pan angle

    _object_scanner.update();

    // exit immediately if scanner is disabled
    // if (!_object_scanner.healthy()) {
    //    return;
    // }

    _scanner_reading = _object_scanner.distance_cm();

    if (_scanner_reading < _object_distance){
        _object_distance = _scanner_reading;
    }

    _sweep_value += _sweep_increment;

    // Divide scan cycle in half
    // after 2Pi cycle is complete, reset it
    if (!_sweep_phase && _sweep_value > M_PI){
        _sweep_phase = true;
        reset_scanner_capture();
    }
    if (_sweep_value > 2*M_PI){
        _sweep_value = 0.0f;
        _sweep_phase = false;
        reset_scanner_capture();
    }

    if (_sweep_tilt > 1.0){
        tilt_sweep_angle = _sweep_tilt * cos(_sweep_value);
    } else {
        tilt_sweep_angle = 0;
    }

    // _stab_tilt parameter enables tilt stabilization if not zero, and serves as servo reversing if negative.
    tilt_stab_angle = _stab_tilt * degrees(_ahrs.pitch);

    if (_sweep_pan > 1.0){
        pan_sweep_angle = _sweep_pan * sin(_sweep_value);
    } else {
        pan_sweep_angle = 0;
    }

    // To-Do: figure out how to manage pan stab angle
    // intent is to use it to look in direction of movement.
    pan_targ_angle = 0;

    // avoid divide by zero, 0.1 would be a standard value for a 90° servo on 900 PWM
    if (_tilt_angle_pwm_resolution == 0){
        _tilt_angle_pwm_resolution = SCANNER_SERVO_PWM_RESOLUTION_DEFAULT;
    }
    if (_pan_angle_pwm_resolution == 0){
        _pan_angle_pwm_resolution = SCANNER_SERVO_PWM_RESOLUTION_DEFAULT;
    }

    float tilt_total_angle = (tilt_sweep_angle + tilt_stab_angle);
    tilt_total_angle = constrain_float(tilt_total_angle, _tilt_angle_min, _tilt_angle_max);
    float tilt_total_pwm = _tilt_trim + tilt_total_angle/_tilt_angle_pwm_resolution;
    RC_Channel_aux::set_radio(RC_Channel_aux::k_objectscanner_tilt, tilt_total_pwm);

    float pan_total_angle = (pan_sweep_angle + pan_targ_angle);
    pan_total_angle = constrain_float(pan_total_angle, _pan_angle_min, _pan_angle_max);
    float pan_total_pwm = _pan_trim + pan_total_angle/_pan_angle_pwm_resolution;
    RC_Channel_aux::set_radio(RC_Channel_aux::k_objectscanner_pan, pan_total_pwm);
}

// function to reset scanner distance capture.
void    AP_ObjectScanner::reset_scanner_capture()
{
    _last_object_distance = _object_distance;
    _object_distance = _scanner_reading;
}

// accessor to get the current distance measurement
uint16_t AP_ObjectScanner::get_object_distance()
{
    return min(_object_distance, _last_object_distance);
}
