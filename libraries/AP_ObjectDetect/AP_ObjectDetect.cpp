// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_ObjectDetect.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

const AP_Param::GroupInfo AP_ObjectDetect::var_info[] PROGMEM = {

    // @Param: STAB_TILT
    // @DisplayName: Stabilize object detect's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled, -1:Enabled Reverse
    // @User: Standard
    AP_GROUPINFO("STAB_TILT", 1, AP_ObjectDetect, _stab_tilt,  0),

    // @Param: STAB_PAN
    // @DisplayName: Stabilize object detect pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_PAN",   2, AP_ObjectDetect, _stab_pan,  0),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of object detect.
    // @Units: Centi-Degrees
    // @Range: -9000 8999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 3, AP_ObjectDetect, _tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: -9000 8999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 4, AP_ObjectDetect, _tilt_angle_max, 4500),
    
    // @Param: TRIM_TILT
    // @DisplayName: Tilt Trim
    // @Description: Center position for tilt servo
    // @Units: PWM
    // @Range: 1250 1750
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM_TILT", 5, AP_ObjectDetect, _tilt_trim, 1500),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of object detect.
    // @Units: Centi-Degrees
    // @Range: -16500 0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  6, AP_ObjectDetect, _pan_angle_min,  -16500),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: 0 16500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  7, AP_ObjectDetect, _pan_angle_max,  16500),
    
    // @Param: TRIM_PAN
    // @DisplayName: Pan Trim
    // @Description: Center position for pan servo
    // @Units: PWM
    // @Range: 1250 1750
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRIM_PAN", 8, AP_ObjectDetect, _pan_trim, 1500),
    
    // @Param: SWEEP_TILT
    // @DisplayName: Sweep object detect tilt angle
    // @Description: enable tilt/pitch sweep relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SWEEP_TILT", 9, AP_ObjectDetect, _sweep_tilt,  0),

    // @Param: SWEEP_PAN
    // @DisplayName: Sweep object detect pan angle
    // @Description: enable pan/yaw ssweep relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SWEEP_PAN",   10, AP_ObjectDetect, _sweep_pan,  0),
    
    // @Param: ANG_SWEEP_TILT
    // @DisplayName: Sweep tilt angle
    // @Description: Included angle to be swept in tilt
    // @Units: Centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGSW_TILT", 11, AP_ObjectDetect, _ang_sweep_tilt, 1500),
    
    // @Param: ANG_SWEEP_PAN
    // @DisplayName: Sweep pan angle
    // @Description: Included angle to be swept in pan
    // @Units: Centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGSW_PAN", 12, AP_ObjectDetect, _ang_sweep_pan, 1500),
    
    // @Param: SWEEP_HZ
    // @DisplayName: Sweep Frequency
    // @Description: Speed for a single sweep
    // @Units: Frequency
    // @Range: 0 400
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SWEEP_HZ", 13, AP_ObjectDetect, _sweep_hz, 1),
    
    // @Param: BUFFER_DIST
    // @DisplayName: Buffer Distance
    // @Description: Minimum distance before copter stops
    // @Units: Centimeters
    // @Range: 0 2000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("BUFFER_DIST", 14, AP_ObjectDetect, _buffer_dist, 1000),
    
    // @Param: PROP_DECEL
    // @DisplayName: Proportional Decel
    // @Description: Strength the copter will use to stop
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("PROP_DECEL", 15, AP_ObjectDetect, _decel_p, 1),
    
    AP_GROUPEND
};

AP_ObjectDetect::AP_ObjectDetect(const AP_AHRS &ahrs, RangeFinder &object_scanner) :
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
void AP_ObjectDetect::init(float delta_sec)
{
    _dt = delta_sec;
    _sweep_increment = 3.141592654 * _dt *_sweep_hz * 2;
    _tilt_angle_pwm = ( _tilt_angle_max - _tilt_angle_min )/900;
    _pan_angle_pwm = ( _pan_angle_max - _pan_angle_min )/900;
}

/// This one should be called periodically
void AP_ObjectDetect::update_objectdetect_position()
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

    if (_sweep_value > 6.283185307){
        _sweep_value = 0.0f;
        reset_scanner_capture();
    }
    
    RC_Channel_aux::set_radio(RC_Channel_aux::k_objectdetect_pan, _pan_trim + (_pan_angle/_pan_angle_pwm));
    RC_Channel_aux::set_radio(RC_Channel_aux::k_objectdetect_tilt, _tilt_trim + (_tilt_angle/_tilt_angle_pwm));
}

// function to reset scanner distance capture.
void    AP_ObjectDetect::reset_scanner_capture()
{
    _last_object_distance = _object_distance;
    _object_distance = _scanner_reading;
}

// accessor to get the current distance measurement
uint16_t AP_ObjectDetect::get_object_distance()
{
    return min(_object_distance, _last_object_distance);
}

// accessor to get the current loiter correction
uint16_t    AP_ObjectDetect::get_loiter_decel()
{
    if (_enabled){
        int16_t approach_distance = _buffer_dist - get_object_distance();
        int32_t correction = approach_distance * _decel_p;
        correction = constrain_int32(correction, 0, 4500);
        return correction;
    } else {
        return 0;
    }
}
