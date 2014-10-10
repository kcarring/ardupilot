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
    // @Values: 0:Disabled,1:Enabled
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

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of object detect.
    // @Units: Centi-Degrees
    // @Range: -9000 8999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  5, AP_ObjectDetect, _pan_angle_min,  -4500),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: -9000 8999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  6, AP_ObjectDetect, _pan_angle_max,  4500),
    
    // @Param: SWEEP_TILT
    // @DisplayName: Sweep object detect tilt angle
    // @Description: enable tilt/pitch sweep relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SWEEP_TILT", 7, AP_ObjectDetect, _sweep_tilt,  0),

    // @Param: SWEEP_PAN
    // @DisplayName: Sweep object detect pan angle
    // @Description: enable pan/yaw ssweep relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SWEEP_PAN",   8, AP_ObjectDetect, _sweep_pan,  0),
    
    // @Param: ANG_SWEEP_TILT
    // @DisplayName: Sweep tilt angle
    // @Description: Included angle to be swept in tilt
    // @Units: Centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGSW_TILT", 9, AP_ObjectDetect, _ang_sweep_tilt, 1500),
    
    // @Param: ANG_SWEEP_PAN
    // @DisplayName: Sweep pan angle
    // @Description: Included angle to be swept in pan
    // @Units: Centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGSW_PAN", 10, AP_ObjectDetect, _ang_sweep_pan, 1500),
    
    // @Param: SWEEP_HZ
    // @DisplayName: Sweep Frequency
    // @Description: Speed for a single sweep
    // @Units: Frequency
    // @Range: 0 400
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SWEEP_HZ", 11, AP_ObjectDetect, _sweep_hz, 1),
    
    AP_GROUPEND
};

AP_ObjectDetect::AP_ObjectDetect(const AP_AHRS &ahrs, RangeFinder &object_scanner) :
    _ahrs(ahrs),
    _object_scanner(object_scanner),
    _tilt_angle(0.0f),
    _pan_angle(0.0f),
    _tilt_sweep_reverse(false),
    _pan_sweep_reverse(false)
{
	AP_Param::setup_object_defaults(this, var_info);
}

// Initialize
void AP_ObjectDetect::init(float delta_sec)
{
    _dt = delta_sec;
    _tilt_sweep_increment = _ang_sweep_tilt * _dt *_sweep_hz * 2;
    _pan_sweep_increment = _ang_sweep_pan * _dt *_sweep_hz * 2;
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
    
    _tilt_angle = 0;
    
    if (_pan_sweep_reverse){
        _pan_angle -= _pan_sweep_increment;
    } else {
        _pan_angle += _pan_sweep_increment;
    }
    
    if (_pan_angle >= _ang_sweep_pan){
        _pan_sweep_reverse = true;
        reset_scanner_capture();
    }
    
    if (_pan_angle <= -_ang_sweep_pan){
        _pan_sweep_reverse = false;
        reset_scanner_capture();
    }
    
    RC_Channel_aux::move_servo(RC_Channel_aux::k_objectdetect_pan, _pan_angle, _pan_angle_min,  _pan_angle_max);
    RC_Channel_aux::move_servo(RC_Channel_aux::k_objectdetect_tilt, _tilt_angle, _tilt_angle_min, _tilt_angle_max);
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
