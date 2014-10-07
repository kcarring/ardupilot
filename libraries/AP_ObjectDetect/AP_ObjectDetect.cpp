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
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 3, AP_ObjectDetect, _tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 4, AP_ObjectDetect, _tilt_angle_max, 4500),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of object detect.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  5, AP_ObjectDetect, _pan_angle_min,  -4500),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  6, AP_ObjectDetect, _pan_angle_max,  4500),

    AP_GROUPEND
};

AP_ObjectDetect::AP_ObjectDetect(const AP_AHRS &ahrs) :
    _ahrs(ahrs),
    _tilt_angle(0.0f),
    _pan_angle(0.0f)
{
	AP_Param::setup_object_defaults(this, var_info);

    _tilt_idx = RC_Channel_aux::k_objectdetect_tilt;
    _pan_idx  = RC_Channel_aux::k_objectdetect_pan;
}

/// This one should be called periodically
void AP_ObjectDetect::update_objectdetect_position()
{    
    _tilt_angle = 0;
    _pan_angle = 0;

    // write the results to the servos
    move_servo(_tilt_idx, _tilt_angle*10, _tilt_angle_min*0.1f, _tilt_angle_max*0.1f);
    move_servo(_pan_idx,  _pan_angle*10,  _pan_angle_min*0.1f,  _pan_angle_max*0.1f);
}

/// all angles are degrees * 10 units
void
AP_ObjectDetect::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	// saturate to the closest angle limit if outside of [min max] angle interval
	int16_t servo_out = closest_limit(angle, &angle_min, &angle_max);
	RC_Channel_aux::move_servo((RC_Channel_aux::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
}

/// saturate to the closest angle limit if outside of [min max] angle interval
/// input angle is in degrees * 10
int16_t
AP_ObjectDetect::closest_limit(int16_t angle, int16_t* angle_min, int16_t* angle_max)
{
    // Make sure the angle lies in the interval [-180 .. 180[ degrees
    while (angle < -1800) angle += 3600;
    while (angle >= 1800) angle -= 3600;

    // Make sure the angle limits lie in the interval [-180 .. 180[ degrees
    while (*angle_min < -1800) *angle_min += 3600;
    while (*angle_min >= 1800) *angle_min -= 3600;
    while (*angle_max < -1800) *angle_max += 3600;
    while (*angle_max >= 1800) *angle_max -= 3600;


    // If the angle is outside servo limits, saturate the angle to the closest limit
    // On a circle the closest angular position must be carefully calculated to account for wrap-around
    if ((angle < *angle_min) && (angle > *angle_max)) {
        // angle error if min limit is used
        int16_t err_min = *angle_min - angle + (angle<*angle_min ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        // angle error if max limit is used
        int16_t err_max = angle - *angle_max + (angle>*angle_max ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        angle = err_min<err_max ? *angle_min : *angle_max;
    }

    return angle;
}
