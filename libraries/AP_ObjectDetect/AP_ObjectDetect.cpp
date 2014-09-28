// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_ObjectDetect.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

const AP_Param::GroupInfo AP_ObjectDetect::var_info[] PROGMEM = {
    
    // @Param: NEUTRAL_X
    // @DisplayName: ObjectDetect roll angle when in neutral position
    // @Description: ObjectDetect roll angle when in neutral position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard

    // @Param: NEUTRAL_Y
    // @DisplayName: ObjectDetect tilt/pitch angle when in neutral position
    // @Description: ObjectDetect tilt/pitch angle when in neutral position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard

    // @Param: NEUTRAL_Z
    // @DisplayName: ObjectDetect pan/yaw angle when in neutral position
    // @Description: ObjectDetect pan/yaw angle when in neutral position
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NEUTRAL",    2, AP_ObjectDetect, _neutral_angles, 0),

    // @Param: CONTROL_X
    // @DisplayName: ObjectDetect roll angle command from groundstation
    // @Description: ObjectDetect roll angle when in MavLink or RC control operation mode
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1

    // @Param: CONTROL_Y
    // @DisplayName: ObjectDetect tilt/pitch angle command from groundstation
    // @Description: ObjectDetect tilt/pitch angle when in MavLink or RC control operation mode
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1

    // @Param: CONTROL_Z
    // @DisplayName: ObjectDetect pan/yaw angle command from groundstation
    // @Description: ObjectDetect pan/yaw angle when in MavLink or RC control operation mode
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    AP_GROUPINFO("CONTROL",    3, AP_ObjectDetect, _control_angles, 0),

    // @Param: STAB_ROLL
    // @DisplayName: Stabilize object detect's roll angle
    // @Description: enable roll stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_ROLL",  4, AP_ObjectDetect, _stab_roll, 0),

    // @Param: STAB_TILT
    // @DisplayName: Stabilize object detect's pitch/tilt angle
    // @Description: enable tilt/pitch stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_TILT", 5, AP_ObjectDetect, _stab_tilt,  0),

    // @Param: STAB_PAN
    // @DisplayName: Stabilize object detect pan/yaw angle
    // @Description: enable pan/yaw stabilisation relative to Earth
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("STAB_PAN",   6, AP_ObjectDetect, _stab_pan,  0),

    // @Param: RC_IN_ROLL
    // @DisplayName: roll RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control roll movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_ROLL",  7, AP_ObjectDetect, _roll_rc_in, 0),

    // @Param: RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_TILT",  10, AP_ObjectDetect, _tilt_rc_in,    0),

    // @Param: ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of object detect.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_TIL", 11, AP_ObjectDetect, _tilt_angle_min, -4500),

    // @Param: ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_TIL", 12, AP_ObjectDetect, _tilt_angle_max, 4500),

    // @Param: RC_IN_PAN
    // @DisplayName: pan (yaw) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control pan (yaw) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8
    // @User: Standard
    AP_GROUPINFO("RC_IN_PAN",  13, AP_ObjectDetect, _pan_rc_in,       0),

    // @Param: ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of object detect.
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMIN_PAN",  14, AP_ObjectDetect, _pan_angle_min,  -4500),

    // @Param: ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the object detect
    // @Units: Centi-Degrees
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGMAX_PAN",  15, AP_ObjectDetect, _pan_angle_max,  4500),

    AP_GROUPEND
};

AP_ObjectDetect::AP_ObjectDetect(const AP_AHRS &ahrs) :
    _ahrs(ahrs),
    _tilt_control_angle(0.0f),
    _pan_control_angle(0.0f),
    _tilt_angle(0.0f),
    _pan_angle(0.0f)
{
	AP_Param::setup_object_defaults(this, var_info);

    // default to zero angles
    _neutral_angles = Vector3f(0,0,0);
    _control_angles = Vector3f(0,0,0);

    // default unknown object detect type
    _objectdetect_type = k_unknown;
    _tilt_idx = RC_Channel_aux::k_objectdetect_tilt;
    _pan_idx  = RC_Channel_aux::k_objectdetect_pan;
}

/// sets the servo angles for MAVLink, note angles are in degrees
void AP_ObjectDetect::set_control_angles(float roll, float tilt, float pan)
{
    _control_angles = Vector3f(roll, tilt, pan);
}

/// This one should be called periodically
void AP_ObjectDetect::update_objectdetect_position()
{    
    #define rc_ch(i) RC_Channel::rc_channel(i-1)
    // allow pilot position input to come directly from an RC_Channel
    if (_tilt_rc_in && (rc_ch(_tilt_rc_in))) {
        _tilt_control_angle = angle_input_rad(rc_ch(_tilt_rc_in), _tilt_angle_min, _tilt_angle_max);
    }
    if (_pan_rc_in && (rc_ch(_pan_rc_in))) {
        _pan_control_angle = angle_input_rad(rc_ch(_pan_rc_in), _pan_angle_min, _pan_angle_max);
    }

    // write the results to the servos
    move_servo(_tilt_idx, _tilt_angle*10, _tilt_angle_min*0.1f, _tilt_angle_max*0.1f);
    move_servo(_pan_idx,  _pan_angle*10,  _pan_angle_min*0.1f,  _pan_angle_max*0.1f);
}

/// returns the angle (degrees*100) that the RC_Channel input is receiving
int32_t
AP_ObjectDetect::angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return (rc->get_reverse() ? -1 : 1) * (rc->radio_in - rc->radio_min) * (int32_t)(angle_max - angle_min) / (rc->radio_max - rc->radio_min) + (rc->get_reverse() ? angle_max : angle_min);
}

/// returns the angle (radians) that the RC_Channel input is receiving
float
AP_ObjectDetect::angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(angle_input(rc, angle_min, angle_max)*0.01f);
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

/// all angles are degrees * 10 units
void
AP_ObjectDetect::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	// saturate to the closest angle limit if outside of [min max] angle interval
	int16_t servo_out = closest_limit(angle, &angle_min, &angle_max);
	RC_Channel_aux::move_servo((RC_Channel_aux::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
}
