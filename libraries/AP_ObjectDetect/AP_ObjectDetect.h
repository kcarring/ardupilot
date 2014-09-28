// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/************************************************************
* AP_ObjectDetect -- library to control a 2 axis mount.     *
*															*
* Author:  Robert Lefebvre
*          Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
*		   Gregory Fletcher;								*
*															*
* Purpose:  Move a 2 axis mount attached to vehicle,	    *
*			Used for mount to scan a Lidar.					*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
* Comments: All angles in degrees * 100, distances in meters*
*			unless otherwise stated.						*
************************************************************/
#ifndef __AP_OBJECTDETECT_H__
#define __AP_OBJECTDETECT_H__

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>

class AP_ObjectDetect
{
public:
    //Constructor
    AP_ObjectDetect(const AP_AHRS &ahrs);

    //enums
    enum ObjectDetectType {
        k_unknown = 0,                  ///< unknown type
        k_pan_tilt = 1,                 ///< yaw-pitch
        k_tilt_roll = 2,                ///< pitch-roll
    };

    // set_mode_to_default - restores the mode to it's default held in the MNT_MODE parameter
    //      this operation requires 230us on an APM2, 60us on a Pixhawk/PX4
    void                    set_mode_to_default() { _objectdetect_mode.load(); }

    // should be called periodically
    void                    update_objectdetect_position();

    // Accessors
    enum ObjectDetectType          get_objectdetect_type() {
        return _objectdetect_type;
    }
    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:

    //methods
    void                            set_control_angles(float roll, float tilt, float pan);

    // internal methods
    void                            stabilize();
    int16_t                         closest_limit(int16_t angle, int16_t* angle_min, int16_t* angle_max);
    void                            move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);
    int32_t                         angle_input(RC_Channel* rc, int16_t angle_min, int16_t angle_max);
    float                           angle_input_rad(RC_Channel* rc, int16_t angle_min, int16_t angle_max);

    //members
    const AP_AHRS                   &_ahrs; ///< Rotation matrix from earth to plane.
    ObjectDetectType                _objectdetect_type;

    uint8_t                         _tilt_idx; ///< RC_Channel_aux object detect tilt function index
    uint8_t                         _pan_idx;  ///< RC_Channel_aux object detect pan  function index

    float                           _tilt_control_angle; ///< radians
    float                           _pan_control_angle;  ///< radians

    float                           _tilt_angle; ///< degrees
    float                           _pan_angle;  ///< degrees

    // EEPROM parameters
    AP_Int8                         _stab_roll; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_tilt; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_pan;  ///< (1 = yes, 0 = no)
    AP_Int8                         _objectdetect_mode;

    // RC_Channel for providing direct angular input from pilot
    AP_Int8                         _roll_rc_in;
    AP_Int8                         _tilt_rc_in;
    AP_Int8                         _pan_rc_in;

    AP_Int16                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_min;  ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_max;  ///< max angle limit of actuated surface in 0.01 degree units

    AP_Vector3f                     _neutral_angles; ///< neutral position for object detect, vector.x = roll vector.y = tilt, vector.z=pan
    AP_Vector3f                     _control_angles; ///< GCS controlled position for object detect, vector.x = roll vector.y = tilt, vector.z=pan
};

#endif // __AP_OBJECT_DETECT_H__
