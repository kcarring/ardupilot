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

    // should be called periodically
    void                    update_objectdetect_position();

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:

    // internal methods
    int16_t                         closest_limit(int16_t angle, int16_t* angle_min, int16_t* angle_max);
    void                            move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    //members
    const AP_AHRS                   &_ahrs; ///< Rotation matrix from earth to plane.

    uint8_t                         _tilt_idx; ///< RC_Channel_aux objectdetect tilt function index
    uint8_t                         _pan_idx;  ///< RC_Channel_aux objectdetect pan  function index

    float                           _tilt_angle; ///< degrees
    float                           _pan_angle;  ///< degrees

    // EEPROM parameters
    AP_Int8                         _stab_tilt; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_pan;  ///< (1 = yes, 0 = no)

    AP_Int16                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_min;  ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_max;  ///< max angle limit of actuated surface in 0.01 degree units
};

#endif // __AP_OBJECT_DETECT_H__
