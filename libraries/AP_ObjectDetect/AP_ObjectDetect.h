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
#include <RangeFinder.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>

class AP_ObjectDetect
{
public:
    //Constructor
    AP_ObjectDetect(const AP_AHRS &ahrs, RangeFinder &object_scanner);
    
    // initialization procedure.
    void    init(float delta_sec);

    // should be called periodically
    void    update_objectdetect_position();
    
    // accessor to get the current distance measurement
    uint16_t    get_object_distance();
    
    // accessor to get the current loiter correction
    uint16_t    get_loiter_decel();
    
    // setter to turn function on and off;
    void    set_enabled(bool true_false) { _enabled = true_false; }
    
    // enabled - returns true if object detector is enabled
    bool enabled() const { return _enabled; }

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

private:

    //members
    const AP_AHRS                   &_ahrs; ///< Rotation matrix from earth to plane.
    RangeFinder                     &_object_scanner; // Object Scanning Lidar or Sonar.
    
    bool                            _tilt_sweep_reverse;
    bool                            _pan_sweep_reverse;
    bool                            _enabled;
    
    float                           _tilt_angle; ///< degrees
    float                           _pan_angle;  ///< degrees
    float                           _dt;        // time step of loop
    float                           _tilt_sweep_increment;  // centi-degrees
    float                           _pan_sweep_increment;   // centi-degrees
    
    uint16_t                        _object_distance;       // closest object during current scan cycle
    uint16_t                        _last_object_distance;  // closest object during last scan cycle
    uint16_t                        _scanner_reading;       // current scanner reading
    
    // function to reset scanner distance capture
    void    reset_scanner_capture();

    // EEPROM parameters
    AP_Int8                         _stab_tilt; ///< (1 = yes, 0 = no)
    AP_Int8                         _stab_pan;  ///< (1 = yes, 0 = no)
    AP_Int8                         _sweep_tilt; ///< (1 = yes, 0 = no)
    AP_Int8                         _sweep_pan;  ///< (1 = yes, 0 = no)

    AP_Int16                        _tilt_angle_min; ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _tilt_angle_max; ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_min;  ///< min angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _pan_angle_max;  ///< max angle limit of actuated surface in 0.01 degree units
    AP_Int16                        _ang_sweep_tilt; // tilt angle to be swept
    AP_Int16                        _ang_sweep_pan;  // pan angle to be swept
    AP_Int16                        _sweep_hz;       // speed to sweep servo
    AP_Int16                        _buffer_dist;    // distance where the copter starts pulling back
    AP_Float                        _decel_p;        // proportional deceleration term
};

#endif // __AP_OBJECT_DETECT_H__
