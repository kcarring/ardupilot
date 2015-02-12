// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_OBJECTAVOIDANCE_H__
#define __AP_OBJECTAVOIDANCE_H__

// Dependencies
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_AHRS.h>
#include <AP_ObjectScanner.h>
#include <AC_Fence.h>

#define OA_DISABLED 0
#define OA_ENABLED  1

#define OA_BUFFER_DEFAULT                           2.0f    // default distance in meters that autopilot should maintain from fence and objects

// bit masks for enabled object avoidance functions
#define OA_ENABLE_TYPE_NONE                         0       // no object avoidance
#define OA_ENABLE_TYPE_FENCE                        1       // avoid fence objects
#define OA_ENABLE_TYPE_SCANNER                      2       // avoid scanner objects

// bit masks for enabled fence types.  Copied from AC_Fence.h
#define OA_FENCE_TYPE_NONE                          0       // fence disabled
#define OA_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL
#define OA_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)
#define OA_FENCE_CIRCLE_RADIUS_DEFAULT              300.0f  // default circular fence radius is 300m

class AP_ObjectAvoidance
{
public:
    //Constructor
    AP_ObjectAvoidance(const AP_ObjectScanner &object_scanner, const AC_Fence &fence, const AP_InertialNav& inav);

    // initialization procedure.
    void    init(float delta_sec);

    // setter to turn function on and off;
    void    set_enabled(bool true_false) { _enabled = true_false; }

    // enabled - returns true if object avoidance is enabled
    bool enabled() const { return _enabled; }

    // hook for eeprom variables
    static const struct AP_Param::GroupInfo        var_info[];

    // Determine if new desired position needs to be constrained
    // Takes a target position, and checks if it violates a fence or object
    // Returns target position untouched, or modified to avoid the fence or object
    void target_position_clearance_xy(Vector3f& pos_target) const;

private:

    //members
    const AP_ObjectScanner          &_object_scanner;           // object detecting sanning rangefinder
    const AC_Fence                  &_fence;                    // geo-fence
    const AP_InertialNav            &_inav;                     

    uint8_t                         _enabled_fences;            // which type of fences are enabled (ie: altitude and/or circle)

    int16_t                         _scanner_max_distance;      // maximum range of object scanner

    float                           _dt;                        // time step of loop
    float                           _fence_radius;              // fence radius in meters

    bool                            _fence_enabled;             // status of fence enabled
    bool                            _scanner_enabled;           // status of object scanner enabled

    // EEPROM parameters
    AP_Int8                         _enabled;                   // (1 = enabled, 0 = disabled)
    AP_Float                        _buffer_distance;           // distance to maintain from fence and objects
};

#endif // __AP_OBJECTAVOIDANCE_H__
