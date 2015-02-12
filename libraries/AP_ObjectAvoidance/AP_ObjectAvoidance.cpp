// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_ObjectAvoidance.h>

const AP_Param::GroupInfo AP_ObjectAvoidance::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: Object Avoidance Enable
    // @Description: Enable object avoidance program
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLED", 0, AP_ObjectAvoidance, _enabled,  OA_ENABLE_TYPE_NONE),

    // @Param: BUFFER
    // @DisplayName: Object Avoidance Buffer Distance
    // @Description: Safety margin autopilot will maintain from obstacles and fences
    // @Units: Meters
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("BUFFER", 1, AP_ObjectAvoidance, _buffer_distance,  OA_BUFFER_DEFAULT),

    AP_GROUPEND
};

AP_ObjectAvoidance::AP_ObjectAvoidance(const AP_ObjectScanner &object_scanner, const AC_Fence &fence, const AP_InertialNav& inav) :
    _object_scanner(object_scanner),
    _fence(fence),
    _inav(inav),
    _fence_enabled(OA_DISABLED),
    _scanner_enabled(OA_DISABLED),
    _enabled_fences(OA_FENCE_TYPE_ALT_MAX | OA_FENCE_TYPE_CIRCLE),
    _fence_radius(OA_FENCE_CIRCLE_RADIUS_DEFAULT)
{
	AP_Param::setup_object_defaults(this, var_info);
}

// Initialize
void AP_ObjectAvoidance::init(float delta_sec)
{
    _dt = delta_sec;
    _enabled_fences = _fence.get_enabled_fences();
    _fence_radius = _fence.get_circle_radius();

    _scanner_enabled = _object_scanner.enabled();
    _scanner_max_distance = _object_scanner.get_scanner_max_distance();
}

// Determine if new desired position needs to be constrained
// Takes a target position, and checks if it violates a fence or object
// Returns target position untouched, or modified to avoid the fence or object
void AP_ObjectAvoidance::target_position_clearance_xy(Vector3f& pos_target) const
{
    const Vector3f& current_position = _inav.get_position();

}
