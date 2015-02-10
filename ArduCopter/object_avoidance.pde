/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_ObjectAvoidance library with main ArduCopter code

// update_object_avoidance - update object avoidance system
// should be run at 50hz
static void update_object_avoidance()
{
#if OBJECTAVOIDANCE == ENABLED
    // update object_avoidance mount's position
    object_avoidance.update_objectscanner_position();
#endif
}