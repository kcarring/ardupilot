/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_ObjectAvoidance library with main ArduCopter code

// update_object_avoidance - update object avoidance system
// should be run at 50hz
static void update_object_avoidance(){
#if OBJECTAVOIDANCE == ENABLED
    // update object_avoidance mount's position
    object_avoidance.update_objectscanner_position();
#endif
}

#if OBJECTAVOIDANCE == ENABLED
static void init_object_avoidance(void)
{
    // Initilize the detector at 0.02 seconds because it is running at 50Hz in the scheduler
    object_avoidance.init(0.02);
}
#endif