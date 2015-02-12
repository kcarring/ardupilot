/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_ObjectAvoidance library with main ArduCopter code

// update_object_scanner - update object scanner system
// should be run at 50hz
static void update_object_scanner(){
#if OBJECTSCANNER == ENABLED
    // update object scanner mount's position
    object_scanner.update_objectscanner_position();
#endif
}

#if OBJECTSCANNER == ENABLED
static void init_object_scanner(void)
{
    // Initilize the detector at 0.02 seconds because it is running at 50Hz in the scheduler
    object_scanner.init(0.02);
}
#endif

#if OBJECTAVOIDANCE == ENABLED
static void init_object_avoidance(void)
{
    // Initilize the object avoidance algorithm to run at the same rate as Position Controller
    object_avoidance.init(MAIN_LOOP_SECONDS);
}
#endif