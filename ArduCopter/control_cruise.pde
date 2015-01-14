/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_cruise.pde - init and run calls for cruise flight mode
 */

// cruise_init - initialise cruise controller
static bool cruise_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and accelerationj
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        return true;
    }else{
        return false;
    }
}

// cruise_run - runs the cruise controller
// should be called at 100hz or more
static void cruise_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || !inertial_nav.position_ok()) {
        wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {

        int16_t skid_correction;

        // Grab inertial velocity
        const Vector3f& vel = inertial_nav.get_velocity();

        // rotate roll, pitch input from north facing to vehicle's perspective
        // units are cm/sec
        float slip_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw();      // velocity at which copter is sliding sideways
        float forward_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw();    // forward velocity of copter

        // if forward velocity is small, do nothing, to avoid dithering side-to-side when in a stable hover
        if (fabs(forward_vel)>100){
            skid_correction = slip_vel * -g.cruise_slip_comp;
            skid_correction = constrain_int16(skid_correction, -4500, 4500);    // constrain correction within the bounds of normal roll input
        } else {
            skid_correction = 0;
        }

        // process pilot's pitch input, roll input is always zero.
        if (object_detect.enabled()){
            wp_nav.set_pilot_desired_acceleration(skid_correction, loiter_correction);
        } else {
            wp_nav.set_pilot_desired_acceleration(skid_correction, g.rc_2.control_in);
        }

        // get pilot's desired yaw rate, controlled by normal "roll" stick input.
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_1.control_in);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav.clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
    }else{
        // run loiter controller
        wp_nav.update_loiter();

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop

        // run altitude controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}
