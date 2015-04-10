/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * tuning.pde - function to update various parameters in flight using a tuning knob
 *      This should not be confused with the AutoTune feature which can be found in control_autotune.pde
 */

// tuning - updates parameters based on the tuning knob's position
// should be called at 3.3hz
static void tuning() {

    // exit immediately if not tuning or when radio failsafe is invoked so tuning values are not set to zero
    if ((rcin.get_num_tuning_channels() <= 0) || failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    // run available tuning functions
    for (uint8_t index=0; rcin.get_num_tuning_channels() >= index+1; index++){
        do_tuning(rcin.get_tuning_function(index), rcin.get_tuning_value(index));
    }
}

// do_tuning - updates parameters based on tuning knob position
static void do_tuning(uint16_t tuning_function, float tuning_value) {

    switch(tuning_function) {

        case TUNING_STABILIZE_ROLL_PITCH_KP:
            // Roll, Pitch tuning
            g.p_stabilize_roll.kP(tuning_value);
            g.p_stabilize_pitch.kP(tuning_value);
            break;

        case TUNING_STABILIZE_YAW_KP:
            // Yaw tuning
            g.p_stabilize_yaw.kP(tuning_value);
            break;

        case TUNING_RATE_ROLL_PITCH_KP:
            g.pid_rate_roll.kP(tuning_value);
            g.pid_rate_pitch.kP(tuning_value);
            break;

        case TUNING_RATE_ROLL_PITCH_KI:
            g.pid_rate_roll.kI(tuning_value);
            g.pid_rate_pitch.kI(tuning_value);
            break;

        case TUNING_RATE_ROLL_PITCH_KD:
            g.pid_rate_roll.kD(tuning_value);
            g.pid_rate_pitch.kD(tuning_value);
            break;

        case TUNING_RATE_PITCH_KP:
            g.pid_rate_pitch.kP(tuning_value);
            break;

        case TUNING_RATE_PITCH_KI:
            g.pid_rate_pitch.kI(tuning_value);
            break;

        case TUNING_RATE_PITCH_KD:
            g.pid_rate_pitch.kD(tuning_value);
            break;

#if FRAME_CONFIG == HELI_FRAME
        case TUNING_RATE_PITCH_FF:
            g.pid_rate_pitch.ff(tuning_value);
            break;
#endif

        case TUNING_RATE_ROLL_KP:
            g.pid_rate_roll.kP(tuning_value);
            break;

        case TUNING_RATE_ROLL_KI:
            g.pid_rate_roll.kI(tuning_value);
            break;

        case TUNING_RATE_ROLL_KD:
            g.pid_rate_roll.kD(tuning_value);
            break;

#if FRAME_CONFIG == HELI_FRAME
        case TUNING_RATE_ROLL_FF:
            g.pid_rate_roll.ff(tuning_value);
            break;
#endif

        case TUNING_RATE_YAW_KP:
            g.pid_rate_yaw.kP(tuning_value);
            break;

        case TUNING_RATE_YAW_KI:
            g.pid_rate_yaw.kI(tuning_value);
            break;

        case TUNING_RATE_YAW_KD:
            g.pid_rate_yaw.kD(tuning_value);
            break;

#if FRAME_CONFIG == HELI_FRAME
        case TUNING_RATE_YAW_FF:
            g.pid_rate_yaw.ff(tuning_value);
            break;
#endif

        case TUNING_RATE_RP_FILT:
            g.pid_rate_pitch.filt_hz(tuning_value);
            g.pid_rate_roll.filt_hz(tuning_value);
            break;

        case TUNING_RATE_YAW_FILT:
            g.pid_rate_yaw.filt_hz(tuning_value);
            break;

        case TUNING_ALTITUDE_HOLD_KP:
            // Altitude and throttle tuning
            g.p_alt_hold.kP(tuning_value);
            break;

        case TUNING_THROTTLE_RATE_KP:
            g.p_vel_z.kP(tuning_value);
            break;

        case TUNING_ACCEL_Z_KP:
            g.pid_accel_z.kP(tuning_value);
            break;

        case TUNING_ACCEL_Z_KI:
            g.pid_accel_z.kI(tuning_value);
            break;

        case TUNING_ACCEL_Z_KD:
            g.pid_accel_z.kD(tuning_value);
            break;

        case TUNING_WP_SPEED:
            // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
            wp_nav.set_speed_xy(g.rc_6.control_in);
            break;

        case TUNING_CIRCLE_RATE:
            // set circle rate
            circle_nav.set_rate(g.rc_6.control_in/25-20);   // allow approximately 45 degree turn rate in either direction
            break;

        case TUNING_SONAR_GAIN:
            // set sonar gain
            g.sonar_gain.set(tuning_value);
            break;

        case TUNING_LOITER_POSITION_KP:
            // Loiter and navigation tuning
            g.p_pos_xy.kP(tuning_value);
            break;

        case TUNING_VEL_XY_KP:
            g.pi_vel_xy.kP(tuning_value);
            break;

        case TUNING_VEL_XY_KI:
            g.pi_vel_xy.kI(tuning_value);
            break;

        case TUNING_ACRO_RP_KP:
            // Acro roll pitch gain
            g.acro_rp_p = tuning_value;
            break;

        case TUNING_ACRO_YAW_KP:
            // Acro yaw gain
            g.acro_yaw_p = tuning_value;
            break;

        case TUNING_AHRS_RP_KP:
            ahrs._kp.set(tuning_value);
            break;

        case TUNING_AHRS_YAW_KP:
            ahrs._kp_yaw.set(tuning_value);
            break;

// To-Do This is broken and needs to be fixed
//        case TUNING_DECLINATION:
//            // set declination to +-20degrees
//           compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
//            break;

        case TUNING_RC_FEEL_RP:
            // roll-pitch input smoothing
            g.rc_feel_rp = g.rc_6.control_in / 10;
            break;

        case TUNING_RATE_MOT_YAW_HEADROOM:
            motors.set_yaw_headroom(tuning_value*1000);
            break;

#if FRAME_CONFIG == HELI_FRAME
        case TUNING_HELI_EXTERNAL_GYRO:
            motors.ext_gyro_gain(g.rc_6.control_in);
            break;
#endif

#if 0    // disabled for now - we need accessor functions
        case TUNING_EKF_VERTICAL_POS:
            // EKF's baro vs accel (higher rely on accels more, baro impact is reduced)
            ahrs.get_NavEKF()._gpsVertPosNoise = tuning_value;
            break;

        case TUNING_EKF_HORIZONTAL_POS:
            // EKF's gps vs accel (higher rely on accels more, gps impact is reduced)
            ahrs.get_NavEKF()._gpsHorizPosNoise = tuning_value;
            break;

        case TUNING_EKF_ACCEL_NOISE:
            // EKF's accel noise (lower means trust accels more, gps & baro less)
            ahrs.get_NavEKF()._accNoise = tuning_value;
            break;
#endif
    }
}