/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include <AC_RCINPUT.h>

const AP_Param::GroupInfo RCInput::var_info[] PROGMEM = {

    // @Param: CH1_FUNC
    // @DisplayName: Channel 1 function
    // @Description: Select which function is controlled by CH1 input
    // @Values: 1: Roll, 2: Pitch, 3: Throttle, 4: Yaw
    // @User: Standard
    AP_GROUPINFO("CH1_FUNC", 0, RCInput, _ch1_function, CTRL_ROLL),

    // @Param: CH2_FUNC
    // @DisplayName: Channel 2 option
    // @Description: Select which function is controlled by CH2 input
    // @Values: 1: Roll, 2: Pitch, 3: Throttle, 4: Yaw
    // @User: Standard
    AP_GROUPINFO("CH2_FUNC", 1, RCInput, _ch2_function, CTRL_PITCH),

    // @Param: CH3_FUNC
    // @DisplayName: Channel 3 option
    // @Description: Select which function is controlled by CH3 input
    // @Values: 1: Roll, 2: Pitch, 3: Throttle, 4: Yaw
    // @User: Standard
    AP_GROUPINFO("CH3_FUNC", 2, RCInput, _ch3_function, CTRL_THROTTLE),

    // @Param: CH4_FUNC
    // @DisplayName: Channel 4 function
    // @Description: Select which function is controlled by CH4 input
    // @Values: 1: Roll, 2: Pitch, 3: Throttle, 4: Yaw
    // @User: Standard
    AP_GROUPINFO("CH4_FUNC", 3, RCInput, _ch4_function, CTRL_YAW),

    // @Param: CH5_FUNC
    // @DisplayName: Channel 5 function
    // @Description: Select which function is controlled by CH5 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH5_FUNC", 4, RCInput, _ch5_function, SW_FLIGHT_MODE),

    // @Param: CH6_FUNC
    // @DisplayName: Channel 6 option
    // @Description: Select which function is controlled by CH6 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH6_FUNC", 5, RCInput, _ch6_function, NO_FUNCTION),

    // @Param: CH7_FUNC
    // @DisplayName: Channel 7 function
    // @Description: Select which function is controlled by CH7 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH7_FUNC", 6, RCInput, _ch7_function, NO_FUNCTION),

    // @Param: CH8_FUNC
    // @DisplayName: Channel 8 function
    // @Description: Select which function is controlled by CH8 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH8_FUNC", 7, RCInput, _ch8_function, NO_FUNCTION),

    // @Param: CH9_FUNC
    // @DisplayName: Channel 9 function
    // @Description: Select which function is controlled by CH9 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH9_FUNC", 8, RCInput, _ch9_function, NO_FUNCTION),

    // @Param: CH10_FUNC
    // @DisplayName: Channel 10 function
    // @Description: Select which function is controlled by CH10 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH10_FUNC", 9, RCInput, _ch10_function, NO_FUNCTION),

    // @Param: CH11_FUNC
    // @DisplayName: Channel 11 function
    // @Description: Select which function is controlled by CH11 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH11_FUNC", 10, RCInput, _ch11_function, NO_FUNCTION),

    // @Param: CH12_FUNC
    // @DisplayName: Channel 12 function
    // @Description: Select which function is controlled by CH12 input
    // @Values: 0:Do Nothing, 5: Flight Mode, 10:Flip, 11:Simple Mode, 12:RTL, 13:Save Trim, 14:Save WP, 15:Camera Trigger, 16:RangeFinder, 17:Fence, 18:ResetToArmedYaw, 19:Super Simple Mode, 20:Acro Trainer, 22:Auto, 23:AutoTune, 24:Land, 25:EPM, 26:Parachute Enable, 27:Parachute Release, 28:Parachute 3pos, 29:Auto Mission Reset, 30:AttCon Feed Forward, 31:AttCon Accel Limits, 32:Retract Mount, 33:Relay On/Off, 34:Landing Gear, 35:E-Stop, 36:Motor Interlock
    // @User: Standard
    AP_GROUPINFO("CH12_FUNC", 11, RCInput, _ch12_function, NO_FUNCTION),

    AP_GROUPEND
};

// object constructor.
RCInput::RCInput(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

//Search for first channel with assigned function
uint8_t RCInput::find_rc_input_func(int16_t func) const
{
    if (_ch1_function == func){
        return 1;
    } else if (_ch2_function == func){
        return 2;
    } else if (_ch3_function == func){
        return 3;
    } else if (_ch4_function == func){
        return 4;
    } else if (_ch5_function == func){
        return 5;
    } else if (_ch6_function == func){
        return 6;
    } else if (_ch7_function == func){
        return 7;
    } else if (_ch8_function == func){
        return 8;
    } else if (_ch9_function == func){
        return 9;
    } else if (_ch10_function == func){
        return 10;
    } else if (_ch11_function == func){
        return 11;
    } else if (_ch12_function == func){
        return 12;
    } else {
        // Failed to find function, return zero to indicate failure
        return 0;
    }
}

// set_primary_control_channels
void RCInput::set_primary_control_channels()
{
    _roll_chan = find_rc_input_func(CTRL_ROLL);
    _pitch_chan = find_rc_input_func(CTRL_PITCH);
    _throttle_chan = find_rc_input_func(CTRL_THROTTLE);
    _yaw_chan = find_rc_input_func(CTRL_YAW);
    _flight_mode_chan = find_rc_input_func(SW_FLIGHT_MODE);
}

// check_if_rc_input_func_used - Check to see if any of the RC input switches are set to a given mode.
bool RCInput::check_if_rc_input_func_used(int16_t check_function)
{
    bool ret = _ch1_function == check_function || _ch2_function == check_function || _ch3_function == check_function 
                || _ch4_function == check_function || _ch5_function == check_function || _ch6_function == check_function
                || _ch7_function == check_function || _ch8_function == check_function || _ch9_function == check_function 
                || _ch10_function == check_function || _ch11_function == check_function || _ch12_function == check_function;

    return ret;
}

// check_duplicate_rc_input_func - Check to see if any RC input switch functions are duplicated
bool RCInput::check_duplicate_rc_input_func()
{
    bool ret = ((_ch1_function != NO_FUNCTION) && (_ch1_function == _ch2_function ||
                    _ch1_function == _ch3_function || _ch1_function == _ch4_function ||
                    _ch1_function == _ch5_function || _ch1_function == _ch6_function ||
                    _ch1_function == _ch7_function || _ch1_function == _ch8_function ||
                    _ch1_function == _ch9_function || _ch1_function == _ch10_function ||
                    _ch1_function == _ch11_function || _ch1_function == _ch12_function));

    ret = ret || ((_ch2_function != NO_FUNCTION) && (
                    _ch2_function == _ch3_function || _ch2_function == _ch4_function ||
                    _ch2_function == _ch5_function || _ch2_function == _ch6_function ||
                    _ch2_function == _ch7_function || _ch2_function == _ch8_function ||
                    _ch2_function == _ch9_function || _ch2_function == _ch10_function ||
                    _ch2_function == _ch11_function || _ch2_function == _ch12_function));

    ret = ret || ((_ch3_function != NO_FUNCTION) && (_ch3_function == _ch4_function ||
                    _ch3_function == _ch5_function || _ch3_function == _ch6_function ||
                    _ch3_function == _ch7_function || _ch3_function == _ch8_function ||
                    _ch3_function == _ch9_function || _ch3_function == _ch10_function ||
                    _ch3_function == _ch11_function || _ch3_function == _ch12_function));

    ret = ret || ((_ch4_function != NO_FUNCTION) && (
                    _ch4_function == _ch5_function || _ch4_function == _ch6_function ||
                    _ch4_function == _ch7_function || _ch4_function == _ch8_function ||
                    _ch4_function == _ch9_function || _ch4_function == _ch10_function ||
                    _ch4_function == _ch11_function || _ch4_function == _ch12_function));

    ret = ret || ((_ch5_function != NO_FUNCTION) && (_ch5_function == _ch6_function ||
                    _ch5_function == _ch7_function || _ch5_function == _ch8_function ||
                    _ch5_function == _ch9_function || _ch5_function == _ch10_function ||
                    _ch5_function == _ch11_function || _ch5_function == _ch12_function));

    ret = ret || ((_ch6_function != NO_FUNCTION) && (
                    _ch6_function == _ch7_function || _ch6_function == _ch8_function ||
                    _ch6_function == _ch9_function || _ch6_function == _ch10_function ||
                    _ch6_function == _ch11_function || _ch6_function == _ch12_function));

    ret = ret || ((_ch7_function != NO_FUNCTION) && (_ch7_function == _ch8_function ||
                    _ch7_function == _ch9_function || _ch7_function == _ch10_function ||
                    _ch7_function == _ch11_function || _ch7_function == _ch12_function));

    ret = ret || ((_ch8_function != NO_FUNCTION) && (
                    _ch8_function == _ch9_function || _ch8_function == _ch10_function ||
                    _ch8_function == _ch11_function || _ch8_function == _ch12_function));

    ret = ret || ((_ch9_function != NO_FUNCTION) && (_ch9_function == _ch10_function ||
                    _ch9_function == _ch11_function || _ch9_function == _ch12_function));

    ret = ret || ((_ch10_function != NO_FUNCTION) && (
                    _ch10_function == _ch11_function || _ch10_function == _ch12_function));

    ret = ret || ((_ch11_function != NO_FUNCTION) && (_ch11_function == _ch12_function));

    return ret;
}

