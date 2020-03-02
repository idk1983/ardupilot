/*
 * @Author: zhangxu
 * @Date: 2020-02-28 09:04:35
 * @LastEditTime: 2020-03-02 21:21:04
 * @LastEditors: Please set LastEditors
 * @Description: add script function to quadplane
 * @FilePath: /ardupilot/ArduPlane/script.cpp
 */
#include "Plane.h"
Script *Script::_singleton;

Script::Script()
{
    stage_reset();
    _highjack_mode_switch = false;
    _initialized = false;
}
Script *script() {
    return Script::get_singleton();
}

bool Script::get_highjack_mode_switch()
{
    return _highjack_mode_switch;
}
void Script::set_highjack_mode_switch(bool enable)
{
    if(enable)
        _highjack_mode_switch = true;
    else
        _highjack_mode_switch = false;
}
void Script::init(void)
{
    if(_initialized == true)
        return;
    else
    {
        stage_reset();
        _initialized = true;
    }
}
uint8_t Script::update_highjack_status(void)
{
    
    uint16_t pulsewidth = plane.channel_execute->get_radio_in();
    uint8_t result;
    if (pulsewidth <= 900 || pulsewidth >= 2200) 
    {
        result = 255; 
    }         
    else if (pulsewidth <= 1230)
    {
        result = 0;
    }
    else if (pulsewidth <= 1360)
    {
        result = 1;
    }
    else if (pulsewidth <= 1490)
    {
        result = 2;
    }
    else if (pulsewidth <= 1620)
    {
        result = 3;
    }
    else if (pulsewidth <= 1749)
    {
        result = 4;
    }
    else
    {
        result = 5;
    }
    if(result == 0)  
    {
        set_highjack_mode_switch(false);
        run_enable = false;
    }
    if(result == 3)
    {
        set_highjack_mode_switch(true);
        run_enable = false;
    }
        
    if(result == 5)
    {
        set_highjack_mode_switch(true);
        run_enable = true;
    }
        
    return result;
}
void Script::run_script(void)
{
    uint8_t current_switch_pos;
    FlightStage_t stg;
    float current_depth,pitch,roll;
    init();
    current_switch_pos = update_highjack_status();
    stg = get_current_flight_stage();
    if (current_switch_pos == 0)
    {
        switch(stg)
        {
            case Stage_Init:
                break;
            case Stage_Idle:
                //switch to mode by remoter control in mode switch task
                _initialized = false;
                break;
            case Stage_UnderWater:
                //switch to mode by remoter control in mode switch task
                _initialized = false;
                break;
            case Stage_FLY:
                //switch to mode by remoter control in mode switch task
                if(!stage_emergency)//do we need to release the highjacked mode switch ?
                {
                    plane.set_mode(Mode::QRTL,ModeReason::RC_COMMAND);
                    stage_emergency = true;
                    stage_fly_enter = false;
                    //set_current_flight_stage(Stage_Init);
                }
                break;
        }
        return;
    }
    if(current_switch_pos == 3)
    {    
        switch(stg)
        {
            case Stage_Init:
                if(!stage_idle_enter)
                {
                    plane.set_mode(Mode::MANUAL,ModeReason::RC_COMMAND);
                    stage_idle_enter = true;
                    set_current_flight_stage(Stage_Idle);
                }
                break;
            case Stage_Idle:
                
                break;
            case Stage_UnderWater:
                if(!stage_idle_enter)
                {
                    plane.set_mode(Mode::MANUAL,ModeReason::RC_COMMAND);
                    stage_idle_enter = true;
                    stage_underwater_enter = false;
                    set_current_flight_stage(Stage_Idle);
                }
                break;
            case Stage_FLY:
                if(!stage_emergency)//do we need to release highjacking the mode switch ?
                {
                    plane.set_mode(Mode::QRTL,ModeReason::RC_COMMAND);
                    stage_emergency = true;
                    stage_fly_enter = false;
                    set_current_flight_stage(Stage_Idle);
                }
                break;
        }
        return;
    }
    if(current_switch_pos == 5)
    {
        switch(stg)
        {
            case Stage_Init:
                if(!stage_underwater_enter)
                {
                    plane.set_mode(Mode::SUBPLANE,ModeReason::RC_COMMAND);
                    stage_underwater_enter = true;
                    set_current_flight_stage(Stage_UnderWater);
                }
                break;
            case Stage_Idle:
                if(!stage_underwater_enter)
                {
                    plane.set_mode(Mode::SUBPLANE,ModeReason::RC_COMMAND);
                    stage_underwater_enter = true;
                    stage_idle_enter = false;
                    set_current_flight_stage(Stage_UnderWater);
                }
                break;
            case Stage_UnderWater:
                //get depth and ahrs info to decide to switch to auto mode 
                current_depth = plane.barometer.get_altitude();
                pitch = degrees(plane.ahrs.get_pitch());
                roll = degrees(plane.ahrs.get_roll());
                // 这里必须增加一个判断条件，当前水下任务到状态
                if(plane.mode_subplane.runned)
                {
                    if((current_depth>-2)&&(fabs(pitch)<3)&&(fabs(roll)<3))
                    {
                        gcs().send_text(MAV_SEVERITY_DEBUG,"enter auto mode");
                        plane.set_mode(Mode::AUTO,ModeReason::RC_COMMAND);
                        stage_underwater_enter = false;
                        stage_fly_enter = true;
                        set_current_flight_stage(Stage_FLY);
                    }
                }
                
                break;
            case Stage_FLY:
                //what to do when the auto task is over 
                break;
        }
        return;
    }
}

Script::FlightStage_t Script::get_current_flight_stage(void)
{
  return _flight_stage;
}
void Script::set_current_flight_stage(FlightStage_t stg)
{
  _flight_stage = stg;
}

void Script::stage_reset(void)
{
  _flight_stage = Stage_Init;
  stage_idle_enter = false;
  stage_underwater_enter = false;
  stage_fly_enter = false;
  stage_emergency = false;
}