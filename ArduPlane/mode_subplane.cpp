#include "mode.h"
#include "Plane.h"
#define UNDER_WATER_AlGORITHM 1
//#define TARGET_DEPTH  (-1)
//#define INIT_DEPTH (0)
#define PITCH_LIMIT_MAX (30*100)
#define PITCH_LIMIT_MIN (-30*100)
#define TIME_BASE (1000)
// FBWA:1 or FBWB:2
//需要更新所有新加的变量的初始化以及如何更新,地面站更新？还是遥控器输入？默认值是多少？

bool ModeSubPlane::_enter()
{
    //FBWA
#if UNDER_WATER_AlGORITHM == 1
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;
    current_stage = dive_stage;
    last_stage = idle_stage;
    mission_start = false;
    start_time = 0;
    stable_time = 0;
    float_time = 0;
    mission_intervel_time = 0;
    control_mode = manual;
#endif
    //FBWB
#if UNDER_WATER_AlGORITHM == 2 
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = false;
#if SOARING_ENABLED == ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif
#endif    
   

    //change barometer under water surface
    plane.barometer.set_primary_baro(plane.barometer.get_depth_sensor());
    plane.barometer.get_altitude();
    //avoid quadplane motors start in the water
    assist_speed_temp = plane.quadplane.get_assisted_speed();
    plane.quadplane.set_assisted_speed(0);

    //this message is for debug
    float temp = plane.quadplane.get_assisted_speed();
    gcs().send_text(MAV_SEVERITY_DEBUG,"set assist speed %f",temp);  
    //change the default channel for throttle in able to use another motor underwater
    if(SRV_Channels::exchange_throttle_function(CH_9))
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"succeed to exchange throttle");  
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"fail to exchange throttle");  
    }
    //FBWB
#if UNDER_WATER_AlGORITHM == 2      
    plane.set_target_altitude_current();
    // plane.change_target_altitude(-50);
#endif     
    gcs().send_text(MAV_SEVERITY_DEBUG,"entered the subplane mode");  

    return true;
}
void ModeSubPlane::_exit()
{
    //change barometer under water
    plane.barometer.set_primary_baro(plane.barometer.get_height_sensor());
    //change to the default channel for throttle when we intent to fly
    if(SRV_Channels::exchange_throttle_function(CH_3))
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"succeed to exchange throttle");  
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"fail to exchange throttle");  
    }
    //restore the assist speed
    plane.quadplane.set_assisted_speed(assist_speed_temp);
    gcs().send_text(MAV_SEVERITY_DEBUG,"set assist speed %f",assist_speed_temp); 
    mission_start = false;
    current_stage = dive_stage;  
    start_time = 0;
    stable_time = 0;
    mission_intervel_time = 0;
    float_time = 0;
    control_mode = manual;
    gcs().send_text(MAV_SEVERITY_DEBUG,"exited the subplane mode");   

}
void ModeSubPlane::update()
{
    //FBWB
#if UNDER_WATER_AlGORITHM == 2        
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    plane.update_fbwb_speed_height();
    if (plane.failsafe.rc_failsafe) 
    {
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 30;
    }
#endif
   //FBWA
#if UNDER_WATER_AlGORITHM == 1 
    int16_t channel_execute_input = plane.channel_execute->get_radio_in();
    if(channel_execute_input>1800)
    {
        control_mode = emergency;
        current_stage = dive_stage;
        last_stage = idle_stage;
        mission_start = false;
        start_time = 0;
        stable_time = 0;
        float_time = 0;
        mission_intervel_time = 0;
    }
    else if (channel_execute_input < 1150)
    {
        control_mode = autorun;
    } 
    else
    {
        control_mode = manual;
        current_stage = dive_stage;
        last_stage = idle_stage;
        mission_start = false;
        start_time = 0;
        stable_time = 0;
        float_time = 0;
        mission_intervel_time = 0;
    }
    uint32_t tnow = millis();
    float current_depth = plane.barometer.get_altitude();
    if((tnow - mission_intervel_time)>1000)
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"current depth %f",current_depth);
        mission_intervel_time = tnow;
    }
    if(control_mode == autorun)
    {
        if(plane.channel_throttle->norm_input_dz()<0.05)
            return;
        
        if(mission_start == false)
        {
            start_time = tnow;
            mission_start = true;
        }
       
        float pitch_pid_p = 1.0;
        
        //do not change the direction
        plane.channel_rudder->set_control_in(0);

        plane.nav_roll_cd  = 0;
        plane.update_load_factor();

        switch (current_stage)
        {
        case stable_stage:
            //KEEP DEPTH
            if(last_stage != stable_stage)
                gcs().send_text(MAV_SEVERITY_DEBUG,"stable");
            last_stage = stable_stage;
            pitch_input_offset = pitch_pid_p*(current_depth-plane.g.d_target_depth);
            //pitch_input_offset = -1;
            
            manual_interact();
            if(((tnow - stable_time)>(plane.g.d_swim_time*TIME_BASE))&&(stable_time != 0))
            {
                current_stage = float_stage;
                float_time = tnow;
            }
            break;
        case float_stage:
            if(last_stage != float_stage)
                gcs().send_text(MAV_SEVERITY_DEBUG,"float");
            last_stage = float_stage;
            pitch_input_offset = pitch_pid_p*(current_depth-plane.g.d_init_depth);
            //pitch_input_offset = 1;
            manual_interact();
            if(abs(current_depth-plane.g.d_init_depth)<0.05)
            current_stage = idle_stage;
            break;
        case dive_stage:
            if(last_stage != dive_stage)
                gcs().send_text(MAV_SEVERITY_DEBUG,"dive");
            last_stage = dive_stage;
            pitch_input_offset = pitch_pid_p*(current_depth-plane.g.d_target_depth);
            //pitch_input_offset = -1;
            manual_interact();
            //achiev specific depth
            if((current_depth - plane.g.d_target_depth)<=0)
            {
                current_stage = stable_stage;
                stable_time = tnow;
            }
            break;
        case idle_stage:
            if(last_stage != idle_stage)
                gcs().send_text(MAV_SEVERITY_DEBUG,"idle");
            last_stage = idle_stage;
            plane.channel_rudder->set_control_in(0);
            plane.channel_pitch->set_control_in(0);
            plane.channel_roll->set_control_in(0);
            start_time = 0;
            stable_time = 0;
            if(mission_intervel_time == 0)
                mission_intervel_time = tnow;
            //if(((tnow - mission_intervel_time)>5000)&&(mission_intervel_time != 0))
            //    current_stage = dive_stage;
            break;
        default:
            gcs().send_text(MAV_SEVERITY_DEBUG,"never should run here");
            break;
        }
        if(((tnow - start_time)>(plane.g.d_total_time*TIME_BASE))&&(start_time != 0))
        {
            plane.channel_rudder->set_control_in(0);
            plane.channel_pitch->set_control_in(0);
            plane.channel_roll->set_control_in(0);
            current_stage = idle_stage;
            gcs().send_text(MAV_SEVERITY_DEBUG,"30s timeout");
        }
    }
    else if (control_mode == manual)
    {
        plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
        pitch_input = plane.channel_pitch->norm_input();
        if (pitch_input > 0) {
            plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
        } else {
            plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
        }
        plane.adjust_nav_pitch_throttle();
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
        if (plane.fly_inverted()) {
            plane.nav_pitch_cd = -plane.nav_pitch_cd;
        }
    }
    else
    {
        current_stage = dive_stage;
        last_stage = idle_stage;
        mission_start = false;
        start_time = 0;
        stable_time = 0;
        mission_intervel_time = 0;
    }
    
#endif    
}
void ModeSubPlane::manual_interact()
{
    pitch_input = plane.channel_pitch->norm_input();
    //this value is make sure the subplane can dive into water,need to test its polarity
    //accept manual input for change pitch control value
    if (pitch_input > 0) 
    {
        plane.nav_pitch_cd = pitch_input * PITCH_LIMIT_MAX + (pitch_input_offset+plane.g.d_swim_offset) * PITCH_LIMIT_MAX;
    } 
    else 
    {
        plane.nav_pitch_cd = -(pitch_input * PITCH_LIMIT_MIN) + (pitch_input_offset+plane.g.d_swim_offset) * PITCH_LIMIT_MIN;
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, PITCH_LIMIT_MIN, PITCH_LIMIT_MAX);
    if (plane.fly_inverted()) 
    {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
}