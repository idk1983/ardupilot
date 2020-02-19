#include "mode.h"
#include "Plane.h"

bool ModeFull::_enter()
{
    FlightStage::FlightStage_t stg = flightstage.get_current_flight_stage();
    if(stg == FlightStage::FlightStage_t::Stage_Idle)
    {
        //manual for test
        plane.throttle_allows_nudging = false;
        plane.auto_throttle_mode = false;
        plane.auto_navigation_mode = false;

        flightstage.stage_idle_init = true;
        
        gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is in the full auto mode");
        gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is in the idle stage");
        return true;
    }
    else 
    {
        flightstage.stage_idle_init = false;
        gcs().send_text(MAV_SEVERITY_INFO, "flight stage init error,try again");
    }


    return true;
}

void ModeFull::_exit()
{
    plane.barometer.set_primary_baro(plane.barometer.get_depth_sensor());
            

    if(SRV_Channels::exchange_throttle_function(CH_14))
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"succeed to exchange throttle");  
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_DEBUG,"fail to exchange throttle");  
    }

}

void ModeFull::update()
{
    FlightStage::FlightStage_t stg = flightstage.get_current_flight_stage();
    if(stg == FlightStage::FlightStage_t::Stage_Idle)
    {
        if(!flightstage.stage_idle_init)
        {
            plane.throttle_allows_nudging = false;
            plane.auto_throttle_mode = false;
            plane.auto_navigation_mode = false;

            flightstage.stage_idle_init = true;
            
            gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is in the full auto mode");
            gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is in the idle stage");
        }
        else
        {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
            plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();
            //read rc input to enter next stage
            if(plane.channel_execute->get_radio_in()>1000)
            {
                flightstage.set_current_flight_stage(FlightStage::FlightStage_t::Stage_UnderWater);
                flightstage.stage_idle_init = false;
            }
        }
    }
    else if(stg == FlightStage::FlightStage_t::Stage_UnderWater)
    {
        if(!flightstage.stage_underwater_init)
        {
            plane.throttle_allows_nudging = false;
            plane.auto_throttle_mode = true;
            plane.auto_navigation_mode = false;

#if SOARING_ENABLED == ENABLED
    // for ArduSoar soaring_controller
            plane.g2.soaring_controller.init_cruising();
#endif

            plane.set_target_altitude_current();
            //change barometer 
            plane.barometer.set_primary_baro(plane.barometer.get_depth_sensor());
            
            //change the default channel for throttle in able to use another motor underwater
            if(SRV_Channels::exchange_throttle_function(CH_9))
            {
                gcs().send_text(MAV_SEVERITY_DEBUG,"succeed to exchange throttle");  
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_DEBUG,"fail to exchange throttle");  
            }
            flightstage.stage_underwater_init = true;
            plane.change_target_altitude(-50);
            gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is in the full auto mode");
            gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is in the underwater stage");
        }
        else
        {
            // after achieve specified depth for 5 senconds enter next stage 
            // get current depth
            float current_depth = plane.barometer.get_altitude();
            if ((fabs(current_depth - flightstage.specified_depth)<flightstage.depth_tolerance)&&(!flightstage.stage_achieve_depth))
            {
                flightstage.achived_depth_time = millis();
                flightstage.stage_achieve_depth = true;
                gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is at the specified depth");
            }
            uint32_t now = millis();
            if((now - flightstage.achived_depth_time>5000)&&(flightstage.achived_depth_time!=0))
            {
                // set target depth to 0, make the plane back to surface
                plane.change_target_altitude(50);

                if(fabs(current_depth)<flightstage.depth_tolerance)
                {
                    flightstage.set_current_flight_stage(FlightStage::FlightStage_t::Stage_Takeoff);
                    flightstage.stage_underwater_init = false;
                    gcs().send_text(MAV_SEVERITY_INFO, "the vehicle is leaving underwater stage");
                }
                
            }
            else
            {
                plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
                plane.update_load_factor();
                plane.update_fbwb_speed_height();
            }
        }
    }
    else if (stg == FlightStage::FlightStage_t::Stage_Takeoff)
    {//takeoff 需要查一下应该使用哪个模式
        if(!flightstage.stage_takeoff_init)
        {
            flightstage.stage_takeoff_init = true;
            plane.throttle_allows_nudging = true;
            plane.auto_navigation_mode = false;
            plane.barometer.set_primary_baro(plane.barometer.get_height_sensor());
            if(!plane.quadplane.init_mode() && plane.previous_mode != nullptr)
            {
                plane.control_mode = plane.previous_mode;
            } 
            else 
            {
                plane.auto_throttle_mode = false;
                plane.auto_state.vtol_mode = true;
            }
            
            //change to the default channel for throttle when we intent to fly
            if(SRV_Channels::exchange_throttle_function(CH_3))
            {
                gcs().send_text(MAV_SEVERITY_DEBUG,"succeed to exchange throttle");  
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_DEBUG,"fail to exchange throttle");  
            }
            gcs().send_text(MAV_SEVERITY_DEBUG,"the vehicle is in takeoff stage");  
        }
        else
        {
            // set nav_roll and nav_pitch using sticks
            int16_t roll_limit = MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);

            float pitch_input = plane.channel_pitch->norm_input();
            // Scale from normalized input [-1,1] to centidegrees
            if (plane.quadplane.tailsitter_active()) 
            {
                // separate limit for tailsitter roll, if set
                if (plane.quadplane.tailsitter.max_roll_angle > 0) 
                {
                    roll_limit = plane.quadplane.tailsitter.max_roll_angle * 100.0f;
                }
                // angle max for tailsitter pitch
                plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
            } 
            else 
            {
                // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
                // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
                if (pitch_input > 0) 
                {
                    plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max_cd, plane.quadplane.aparm.angle_max);
                } 
                else 
                {
                    plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min_cd, plane.quadplane.aparm.angle_max);
                }
                plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
            }
            plane.nav_roll_cd = (plane.channel_roll->get_control_in() / 4500.0) * roll_limit;
            plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -roll_limit, roll_limit);
            // 判断rc输入的条件，进入下一阶段
            if(plane.channel_execute->get_radio_in()>2000)
            {
                flightstage.set_current_flight_stage(FlightStage::FlightStage_t::Stage_FixWing);
                flightstage.stage_takeoff_init = false;
                gcs().send_text(MAV_SEVERITY_DEBUG,"the vehicle is leaving takeoff and loiter stage");
            }
            // if(plane.channel_execute->get_radio_in()<1000)
            // {
            //     flightstage.set_current_flight_stage(FlightStage::FlightStage_t::Stage_Land);
            //     flightstage.stage_takeoff_init = false;
            //     gcs().send_text(MAV_SEVERITY_DEBUG,"the vehicle is leaving takeoff and loiter stage");
            // }
            
        }
    }
    else if (stg == FlightStage::FlightStage_t::Stage_FixWing)
    {
        if(!flightstage.stage_fixwing_init)
        {
            plane.throttle_allows_nudging = false;
            plane.auto_throttle_mode = true;
            plane.auto_navigation_mode = false;
#if SOARING_ENABLED == ENABLED
    // for ArduSoar soaring_controller
            plane.g2.soaring_controller.init_cruising();
#endif
            plane.set_target_altitude_current();
        }
        else
        {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
            plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();
            //read rc input to enter next stage
            if(plane.channel_execute->get_radio_in()>1000)
            {
                flightstage.set_current_flight_stage(FlightStage::FlightStage_t::Stage_UnderWater);
                flightstage.stage_idle_init = false;
            }
        }

    }
    else
    {
        //do nothing
    }
}

