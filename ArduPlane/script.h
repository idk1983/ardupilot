/*
 * @Author: zhangxu
 * @Date: 2020-02-28 09:04:35
 * @LastEditTime: 2020-03-02 19:48:31
 * @LastEditors: Please set LastEditors
 * @Description: add script function to quadplane
 * @FilePath: /ardupilot/ArduPlane/script.h
 */
#pragma once

class Script
{
public:
    //friend class Plane;
    Script();
    friend class Plane;
    /* Do not allow copies */
    Script(const Script &other) = delete;
    Script &operator=(const Script&) = delete;

    // get singleton instance
    static Script *get_singleton()
    {
        return _singleton;
    }
    typedef enum
    {
        Stage_Idle          = 0,
        Stage_UnderWater    = 1,
        Stage_FLY           = 2,
        Stage_Init
    }FlightStage_t;
    
    bool get_highjack_mode_switch(void);
    
    void run_script(void);
    

private:
    bool stage_idle_enter;
    bool stage_underwater_enter;
    bool stage_fly_enter;
    bool stage_emergency;
    bool _initialized;
    bool run_enable;
    bool _highjack_mode_switch;
    uint8_t update_highjack_status(void);
    void set_specified_depth(float depth);
    float get_specified_depth(void);
    void set_depth_tolerance(float tolerance);
    float get_depth_tolerance(void);
    void stage_reset(void);
    FlightStage_t get_current_flight_stage(void);
    void set_current_flight_stage(FlightStage_t stg);
    void set_highjack_mode_switch(bool enable = true);
    void init(void);
    FlightStage_t _flight_stage;
    static Script *_singleton;
};
Script *script();

