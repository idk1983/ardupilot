/*
 * @Author: your name
 * @Date: 2020-02-21 00:35:05
 * @LastEditTime: 2020-02-21 02:19:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_BestSensorSerial.cpp
 */
#include "AP_RangeFinder_BestSensorSerial.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

// read - return last value measured by sensor
bool AP_RangeFinder_BestSensorSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    int32_t sum = 0;
    int16_t nbytes = uart->available();
    uint16_t count = 0;
    
    while (nbytes-- > 0) 
    {
        uint8_t c = uart->read();
        if ((c != 0xFF)&&(buf_len == 0)) 
        {
            //discard
            continue;
        } 
        buf[buf_len++] = c;
        if(buf_len > 3)
        {
            if(((buf[1]+buf[2]+0xFF)&0xFF) == c)
            {
                sum += (int16_t)(buf[1]*256) + buf[2];
                count ++;           
            }
            buf_len = 0;  
        }
    }

    if (count == 0) {
        return false;
    }

    // This sonar gives the metrics in inches, so we have to transform this to centimeters
    reading_cm = 0.1f * sum / count;

    return true;
}
