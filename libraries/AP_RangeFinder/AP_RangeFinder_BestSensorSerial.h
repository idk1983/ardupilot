/*
 * @Author: your name
 * @Date: 2020-02-21 00:34:42
 * @LastEditTime: 2020-02-21 01:54:29
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_BestSensorSerial.h
 */
#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define BESTSENSOR_SERIAL_BAUD_RATE 9600

class AP_RangeFinder_BestSensorSerial : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return BESTSENSOR_SERIAL_BAUD_RATE;
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm) override;

    uint16_t read_timeout_ms() const override { return 500; }

    char buf[4];
    uint8_t buf_len = 0;
};
