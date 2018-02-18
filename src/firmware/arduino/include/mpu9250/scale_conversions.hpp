#pragma once

#include "types.hpp"

namespace mpu9250
{

float get_accel_resolution(accel_scale scale)
{
    switch (scale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        case accel_scale::AFS_2G:
            return 2.0 / 32768.0;
            break;
        case accel_scale::AFS_4G:
            return 4.0 / 32768.0;
            break;
        case accel_scale::AFS_8G:
            return 8.0 / 32768.0;
            break;
        case accel_scale::AFS_16G:
            return 16.0 / 32768.0;
            break;
    }
}

float get_gyro_resolution(gyro_scale scale)
{
    switch (scale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit
        // value:
        case gyro_scale::GFS_250DPS:
            return 250.0 / 32768.0;

        case gyro_scale::GFS_500DPS:
            return 500.0 / 32768.0;

        case gyro_scale::GFS_1000DPS:
            return 1000.0 / 32768.0;

        case gyro_scale::GFS_2000DPS:
            return 2000.0 / 32768.0;
    }
}

} // namespace mpu9250
