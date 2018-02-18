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

} // namespace mpu9250
