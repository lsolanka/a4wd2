#pragma once

#include <stdint.h>

namespace mpu9250
{
enum class accel_scale : uint8_t
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum class gyro_scale : uint8_t
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum class mag_scale : uint8_t
{
    MFS_14BITS = 0,  // 0.6 mG per LSB
    MFS_16BITS       // 0.15 mG per LSB
};

enum class mag_mode : uint8_t
{
    POWER_DOWN = 0x00,
    SINGLE_MEASUREMENT = 0x01,
    CONT_MEASUREMENT_1 = 0x02,
    CONT_MEASUREMENT_2 = 0x06,
    EXTERNAL_TRIGGER_MEASUREMENT = 0x04,
    SELF_TEST = 0x08,
    FUSE_ROM_ACCESS = 0x0f
};

}  // namespace mpu9250
