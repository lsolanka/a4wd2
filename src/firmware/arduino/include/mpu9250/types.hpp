#pragma once

namespace mpu9250
{

enum class accel_scale
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum class gyro_scale
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum class mag_scale
{
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
};

} // namespace mpu9250
