#pragma once

#include <stdint.h>

#include <SoftwareSerial.h>

#include "ArduinoJson.hpp"
#include "I2Cdev/I2Cdev.h"

#include "mpu9250.hpp"
#include <mpu9250/scale_conversions.hpp>
#include <mpu9250/types.hpp>

namespace mpu9250
{

template <uint16_t JSON_BUFFER_SIZE, uint16_t READ_PERIOD_MS,
          uint8_t ADDRESS = regs::addr::DEFAULT_ADDRESS>
class timed_mpu9250 : public mpu9250
{
    static_assert(JSON_BUFFER_SIZE > 0, "JSON_BUFFER_SIZE must be > 0");
    static_assert(READ_PERIOD_MS > 0, "READ_PERIOD_MS must be > 0");

  public:
    timed_mpu9250()
        : mpu9250(mpu9250::parameters{ADDRESS}), last_send_time_ms(0),
          imu_initialized(false)
    {
    }

    /** Check timer and send data if necessary. */
    void spin()
    {
        uint32_t now = millis();
        if (now - last_send_time_ms > READ_PERIOD_MS)
        {
            if (!imu_initialized)
            {
                Serial.println("# Cannot read and send data. MPU9250 was not initialized "
                               "or initialization failed!");
                return;
            }

            last_send_time_ms = now;
            read_and_serialise();
        }
    }

    void init_and_calibrate(bool calibrate_mag = false)
    {
        // IMU init
        Serial.println("# Initialising MPU9250...");
        delay(1000);

        initialize();
        if (testConnection())
        {
            imu_initialized = true;
        }
        else
        {
            Serial.println("# Failed to initialize MPU9250. Data will not be read or "
                           "sent over serial connection.");
            return;
        }
        Serial.println("# MPU9250 initialized for active data mode....");

        Serial.println("# Accel/Gyro calibration...");
        calibrateAccelGyro(gyroBias, accelBias);
        Serial.print("# accel biases (mg): ");
        Serial.print(1000. * accelBias[0]);
        Serial.print(" ");
        Serial.print(1000. * accelBias[1]);
        Serial.print(" ");
        Serial.println(1000. * accelBias[2]);

        Serial.print("# gyro biases (dps)");
        Serial.print(" ");
        Serial.print(gyroBias[0]);
        Serial.print(" ");
        Serial.print(gyroBias[1]);
        Serial.print(" ");
        Serial.println(gyroBias[2]);

        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        delay(10);
        Serial.println("# Magnetometer init...");
        initMagnetometer();
        auto mag_sensitivity_adj = getMagSensitivityAdjustment();
        Serial.print("# magnetometer sensitivity adjustment: ");
        Serial.print(mag_sensitivity_adj[0]);
        Serial.print(" ");
        Serial.print(mag_sensitivity_adj[1]);
        Serial.print(" ");
        Serial.println(mag_sensitivity_adj[2]);

        if (calibrate_mag)
        {
            calibrateMag();
            magBias[0] = getMagBias()[0];
            magBias[1] = getMagBias()[1];
            magBias[2] = getMagBias()[2];
            magScale[0] = getMagScale()[0];
            magScale[1] = getMagScale()[1];
            magScale[2] = getMagScale()[2];
        }
    }

    void print_whoami_to_serial()
    {
        // set i2c bypass enable pin to true to access magnetometer
        I2Cdev::writeByte(getParameters().dev_addr, regs::addr::INT_PIN_CFG, 0x02);
        // Sanity check for the magnetometer
        uint8_t id =
                I2Cdev::readByte(regs::addr::mag::ADDRESS, regs::addr::mag::WHO_AM_I);

        Serial.print("# AK8963: I AM ");
        Serial.print(id, HEX);
        Serial.print("; I should be ");
        Serial.println(0x48, HEX);
    }

  private:
    uint32_t last_send_time_ms;

    float gyroBias[3] = {0, 0, 0};
    float accelBias[3] = {0, 0, 0};

    // Manually collected calibration data with the sensor mounted on the chasis
    float magBias[3] = {121, 460, -420}; // mG
    float magScale[3] = {1.01, 1.01, 0.98};

    bool imu_initialized;

    void read_and_serialise()
    {
        using namespace ArduinoJson;

        static const float accel_resolution =
                get_accel_resolution(getParameters().ascale);
        static const float gyro_resolution = get_gyro_resolution(getParameters().gscale);
        static const float mag_resolution = get_mag_resolution(getParameters().mscale);

        const int IMU_BUFFER_SIZE = 300;
        StaticJsonBuffer<IMU_BUFFER_SIZE> buffer;
        JsonObject& root = buffer.createObject();
        JsonObject& imu_json = root.createNestedObject("imu");

        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        imu_json["ax"] = ax * accel_resolution - accelBias[0];
        imu_json["ay"] = ay * accel_resolution - accelBias[1];
        imu_json["az"] = az * accel_resolution - accelBias[2];

        imu_json["gx"] = gx * gyro_resolution;
        imu_json["gy"] = gy * gyro_resolution;
        imu_json["gz"] = gz * gyro_resolution;

        imu_json["mx"] =
                (mx * mag_resolution * getMagSensitivityAdjustment()[0] - magBias[0]) *
                magScale[0];
        imu_json["my"] =
                (my * mag_resolution * getMagSensitivityAdjustment()[1] - magBias[1]) *
                magScale[1];
        imu_json["mz"] =
                (mz * mag_resolution * getMagSensitivityAdjustment()[2] - magBias[2]) *
                magScale[2];
        root.printTo(Serial);
        Serial.println();
    }
};

} // namespace mpu9250
