#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include "ArduinoJson.hpp"
#include "I2Cdev/I2Cdev.h"

#include "mpu9250/mpu9250.hpp"
#include "mpu9250/registers.hpp"
#include "mpu9250/scale_conversions.hpp"
#include "mpu9250/types.hpp"

#define SRF_BASE_ADDRESS 0x70 // Address of the SRF08
#define NUM_SENSORS 2
#define CMD                                                                              \
    (uint8_t)0x00 // Command byte, values of 0 being sent with write have to be masked as
                  // a byte to stop them being misinterpreted as NULL this is a bug with
                  // arduino 1.0
#define LIGHTBYTE 0x01 // Byte to read light sensor
#define RANGEBYTE 0x02 // Byte for start of ranging data
#define GAINBYTE 0x01

using namespace ArduinoJson;

mpu9250::mpu9250 imu;
bool imu_initialized = false;

float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};

void setup()
{
    Wire.begin();
    delay(100);

    // Reduce range
    for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; ++sensor_idx)
    {
        Wire.beginTransmission(SRF_BASE_ADDRESS + sensor_idx);
        Wire.write(RANGEBYTE);
        Wire.write(0x46);
        Wire.endTransmission();

        Wire.beginTransmission(SRF_BASE_ADDRESS + sensor_idx);
        Wire.write(GAINBYTE);
        Wire.write(0x18);
        Wire.endTransmission();
    }

    Serial.begin(57600);

    // IMU init
    delay(5000);

    imu.calibrateAccelGyro(gyroBias, accelBias);
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

    imu.initialize();
    if (imu.testConnection())
    {
        imu_initialized = true;
    }
    Serial.println("# MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = I2Cdev::readByte(mpu9250::regs::addr::mag::ADDRESS,
                              mpu9250::regs::addr::mag::WHO_AM_I);
    Serial.print("# AK8963: I AM ");
    Serial.print(d, HEX);
    Serial.print("; I should be ");
    Serial.println(0x48, HEX);
}

void triggerRanging(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(CMD);
    Wire.write(0x51);
    Wire.endTransmission();

    delay(30);
}

/** Get a range reading from a ranger at the specified address. **/
int read_range(uint8_t address)
{
    uint8_t highByte = 0x00;
    uint8_t lowByte = 0x00;

    int range = 0;

    triggerRanging(address);

    Wire.beginTransmission(address);
    Wire.write(RANGEBYTE);
    Wire.endTransmission();

    Wire.requestFrom((int)address, 2);
    while (Wire.available() < 2)
        ;
    highByte = Wire.read();
    lowByte = Wire.read();

    range = (highByte << 8) + lowByte;

    return (range);
}

/** Get light sensor reading from the sensor at the specified address **/
int read_light(uint8_t address)
{
    Wire.beginTransmission(address);
    Wire.write(LIGHTBYTE);
    Wire.endTransmission();

    Wire.requestFrom((int)address, 1);
    while (Wire.available() < 0)
        ;
    return Wire.read();
}

void loop()
{
    static const float accel_resolution =
            mpu9250::get_accel_resolution(imu.getParameters().ascale);
    static const float gyro_resolution =
            mpu9250::get_gyro_resolution(imu.getParameters().gscale);

    // To generate an object of 3 srf08 values we need roughly 100 bytes
    // const int BUFFER_SIZE = 100;
    // StaticJsonBuffer<BUFFER_SIZE> buffer;
    // JsonObject& root = buffer.createObject();
    // JsonObject& srf08_json = root.createNestedObject("srf08");

    // for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; ++sensor_idx)
    //{
    //    uint8_t sensor_address = SRF_BASE_ADDRESS + sensor_idx;
    //    int range = read_range(sensor_address);
    //    int light = read_light(sensor_address);

    //    srf08_json["a"] = sensor_address;
    //    srf08_json["r"] = range;
    //    srf08_json["l"] = light;

    //    root.printTo(Serial);
    //    Serial.println();
    //}

    {
        const int IMU_BUFFER_SIZE = 350;
        StaticJsonBuffer<IMU_BUFFER_SIZE> buffer;
        JsonObject& root = buffer.createObject();
        JsonObject& imu_json = root.createNestedObject("imu");

        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        imu_json["ax"] = ax * accel_resolution - accelBias[0];
        imu_json["ay"] = ay * accel_resolution - accelBias[1];
        imu_json["az"] = az * accel_resolution - accelBias[2];

        imu_json["gx"] = gx * gyro_resolution;
        imu_json["gy"] = gy * gyro_resolution;
        imu_json["gz"] = gz * gyro_resolution;

        imu_json["mx"] = mx;
        imu_json["my"] = my;
        imu_json["mz"] = mz;
        root.printTo(Serial);
        Serial.println();
    }
}

//// I2C address is double of what is the address on TWI
//// Range of addresses on TWI - 0x70 - 0x7f
// void setAddressTwi(uint8_t address_twi)
//{
//  if (address_twi >= 0x70 && address_twi <= 0x7f)
//  {
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write((uint8_t)0xA0);
//    Wire.endTransmission();
//
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write((uint8_t)0xAA);
//    Wire.endTransmission();
//
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write((uint8_t)0xA5);
//    Wire.endTransmission();
//
//    Wire.beginTransmission(SRF_ADDRESS);
//    Wire.write(CMD);
//    Wire.write(address_twi * 2);
//    Wire.endTransmission();
//  }
//}
//
