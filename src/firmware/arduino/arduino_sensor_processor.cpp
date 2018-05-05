#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include "mpu9250/timed_mpu9250.hpp"
#include "srf08/srf08.hpp"

#define SRF_BASE_ADDRESS 0x70  // Address of the SRF08
#define NUM_SENSORS 2

static constexpr uint8_t MPU9250_ADDRESS = 0x68;

namespace addr = mpu9250::regs::addr;

static constexpr uint16_t IMU_READ_PERIOD_MS = 50;
static constexpr uint16_t IMU_JSON_BUFFER_SIZE = 300;
mpu9250::timed_mpu9250<IMU_JSON_BUFFER_SIZE, IMU_READ_PERIOD_MS> imu;

static constexpr uint16_t SRF08_RANGE_TIME_MS = 50.;
srf08::sensor_list<SRF_BASE_ADDRESS, NUM_SENSORS, SRF08_RANGE_TIME_MS> srf08_sensors;

void setup()
{
    Wire.begin();
    delay(100);

    Serial.begin(57600);
    Serial.println("# arduino_sensor_processor: start");

    srf08_sensors.reduce_range();

    imu.init_and_calibrate(false);
    imu.print_whoami_to_serial();
}

void loop()
{
    unsigned long start_send_time = millis();

    srf08_sensors.spin();
    if (srf08_sensors.has_new_reading())
    {
        srf08::printTo(srf08_sensors.get_reading(), Serial);
    }

    imu.spin();
}
