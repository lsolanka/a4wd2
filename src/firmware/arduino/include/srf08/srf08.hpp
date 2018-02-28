#pragma once
#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>

#include "ArduinoJson.hpp"

#include "I2Cdev/I2Cdev.h"

namespace srf08
{
namespace regs
{
namespace addr
{

static constexpr uint8_t CMD =
        0x00; // Command byte, values of 0 being sent with write have to be masked as
              // a byte to stop them being misinterpreted as NULL this is a bug with
              // arduino 1.0
static constexpr uint8_t LIGHT = 0x01; // Byte to read light sensor
static constexpr uint8_t RANGE = 0x02; // Byte for start of ranging data
static constexpr uint8_t GAIN = 0x01;

} // namespace addr
} // namespace regs

struct data_t
{
    uint8_t sensor_address;
    uint8_t range;
    uint8_t light;
};

//// Reduce range
// for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; ++sensor_idx)
//{
//    Wire.beginTransmission(SRF_BASE_ADDRESS + sensor_idx);
//    Wire.write(RANGE);
//    Wire.write(0x46);
//    Wire.endTransmission();

//    Wire.beginTransmission(SRF_BASE_ADDRESS + sensor_idx);
//    Wire.write(GAIN);
//    Wire.write(0x18);
//    Wire.endTransmission();
//}

void triggerRanging(uint8_t address)
{
    I2Cdev::writeByte(address, regs::addr::CMD, 0x51);
}

/** Get a range reading from a ranger at the specified address. **/
uint16_t read_range(uint8_t address)
{
    uint16_t range = 0;
    I2Cdev::readWord(address, regs::addr::RANGE, &range);
    return range;
}

/** Get light sensor reading from the sensor at the specified address **/
uint8_t read_light(uint8_t address)
{
    uint8_t light;
    I2Cdev::readByte(address, regs::addr::LIGHT, &light);
    return light;
}

//// I2C address is double of what is the address on TWI
//// Range of addresses on TWI - 0x70 - 0x7f
// void setAddressTwi(uint8_t address_twi)
//{
//    if (address_twi >= 0x70 && address_twi <= 0x7f)
//    {
//        Wire.beginTransmission(SRF_ADDRESS);
//        Wire.write(CMD);
//        Wire.write((uint8_t)0xA0);
//        Wire.endTransmission();
//
//        Wire.beginTransmission(SRF_ADDRESS);
//        Wire.write(CMD);
//        Wire.write((uint8_t)0xAA);
//        Wire.endTransmission();
//
//        Wire.beginTransmission(SRF_ADDRESS);
//        Wire.write(CMD);
//        Wire.write((uint8_t)0xA5);
//        Wire.endTransmission();
//
//        Wire.beginTransmission(SRF_ADDRESS);
//        Wire.write(CMD);
//        Wire.write(address_twi * 2);
//        Wire.endTransmission();
//    }
//}

template <typename Print>
void printTo(const data_t& data, Print& stream)
{
    using namespace ArduinoJson;

    static constexpr uint8_t BUFFER_SIZE = 100;
    StaticJsonBuffer<BUFFER_SIZE> buffer;
    JsonObject& root = buffer.createObject();
    JsonObject& srf08_json = root.createNestedObject("srf08");

    srf08_json["a"] = data.sensor_address;
    srf08_json["r"] = data.range;
    srf08_json["l"] = data.light;

    root.printTo(stream);
    stream.println();
}

template <uint8_t BASE_ADDRESS, uint8_t NUM_SENSORS, uint16_t RANGE_TIME_MS>
class sensor_list
{
    static_assert(RANGE_TIME_MS > 0, "RANGE_TIME_MS must be > 0");

  public:
    sensor_list()
        : last_range_time(0), ranging(false), current_sensor_idx(0),
          current_data_valid(false)
    {
    }

    void initiate_ranging()
    {
        uint8_t current_address = BASE_ADDRESS + current_sensor_idx;
        triggerRanging(current_address);
        ranging = true;
        last_range_time = millis();
    }

    void spin()
    {
        if (ranging)
        {
            if (millis() - last_range_time > RANGE_TIME_MS)
            {
                uint8_t current_address = BASE_ADDRESS + current_sensor_idx;

                current_data.sensor_address = current_address;
                current_data.range = read_range(current_address);
                current_data.light = read_light(current_address);
                current_data_valid = true;

                ++current_sensor_idx;
                if (current_sensor_idx >= NUM_SENSORS)
                {
                    current_sensor_idx = 0;
                }

                // Range immediately again for the next sensor
                initiate_ranging();
            }
        }
        else
        {
            initiate_ranging();
        }
    }

    bool has_new_reading() const { return current_data_valid; }

    const data_t& get_reading()
    {
        current_data_valid = false;
        return current_data;
    }

  private:
    uint32_t last_range_time;
    bool ranging;
    uint8_t current_sensor_idx;
    data_t current_data;
    bool current_data_valid;
};

} // namespace srf08
