#pragma once

#include <nlohmann/json.hpp>

namespace a4wd2::sensor_reader::sensors
{
/** Sensor class which can parse a json object containing the sensor data. */
class sensor
{
  public:
    /** Parse the json object. The object will have classification ID stripped
     * off when this method is called.
     */
    virtual bool parse(const nlohmann::json& j) = 0;
};
}  // namespace a4wd2::sensor_reader::sensors
