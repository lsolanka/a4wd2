#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <nlohmann/json.hpp>

#include <sensor_reader/sensor.hpp>


namespace a4wd2::sensor_reader::sensors
{

/** Represents a Devantech SRF08 ultrasonic range finder.
 * Will provide an address on the I2C/TWI bus, range and light reading.
 */
class mpu9250 : public sensor
{
  public:
    struct data_t
    {
        float ax;  //!< Accelerometer X
        float ay;  //!< Accelerometer Y
        float az;  //!< Accelerometer Z

        int16_t gx;  //!< Gyro X
        int16_t gy;  //!< Gyro Y
        int16_t gz;  //!< Gyro Z

        int16_t mx;  //!< Magnetometer X
        int16_t my;  //!< Magnetometer Y
        int16_t mz;  //!< Magnetometer Z
    };

    static const std::string ID; //!< String used for classification

    /** Create the sensor listener with a callback function.
     * @param on_data Functor to call when data arrives.
     */
    mpu9250(std::function<void (const data_t& data)> on_data);

    /** Parse json string and call on_data if the parsing is successful */
    bool parse(const nlohmann::json& j) override;

  private:
    std::function<void (const data_t& data)> m_on_data;
};

void from_json(const nlohmann::json& j, mpu9250::data_t& data);

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t& data);


} // namespace a4wd2::sensor_reader::sensors
