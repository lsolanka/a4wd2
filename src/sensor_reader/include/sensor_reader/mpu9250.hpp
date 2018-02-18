#pragma once

#include <cstdint>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>

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
        struct accel
        {
            float x; //!< Accelerometer X [g]
            float y; //!< Accelerometer Y [g]
            float z; //!< Accelerometer Z [g]
        };

        struct gyro
        {
            float x; //!< Gyro X [degrees/s]
            float y; //!< Gyro Y [degrees/s]
            float z; //!< Gyro Z [degrees/s]
        };

        struct mag
        {
            int16_t x; //!< Magnetometer X
            int16_t y; //!< Magnetometer Y
            int16_t z; //!< Magnetometer Z
        };

        accel a;
        gyro g;
        mag m;
    };

    static const std::string ID; //!< String used for classification

    /** Create the sensor listener with a callback function.
     * @param on_data Functor to call when data arrives.
     */
    mpu9250(std::function<void(const data_t& data)> on_data);

    /** Parse json string and call on_data if the parsing is successful */
    bool parse(const nlohmann::json& j) override;

  private:
    std::function<void(const data_t& data)> m_on_data;
};

void from_json(const nlohmann::json& j, mpu9250::data_t& data);

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::accel& a);
std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::gyro& g);
std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::mag& m);
std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t& data);

} // namespace a4wd2::sensor_reader::sensors
