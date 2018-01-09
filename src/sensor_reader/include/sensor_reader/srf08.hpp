#include <cstdint>
#include <nlohmann/json.hpp>

#include <sensor_reader/sensor.hpp>


namespace a4wd2::sensor_reader::sensors
{

/** Represents a Devantech SRF08 ultrasonic range finder.
 * Will provide an address on the I2C/TWI bus, range and light reading.
 */
class srf08 : public sensor
{
  public:
    struct data_t
    {
        uint8_t address; //!< Address on the I2C/TWI bus
        uint16_t range;  //!< Range reading in cm
        uint16_t light;  //!< Light reading (without units)
    };

    static const std::string ID; //!< String used for classification

    /** Create the sensor listener with a callback function.
     * @param addr Sensor address on the TWI bus. Only address mathing this
     *             will be published to on_data
     * @param on_data Functor to call when data arrives.
     */
    srf08(uint8_t addr, std::function<void (const data_t& data)> on_data);

    /** Parse json string and call on_data if the parsing is successful */
    bool parse(const nlohmann::json& j) override;

  private:
    std::function<void (const data_t& data)> m_on_data;
    int m_address;
};

void from_json(const nlohmann::json& j, srf08::data_t& data);

std::ostream& operator<<(std::ostream& stream, const srf08::data_t& data);


} // namespace a4wd2::sensor_reader::sensors
