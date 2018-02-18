#pragma once

#include <exception>
#include <functional>
#include <istream>
#include <list>
#include <memory>
#include <string>

#include <sensor_reader/sensor.hpp>

namespace a4wd2::sensor_reader
{
/** Read sensor data and distribute the data to sensor subscribers registered
 * with the add_sensor method.
 *
 * It is possible to subscribe any number of subscribers.
 *
 * It is currently not possible to unsubscribe an already subscribed sensor
 * subscriber.
 */
class sensor_reader
{
  public:
    /** Create the reader with a specified input stream used for reading
     * line-based data. */
    sensor_reader(std::istream& input_stream) : m_input_stream(input_stream) {}
    /** Register a new sensor subscriber.
     *
     * This method creates a new instance of the sensor_t class and forwards
     * all the arguments to the constructor of the instance.
     *
     * The class of type sensor_t must inherit from
     * a4wd2::sensor_reader::sensor and must define a static const string
     * member called ID, which return the classification ID for the sensor
     * data.
     *
     * It is possible to subscribe more sensors with the same classifier ID.
     * The parsing will happen in the order of subscription (i.e. call to this
     * method).
     */
    template <typename sensor_t, typename... Args> void add_sensor(Args&&... args)
    {
        m_sensor_map.try_emplace(sensor_t::ID, sensor_list_t{});
        m_sensor_map[sensor_t::ID].push_back(std::make_shared<sensor_t>(args...));
    }

    /** Read all data line-by-line, until the end of stream is reached or the
     * stream is no longer good().
     *
     * All parsed data is passed for parsing to the registered sensor
     * subscribers.
     */
    void read_all();

  private:
    using sensor_list_t = std::list<std::shared_ptr<sensors::sensor>>;

    std::istream& m_input_stream;
    std::map<std::string, sensor_list_t> m_sensor_map;
};

} // namespace a4wd2::sensor_reader
