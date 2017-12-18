#pragma once

#include <string>
#include <functional>
#include <exception>
#include <list>
#include <memory>
#include <istream>

#include <sensor_reader/sensor.hpp>

namespace a4wd2::sensor_reader
{

class sensor_reader
{
  public:

    /** Create the reader with a specified input stream used for reading
     * line-based data. */
    sensor_reader(std::istream& input_stream) :
        m_input_stream(input_stream)
    {
    }

    template<typename sensor_t, typename... Args>
    void add_sensor(Args&&... args)
    {
        m_sensor_map.try_emplace(sensor_t::ID, sensor_list_t{});
        m_sensor_map[sensor_t::ID].push_back(std::make_shared<sensor_t>(
                args...));
    }

    void read_all();

  private:
    using sensor_list_t = std::list<std::shared_ptr<sensors::sensor>>;

    std::istream& m_input_stream;
    std::map<std::string, sensor_list_t> m_sensor_map;
};

} // namespace a4wd2::sensor_reader
