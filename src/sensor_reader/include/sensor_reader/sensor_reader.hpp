#pragma once

#include <string>
#include <functional>
#include <exception>
#include <list>
#include <memory>

#include <sensor_reader/sensor.hpp>

namespace a4wd2::sensor_reader
{

template<typename link_ifc_t>
class sensor_reader
{
  public:

    /** Create the reader with a specified link interface. */
    sensor_reader(link_ifc_t& link_interface) :
        m_link_interface(link_interface)
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

    link_ifc_t& m_link_interface;
    std::map<std::string, sensor_list_t> m_sensor_map;
};

extern template class sensor_reader<std::stringstream>;

} // namespace a4wd2::sensor_reader
