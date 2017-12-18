#include <iostream>

#include <nlohmann/json.hpp>

#include <sensor_reader/sensor_reader.hpp>

using json = nlohmann::json;

namespace a4wd2::sensor_reader
{

template<typename link_ifc_t>
void sensor_reader<link_ifc_t>::read_all()
{
    while (m_link_interface.good())
    {
        std::string line;
        std::getline(m_link_interface, line);
        json j;
        try
        {
            std::stringstream(line) >> j;
        }
        catch (json::exception& e)
        {
            // FIXME: replace this with a log message
            std::cerr << "Parsing input line failed: " << std::quoted(line, '\'') <<
                "; message: " << e.what() << std::endl;
            continue;
        }

        for (auto json_it = j.begin(); json_it != j.end(); ++json_it)
        {
            auto sensor_it = m_sensor_map.find(json_it.key());
            if (sensor_it != m_sensor_map.end())
            {
                for (auto& sensor : sensor_it->second)
                {
                    sensor->parse(json_it.value());
                }
            }
        }
    }
}

template class sensor_reader<std::stringstream>;

}
