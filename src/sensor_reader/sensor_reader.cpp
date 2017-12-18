#include <iostream>
#include <boost/log/trivial.hpp>

#include <nlohmann/json.hpp>

#include <sensor_reader/sensor_reader.hpp>

using boost::log::trivial::error;
using json = nlohmann::json;

namespace a4wd2::sensor_reader
{

void sensor_reader::read_all()
{
    while (m_input_stream.good() && !m_input_stream.eof())
    {
        std::string line;
        std::getline(m_input_stream, line);
        if (!m_input_stream.good() && line.size() == 0)
        {
            break;
        }

        json j;
        try
        {
            std::stringstream(line) >> j;
        }
        catch (json::exception& e)
        {
            BOOST_LOG_TRIVIAL(error) << "Parsing input line failed: " <<
                std::quoted(line, '\'') << "; message: " << e.what() << std::endl;
            continue;
        }

        if (!j.is_object())
        {
            BOOST_LOG_TRIVIAL(error) << "Top level JSON structure must be an object: "
                << std::quoted(line, '\'') << ". Skipping this line.";
            continue;
        }

        for (auto json_it = j.begin(); json_it != j.end(); ++json_it)
        {
            std::string classifier = json_it.key();
            auto sensor_it = m_sensor_map.find(classifier);
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

} // namespace a4wd2::sensor_reader
