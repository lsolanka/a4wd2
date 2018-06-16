#include <iostream>

#include <nlohmann/json.hpp>

#include <sensor_reader/sensor_reader.hpp>

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

        if (line.size() > 0 && line[0] == '#')
        {
            m_logger->info(line);
            continue;
        }

        json j;
        try
        {
            std::stringstream(line) >> j;
        }
        catch (json::exception& e)
        {
            m_logger->error("failed to parse input line: '{}'; message: {}", line,
                            e.what());
            continue;
        }

        if (!j.is_object())
        {
            m_logger->error(
                    "Top level JSON structure must be an object: '{}'. Skipping this "
                    "line.",
                    line);
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

}  // namespace a4wd2::sensor_reader
