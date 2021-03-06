#include <boost/io/ios_state.hpp>
#include <iostream>
#include <sensor_reader/srf08.hpp>

namespace a4wd2::sensor_reader::sensors
{
void from_json(const nlohmann::json& j, srf08::data_t& data)
{
    data.address = j.at("a").get<int>();
    data.range = j.at("r").get<int>();
    data.light = j.at("l").get<int>();
}

const std::string srf08::ID = "srf08";

srf08::srf08(uint8_t addr, std::function<void(const data_t& data)> on_data)
    : m_on_data(on_data), m_address(addr)
{
}

bool srf08::parse(const nlohmann::json& j)
{
    data_t data = j;
    if (data.address == m_address)
    {
        m_on_data(data);
        return true;
    }
    else
    {
        return false;
    }
}

std::ostream& operator<<(std::ostream& stream, const srf08::data_t& data)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << "a: " << std::showbase << std::hex << +data.address
           << ", "
              "r: "
           << std::dec << data.range
           << ", "
              "l: "
           << data.light;
    return stream;
}
}  // namespace a4wd2::sensor_reader::sensors
