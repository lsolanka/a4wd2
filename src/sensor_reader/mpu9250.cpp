#include <sensor_reader/mpu9250.hpp>
#include <boost/io/ios_state.hpp>

namespace a4wd2::sensor_reader::sensors
{

void from_json(const nlohmann::json& j, mpu9250::data_t& data)
{
    data.ax = j.at("ax").get<float>();
    data.ay = j.at("ay").get<float>();
    data.az = j.at("az").get<float>();

    data.gx = j.at("gx").get<int>();
    data.gy = j.at("gy").get<int>();
    data.gz = j.at("gz").get<int>();

    data.mx = j.at("mx").get<int>();
    data.my = j.at("my").get<int>();
    data.mz = j.at("mz").get<int>();
}


const std::string mpu9250::ID = "imu";

mpu9250::mpu9250(std::function<void (const data_t& data)> on_data)
    : m_on_data(on_data)
{
}

bool mpu9250::parse(const nlohmann::json& j)
{
    m_on_data(j);
    return true;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t& data)
{
    boost::io::ios_flags_saver ifs(stream);
    stream << std::dec <<
        "ax: " << std::setw(6) << std::fixed << std::setprecision(3) << data.ax << ", " <<
        "ay: " << std::setw(6) << std::fixed << std::setprecision(3) << data.ay << ", " <<
        "az: " << std::setw(6) << std::fixed << std::setprecision(3) << data.az << "; " <<

        "gx: " << std::setw(6) << data.gx << ", " <<
        "gy: " << std::setw(6) << data.gy << ", " <<
        "gz: " << std::setw(6) << data.gz << "; " <<

        "mx: " << std::setw(6) << data.mx << ", " <<
        "my: " << std::setw(6) << data.my << ", " <<
        "mz: " << std::setw(6) << data.mz << "; ";

    return stream;
}

} // namespace a4wd2::sensor_reader::sensors
