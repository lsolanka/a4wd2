#include <boost/io/ios_state.hpp>
#include <sensor_reader/mpu9250.hpp>

namespace a4wd2::sensor_reader::sensors
{
void from_json(const nlohmann::json& j, mpu9250::data_t& data)
{
    data.a.x = j.at("ax").get<float>();
    data.a.y = j.at("ay").get<float>();
    data.a.z = j.at("az").get<float>();

    data.g.x = j.at("gx").get<float>();
    data.g.y = j.at("gy").get<float>();
    data.g.z = j.at("gz").get<float>();

    data.m.x = j.at("mx").get<float>();
    data.m.y = j.at("my").get<float>();
    data.m.z = j.at("mz").get<float>();
}

const std::string mpu9250::ID = "imu";

mpu9250::mpu9250(std::function<void(const data_t& data)> on_data) : m_on_data(on_data) {}

bool mpu9250::parse(const nlohmann::json& j)
{
    m_on_data(j);
    return true;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::accel& a)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << std::dec << "ax: " << std::setw(6) << std::fixed << std::setprecision(3)
           << a.x << ", "
           << "ay: " << std::setw(6) << std::fixed << std::setprecision(3) << a.y << ", "
           << "az: " << std::setw(6) << std::fixed << std::setprecision(3) << a.z;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::gyro& g)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << std::dec << "gx: " << std::setw(6) << std::fixed << std::setprecision(3)
           << g.x << ", "
           << "gy: " << std::setw(6) << std::fixed << std::setprecision(3) << g.y << ", "
           << "gz: " << std::setw(6) << std::fixed << std::setprecision(3) << g.z;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::mag& m)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << std::dec << "mx: " << std::setw(6) << std::fixed << std::setprecision(3)
           << m.x << ", "
           << "my: " << std::setw(6) << std::fixed << std::setprecision(3) << m.y << ", "
           << "mz: " << std::setw(6) << std::fixed << std::setprecision(3) << m.z;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t& data)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << data.a << "; " << data.g << "; " << data.m;

    return stream;
}

}  // namespace a4wd2::sensor_reader::sensors
