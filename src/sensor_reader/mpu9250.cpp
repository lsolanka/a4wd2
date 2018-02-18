#include <boost/io/ios_state.hpp>
#include <sensor_reader/mpu9250.hpp>

namespace a4wd2::sensor_reader::sensors
{
void from_json(const nlohmann::json& j, mpu9250::data_t& data)
{
    data.a.x = j.at("ax").get<float>();
    data.a.y = j.at("ay").get<float>();
    data.a.z = j.at("az").get<float>();

    data.g.x = j.at("gx").get<int>();
    data.g.y = j.at("gy").get<int>();
    data.g.z = j.at("gz").get<int>();

    data.m.x = j.at("mx").get<int>();
    data.m.y = j.at("my").get<int>();
    data.m.z = j.at("mz").get<int>();
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

    stream << "gx: " << std::setw(6) << g.x << ", "
           << "gy: " << std::setw(6) << g.y << ", "
           << "gz: " << std::setw(6) << g.z;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t::mag& m)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << "mx: " << std::setw(6) << m.x << ", "
           << "my: " << std::setw(6) << m.y << ", "
           << "mz: " << std::setw(6) << m.z;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const mpu9250::data_t& data)
{
    boost::io::ios_flags_saver ifs(stream);

    stream << data.a << "; " << data.g << "; " << data.m;

    return stream;
}

} // namespace a4wd2::sensor_reader::sensors
