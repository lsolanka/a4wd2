#include <iostream>

#include <sensor_reader/mpu9250.hpp>
#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/srf08.hpp>
#include <sensor_reader/string_line_reader.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "read_serial <port_name> [<filter>]" << std::endl;
        return EXIT_FAILURE;
    }

    const char* port_name = argv[1];
    const char* filter = nullptr;

    if (argc >= 3)
    {
        filter = argv[2];
    }

    serial_string_line_reader serial_reader(port_name, 9600);
    sensor_reader reader{serial_reader.get_stream()};
    int sample_num = 0;

    if (filter != nullptr)
    {
        std::cout << "using filter: " << filter << std::endl;

        if (std::string(filter) == "imu")
        {
            reader.add_sensor<sensors::mpu9250>([&](const auto& data) {
                std::cout << '\r' << std::setw(6) << sample_num << "; " << data
                          << std::flush;
                ++sample_num;
            });
        }
        else if (std::string(filter) == "srf08")
        {
            reader.add_sensor<sensors::srf08>(0x70, [&](const auto& data) {
                std::cout << std::setw(6) << sample_num << "; " << data << "; ";
                ++sample_num;
            });
            reader.add_sensor<sensors::srf08>(0x71, [&](const auto& data) {
                std::cout << std::setw(6) << sample_num << "; " << data << std::endl;
                ++sample_num;
            });
        }
    }

    reader.read_all();

    return 0;
}
