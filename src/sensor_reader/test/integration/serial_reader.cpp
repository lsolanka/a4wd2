#include <iostream>

#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/string_line_reader.hpp>
#include <sensor_reader/srf08.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "read_serial <port_name>" << std::endl;
        return EXIT_FAILURE;
    }

    const char *port_name = argv[1];


    serial_string_line_reader serial_reader(port_name, 9600);
    sensor_reader reader{serial_reader.get_stream()};

    reader.add_sensor<sensors::srf08>(0x70, [&] (const auto& data)
        {
            std::cout << data << "; ";
        }
    );
    reader.add_sensor<sensors::srf08>(0x71, [&] (const auto& data)
        {
            std::cout << data << std::endl;
        }
    );
    reader.read_all();

    return 0;
}

