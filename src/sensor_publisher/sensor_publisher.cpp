#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/string_line_reader.hpp>
#include <sensor_reader/srf08.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

static constexpr int QUEUE_SIZE = 100;

sensor_msgs::Range range_from_srf08(const sensors::srf08::data_t& data)
{
    sensor_msgs::Range r;
    r.max_range = 6.;
    r.min_range = 0;
    r.radiation_type = sensor_msgs::Range::ULTRASOUND;
    r.field_of_view = M_PI / 4.;
    r.range = data.range / 100.;
    r.header.stamp = ros::Time::now();

    return r;
}

int main(int argc, char* argv[])
{
    if (argc <= 2)
    {
        std::cerr << "read_serial <port_name> <ros args...>" << std::endl;
        return EXIT_FAILURE;
    }
    ros::init(argc, argv, "sensor_publisher");

    const char *port_name = argv[1];
    serial_string_line_reader serial_reader(port_name, 9600);
    sensor_reader reader{serial_reader.get_stream()};

    ros::NodeHandle node_handle;
    ros::Publisher range_pub;
    range_pub =
        node_handle.advertise<sensor_msgs::Range>("sonar_ranger",
                QUEUE_SIZE, false);

    reader.add_sensor<sensors::srf08>(0x70, [&] (const auto& data)
        {
            sensor_msgs::Range r = range_from_srf08(data);
            r.header.frame_id = "range_fr";
            range_pub.publish(r);
            std::cout << data << "; ";
        }
    );
    reader.add_sensor<sensors::srf08>(0x71, [&] (const auto& data)
        {
            sensor_msgs::Range r = range_from_srf08(data);
            r.header.frame_id = "range_fl";
            range_pub.publish(r);
            std::cout << data << std::endl;
        }
    );
    reader.read_all();

    return EXIT_SUCCESS;
}
