#include <cstdlib>

#include <ros/ros.h>
#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/srf08.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sensor_publisher");
    return EXIT_SUCCESS;
}
