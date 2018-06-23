#include <cmath>
#include <iostream>

#include <spdlog/spdlog.h>
#include <fmt/ostream.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include <sensor_reader/mpu9250.hpp>
#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/srf08.hpp>
#include <sensor_reader/string_line_reader.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

static constexpr int QUEUE_SIZE = 100;
static constexpr double deg_to_rad = 2 * M_PI / 360.;
static constexpr double one_g_in_m_per_s2 = 9.81;

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

sensor_msgs::Imu imu_from_mpu9250(const sensors::mpu9250::data_t& data)
{
    sensor_msgs::Imu imu;

    // If you have no estimate for one of the data elements (e.g. your IMU doesn't produce
    // an orientation estimate), please set element 0 of the associated covariance matrix
    // to -1 If you are interpreting this message, please check for a value of -1 in the
    // first element of each covariance matrix, and disregard the associated estimate.
    imu.orientation_covariance[0] = -1;

    // rad/s
    imu.angular_velocity.x = deg_to_rad * data.g.x;
    imu.angular_velocity.y = deg_to_rad * data.g.y;
    imu.angular_velocity.z = deg_to_rad * data.g.z;

    // acceleration in m/s^2
    imu.linear_acceleration.x = data.a.x * one_g_in_m_per_s2;
    imu.linear_acceleration.y = data.a.y * one_g_in_m_per_s2;
    imu.linear_acceleration.z = data.a.z * one_g_in_m_per_s2;

    return imu;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "sensor_publisher <port_name> <ros args...>" << std::endl;
        return EXIT_FAILURE;
    }
    ros::init(argc, argv, "sensor_publisher");

    auto lg_sensor_reader = spdlog::stdout_color_mt("sensor_reader");
    auto logger = spdlog::stdout_color_mt("sensor_publisher");
    spdlog::set_async_mode(8192);

    const char* port_name = argv[1];
    serial_string_line_reader serial_reader(port_name, 9600);
    sensor_reader reader{serial_reader.get_stream()};

    ros::NodeHandle node_handle;
    ros::Publisher range_pub;
    ros::Publisher imu_pub;
    range_pub =
            node_handle.advertise<sensor_msgs::Range>("sonar_ranger", QUEUE_SIZE, false);
    imu_pub = node_handle.advertise<sensor_msgs::Imu>("imu", QUEUE_SIZE, false);

    // Sonar data
    reader.add_sensor<sensors::srf08>(0x70, [&](const auto& data) {
        sensor_msgs::Range r = range_from_srf08(data);
        r.header.stamp = ros::Time::now();
        r.header.frame_id = "range_fr";
        range_pub.publish(r);
        logger->debug("sonar data: {}", data);
    });
    reader.add_sensor<sensors::srf08>(0x71, [&](const auto& data) {
        sensor_msgs::Range r = range_from_srf08(data);
        r.header.stamp = ros::Time::now();
        r.header.frame_id = "range_fl";
        range_pub.publish(r);
        logger->debug("sonar data: {}", data);
    });

    // MPU9250 IMU + MAG data (publishing only IMU data)
    reader.add_sensor<sensors::mpu9250>([&](const auto& data) {
        sensor_msgs::Imu imu_data = imu_from_mpu9250(data);
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu_frame";
        imu_pub.publish(imu_data);
        logger->debug("IMU data: {}", data);
    });

    reader.read_all();

    return EXIT_SUCCESS;
}
