#pragma once

#include <memory>
#include <optional>
#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace a4wd2
{

namespace toolkit
{

class odometry_provider
{
public:
    odometry_provider(ros::NodeHandle nh, const std::string& topic_name);

    bool get_odometry(mrpt::math::TPose2D& pose, mrpt::math::TTwist2D& twist) const;

    odometry_provider(const odometry_provider&) = delete;
    odometry_provider& operator=(const odometry_provider&) = delete;

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_subscriber;
    std::optional<nav_msgs::Odometry> m_odometry;

    std::shared_ptr<spdlog::logger> m_logger;

    static constexpr uint32_t QUEUE_SIZE = 10;

    void callback(const nav_msgs::OdometryConstPtr& msg) { m_odometry = *msg; }
};

}  // namespace toolkit

}  // namespace a4wd2
