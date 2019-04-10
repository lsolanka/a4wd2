#include <a4wd2/toolkit/odometry_provider.h>

#include <Eigen/Geometry>

namespace a4wd2
{

namespace toolkit
{

odometry_provider::odometry_provider(ros::NodeHandle nh, const std::string& topic_name)
    : m_nh(nh), m_odometry(std::nullopt)
{
    m_logger = spdlog::get("odometry_provider");
    if (!m_logger)
    {
        m_logger = spdlog::stdout_color_mt("odometry_provider");
    }

    m_subscriber =
            m_nh.subscribe(topic_name, QUEUE_SIZE, &odometry_provider::callback, this);
    m_logger->info("odometry_provider: subscribing to topic '{}'", topic_name);
}

bool odometry_provider::get_odometry(mrpt::math::TPose2D& pose,
                                     mrpt::math::TTwist2D& twist) const
{
    if (!m_odometry) return false;

    pose.x = m_odometry->pose.pose.position.x;
    pose.y = m_odometry->pose.pose.position.y;
    auto& ros_q = m_odometry->pose.pose.orientation;
    Eigen::Quaternionf q(ros_q.w, ros_q.x, ros_q.y, ros_q.z);
    pose.phi = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];  // get yaw

    twist.vx = m_odometry->twist.twist.linear.x;
    twist.vy = m_odometry->twist.twist.linear.y;
    twist.omega = m_odometry->twist.twist.angular.z;

    std::string pose_string;
    pose.asString(pose_string);
    m_logger->debug("getting odometry: {}", pose_string);

    return true;
}

}  // namespace toolkit

}  // namespace a4wd2
