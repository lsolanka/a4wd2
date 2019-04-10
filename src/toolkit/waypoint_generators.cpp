#include <random>
#include <boost/math/constants/constants.hpp>

#include <a4wd2/toolkit/waypoint_generators.h>

using mrpt::nav::TWaypoint;
using mrpt::nav::TWaypointSequence;

static constexpr double PI = boost::math::constants::pi<double>();

namespace a4wd2::toolkit
{

TWaypointSequence uniform_random_waypoint_generator::operator()(
        const odometry_provider& odometry_provider) const
{
    m_logger->debug("Request for new waypoint");

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distance_sample(1., 3.0);
    std::uniform_real_distribution<> theta_sample(-PI, PI);

    mrpt::math::TPose2D pose;
    mrpt::math::TTwist2D twist;
    bool have_odometry = odometry_provider.get_odometry(pose, twist);

    if (have_odometry)
    {
        double theta = theta_sample(gen);
        double distance = distance_sample(gen);

        double x = pose.x + distance * std::cos(theta);
        double y = pose.y + distance * std::sin(theta);

        TWaypointSequence waypoints;
        waypoints.waypoints = {TWaypoint(x, y, 0.1, false, theta)};

        m_logger->info("New waypoint distance: {}, theta: {}", distance, theta);
        m_logger->info("Current pose: x: {}, y: {}", pose.x, pose.y);
        m_logger->info("New waypoint list: x: {}, y: {}, theta: {}", x, y, theta);
        return waypoints;
    }
    else
    {
        m_logger->warn("Don't have odometry, returning empty waypoint list");
        return TWaypointSequence{};
    }
}

}  // namespace a4wd2::toolkit
