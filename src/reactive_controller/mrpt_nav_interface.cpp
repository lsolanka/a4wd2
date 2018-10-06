#include <chrono>
#include <cstdint>

#include <fmt/format.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <ros/ros.h>
#include <boost/math/constants/constants.hpp>

#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>

#include "laser_scan_provider.h"
#include "mrpt_nav_interface.h"
#include "odometry_provider.h"

using namespace std::literals::chrono_literals;

namespace read_commands = roboclaw::io::read_commands;
namespace write_commands = roboclaw::io::write_commands;
using mrpt::kinematics::CVehicleVelCmd_DiffDriven;

static constexpr double PI = boost::math::constants::pi<double>();

namespace a4wd2
{

mrpt_nav_interface::mrpt_nav_interface(roboclaw::io::serial_controller& controller,
                                       const robot_properties& props,
                                       const laser_scan_provider& scan_provider,
                                       const odometry_provider& odometry_provider)
    : m_controller{controller},
      m_props(props),
      m_scan_provider(scan_provider),
      m_odometry_provider(odometry_provider)
{
    m_logger = spdlog::get("mrpt_nav_interface");
    if (!m_logger)
    {
        m_logger = spdlog::stdout_color_mt("mrpt_nav_interface");
    }
}

bool mrpt_nav_interface::getCurrentPoseAndSpeeds(mrpt::math::TPose2D& curPose,
                                                 mrpt::math::TTwist2D& curVelGlobal,
                                                 mrpt::system::TTimeStamp& timestamp,
                                                 mrpt::math::TPose2D& curOdometry,
                                                 std::string& frame_id)
{
    if (!m_odometry_provider.get_odometry(curOdometry, curVelGlobal))
    {
        return false;
    }
    curPose = curOdometry;
    timestamp = mrpt::system::now();
    // frame_id is left default

    return true;
}

bool mrpt_nav_interface::changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
{
    auto diff_cmd = dynamic_cast<const CVehicleVelCmd_DiffDriven*>(&vel_cmd);
    if (!diff_cmd)
    {
        ROS_ERROR(
                "[changeSpeeds()] velocity command type must be "
                "CVehicleVelCmd_DiffDriven");
        return false;
    }

    m_current_speed.l = diff_cmd->lin_vel * si::meters_per_second -
                        0.5 * diff_cmd->ang_vel / si::seconds * m_props.axle_length;
    m_current_speed.r = diff_cmd->lin_vel * si::meters_per_second +
                        0.5 * diff_cmd->ang_vel / si::seconds * m_props.axle_length;

    int32_t left_qpps = to_qpps(m_current_speed.l);
    int32_t right_qpps = to_qpps(m_current_speed.r);

    m_logger->debug("sending qpps command: m1: {}, m2: {}", right_qpps, left_qpps);
    m_controller.write(write_commands::m1_m2_drive_qpps{right_qpps, left_qpps});

    return true;
}

bool mrpt_nav_interface::changeSpeedsNOP() { return true; }

bool mrpt_nav_interface::stop(bool isEmergencyStop)
{
    m_controller.write(write_commands::m1_m2_drive_duty{0});
    m_current_speed.l = 0;
    m_current_speed.r = 0;
    return true;
}

mrpt::kinematics::CVehicleVelCmd::Ptr mrpt_nav_interface::getEmergencyStopCmd()
{
    auto cmd = mrpt::make_aligned_shared<CVehicleVelCmd_DiffDriven>();
    cmd->setToStop();
    return cmd;
}

mrpt::kinematics::CVehicleVelCmd::Ptr mrpt_nav_interface::getStopCmd()
{
    return getEmergencyStopCmd();
}

bool mrpt_nav_interface::senseObstacles(mrpt::maps::CSimplePointsMap& obstacles,
                                        mrpt::system::TTimeStamp& timestamp)
{
    mrpt::obs::CObservation2DRangeScan scan;
    if (!m_scan_provider.get_range_scan(scan))
    {
        m_logger->warn("no range scan available when sensing obstacles");
        return false;
    }

    obstacles.insertionOptions.addToExistingPointsMap = false;
    obstacles.loadFromRangeScan(scan);
    timestamp = mrpt::system::now();

    // std::vector<float> xs, ys, zs;
    // obstacles.getAllPoints(xs, ys, zs);
    // std::string msg = "[";
    // for (int i = 0; i < xs.size(); ++i)
    //{
    //    msg += fmt::format("x: {}, y: {}, z: {}, \n", xs[i], ys[i], zs[i]);
    //}
    // msg += "]";
    // m_logger->debug("obstacles: {}", msg);

    return true;
}

mrpt::kinematics::CVehicleVelCmd::Ptr mrpt_nav_interface::getAlignCmd(
        const double relative_heading_radians)
{
    using boost::math::constants::pi;

    auto cmd = mrpt::make_aligned_shared<CVehicleVelCmd_DiffDriven>();
    cmd->lin_vel = 0;
    cmd->ang_vel = std::min(10 * std::abs(relative_heading_radians), 0.4 * pi<double>());
    cmd->ang_vel *= relative_heading_radians > 0 ? 1 : -1;

    return cmd;
}

void mrpt_nav_interface::sendApparentCollisionEvent()
{
    //if (!m_navigating) return;

    m_logger->warn("Apparent collision event!!");

    //mrpt::kinematics::CVehicleVelCmd_DiffDriven vel_cmd;
    //vel_cmd.ang_vel = 0;
    //vel_cmd.lin_vel = -0.2;  // m/s
    //changeSpeeds(vel_cmd);

    //std::this_thread::sleep_until(std::chrono::steady_clock::now() + 2s);

    //vel_cmd.ang_vel = 0;
    //vel_cmd.lin_vel = 0;
    //changeSpeeds(vel_cmd);
}

}  // namespace a4wd2
