#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include <boost/math/constants/constants.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/io.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/velocity.hpp>

#include <roboclaw/io/io.hpp>

#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>

namespace a4wd2
{

class laser_scan_provider;
class odometry_provider;

namespace si
{

using boost::units::si::length;
using boost::units::si::meters;

using boost::units::si::meters_per_second;
using boost::units::si::velocity;

using boost::units::si::angular_velocity;
using boost::units::si::radians;
using boost::units::si::radians_per_second;
using boost::units::si::seconds;

}  // namespace si

struct robot_properties
{
    boost::units::quantity<si::length> wheel_radius;
    boost::units::quantity<si::length> axle_length;

    /** Quadrature pulses per wheel rotation. */
    int32_t quadrature_pulses_per_rotation;
};

class mrpt_nav_interface : public mrpt::nav::CRobot2NavInterface
{
public:
    mrpt_nav_interface(roboclaw::io::serial_controller& controller,
                       const robot_properties& props,
                       const laser_scan_provider& scan_provider,
                       const odometry_provider& odometry_provider);

    bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D& curPose,
                                 mrpt::math::TTwist2D& curVelGlobal,
                                 mrpt::system::TTimeStamp& timestamp,
                                 mrpt::math::TPose2D& curOdometry,
                                 std::string& frame_id) override;

    bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd) override;

    bool changeSpeedsNOP() override;

    bool stop(bool isEmergencyStop = true) override;

    mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override;

    mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override;

    mrpt::kinematics::CVehicleVelCmd::Ptr getAlignCmd(
            const double relative_heading_radians) override;

    bool senseObstacles(mrpt::maps::CSimplePointsMap& obstacles,
                        mrpt::system::TTimeStamp& timestamp) override;

    virtual void sendNavigationStartEvent() override { m_navigating = true; }
    virtual void sendNavigationEndEvent() override { m_navigating = false; }
    // virtual void sendWaypointReachedEvent(int waypoint_index, bool reached_nSkipped);
    // virtual void sendNewWaypointTargetEvent(int waypoint_index);
    // virtual void sendNavigationEndDueToErrorEvent();
    // virtual void sendWaySeemsBlockedEvent();
    void sendApparentCollisionEvent() override;
    // virtual void sendCannotGetCloserToBlockedTargetEvent();

    // virtual double getNavigationTime();
    // virtual void resetNavigationTimer();

private:
    struct motor_speed
    {
        boost::units::quantity<si::velocity> l;
        boost::units::quantity<si::velocity> r;

        motor_speed() : l(0 * si::meters_per_second), r(0 * si::meters_per_second){};
    };

    roboclaw::io::serial_controller& m_controller;
    motor_speed m_current_speed;
    robot_properties m_props;
    const laser_scan_provider& m_scan_provider;
    const odometry_provider& m_odometry_provider;

    bool m_navigating = false;
    bool m_reversing = false;
    std::chrono::steady_clock::time_point m_reverse_start_time;

    std::shared_ptr<spdlog::logger> m_logger;

    int32_t to_qpps(const boost::units::quantity<si::velocity>& v)
    {
        constexpr double PI = boost::math::constants::pi<double>();

        return ((double)m_props.quadrature_pulses_per_rotation * v / 2.0 / PI /
                m_props.wheel_radius) *
               si::seconds;
    }
};

}  // namespace a4wd2
