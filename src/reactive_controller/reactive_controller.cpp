#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <random>
#include <string>

#include <spdlog/spdlog.h>
#include <boost/math/constants/constants.hpp>
#include <cxxopts.hpp>

#include <roboclaw/io/io.hpp>
#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <a4wd2/config.h>
#include <a4wd2/motor_controller/init.h>

#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/initializer.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/nav.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/serialization/CSerializable.h>

#include "laser_scan_provider.h"
#include "mrpt_nav_interface.h"
#include "odometry_provider.h"

namespace si = a4wd2::si;
using namespace std::literals::chrono_literals;

namespace motor_controller = a4wd2::motor_controller;
using mrpt::config::CConfigFile;
using mrpt::nav::CReactiveNavigationSystem;
using mrpt::nav::TWaypoint;
using mrpt::nav::TWaypointSequence;

namespace write_commands = roboclaw::io::write_commands;

static constexpr double PI = boost::math::constants::pi<double>();

std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("reactive_controller");

/** Handling Ctrl-C **/
volatile std::atomic<bool> g_interruption_requested(false);
void signal_handler(int signal);

TWaypointSequence get_random_waypoints(const a4wd2::odometry_provider& odometry_provider)
{
    logger->debug("Request for new waypoint");

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

        logger->info("New waypoint distance: {}, theta: {}", distance, theta);
        logger->info("Current pose: x: {}, y: {}", pose.x, pose.y);
        logger->info("New waypoint list: x: {}, y: {}, theta: {}", x, y, theta);
        return waypoints;
    }
    else
    {
        logger->warn("Don't have odometry, returning empty waypoint list");
        return TWaypointSequence{};
    }
}

class nav_timer
{
public:
    nav_timer(ros::NodeHandle nh, std::chrono::milliseconds period,
              CReactiveNavigationSystem& nav_system,
              roboclaw::io::serial_controller& controller,
              a4wd2::odometry_provider& odometry)
        : m_nav_system(nav_system),
          m_controller(controller),
          m_odometry_provider(odometry)
    {
        m_timer = nh.createTimer(ros::Duration(period.count() * 1e-3),
                                 &nav_timer::callback, this);
        m_goal_publisher =
                nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", false);
    }

    void callback(const ros::TimerEvent&)
    {
        if (g_interruption_requested)
        {
            m_controller.write(write_commands::m1_drive_duty{0});
            ros::shutdown();
            return;
        }

        auto current_nav_state = m_nav_system.getCurrentState();

        if (current_nav_state == mrpt::nav::CAbstractNavigator::NAVIGATING)
        {
            m_nav_system.navigationStep();
        }
        else if (current_nav_state == mrpt::nav::CAbstractNavigator::NAV_ERROR ||
                 current_nav_state == mrpt::nav::CAbstractNavigator::IDLE)
        {
            auto waypoints = get_random_waypoints(m_odometry_provider);
            m_nav_system.navigateWaypoints(waypoints);
            m_nav_system.navigationStep();

            geometry_msgs::PoseStamped ros_goal;
            ros_goal.header.frame_id = "odom";
            ros_goal.pose.position.x = waypoints.waypoints[0].target.x;
            ros_goal.pose.position.y = waypoints.waypoints[0].target.y;

            Eigen::Quaternionf q;
            q = Eigen::AngleAxis<float>(waypoints.waypoints[0].target_heading, Eigen::Vector3f::UnitZ());
            ros_goal.pose.orientation.x = q.x();
            ros_goal.pose.orientation.y = q.y();
            ros_goal.pose.orientation.z = q.z();
            ros_goal.pose.orientation.w = q.w();
            m_goal_publisher.publish(ros_goal);
        }
    }

    void clear()
    {
        m_timer.stop();
        m_timer.start();
    }

private:
    ros::Timer m_timer;
    ros::Publisher m_goal_publisher;
    CReactiveNavigationSystem& m_nav_system;
    roboclaw::io::serial_controller& m_controller;
    a4wd2::odometry_provider& m_odometry_provider;
};

void register_mrpt_classes()
{
    using namespace mrpt;
    using namespace mrpt::nav;
    using namespace mrpt::math;

    // PTGs:
    registerClass(CLASS_ID(CPTG_DiffDrive_C));
    registerClass(CLASS_ID(CPTG_DiffDrive_alpha));
    registerClass(CLASS_ID(CPTG_DiffDrive_CCS));
    registerClass(CLASS_ID(CPTG_DiffDrive_CC));
    registerClass(CLASS_ID(CPTG_DiffDrive_CS));
    registerClass(CLASS_ID(CPTG_Holo_Blend));

    // Logs:
    registerClass(CLASS_ID(CLogFileRecord));
    registerClass(CLASS_ID(CLogFileRecord_ND));
    registerClass(CLASS_ID(CLogFileRecord_VFF));
    registerClass(CLASS_ID(CLogFileRecord_FullEval));

    // Holo methods:
    registerClass(CLASS_ID(CHolonomicVFF));
    registerClass(CLASS_ID(CHolonomicND));
    registerClass(CLASS_ID(CHolonomicFullEval));

    // Motion choosers:
    registerClass(CLASS_ID(CMultiObjectiveMotionOptimizerBase));
    registerClass(CLASS_ID(CMultiObjMotionOpt_Scalarization));

    registerClass(CLASS_ID(CPolygon));
}

int main(int argc, char** argv)
{
    register_mrpt_classes();

    ros::init(argc, argv, "ros_controller");
    ros::NodeHandle nh;
    ros::Subscriber control_subscriber;

    // clang-format off
    cxxopts::Options options("reactive_controller", "ROS-based A4WD2 reactive movement controller");
    options.add_options()
        ("v,verbosity",
            "Verbosity level [trace, debug, info, warning, error, fatal]",
            cxxopts::value<std::string>()->default_value("disabled"))
        ("p,port",
            "Serial port to use to connect to roboclaw",
            cxxopts::value<std::string>()->default_value("/dev/roboclaw-2x7A"))
        ("c,config",
            "Path to the config file for the reactive controller",
            cxxopts::value<std::string>());
    // clang-format on
    const char** parse_argv = const_cast<const char**>(argv);
    auto options_result = options.parse(argc, parse_argv);

    if (options_result.count("help"))
    {
        std::cout << options.help();
        return EXIT_SUCCESS;
    }
    if (!a4wd2::config::setup_verbosity(options_result))
    {
        std::cout << options.help();
        return EXIT_FAILURE;
    }
    if (!options_result.count("config"))
    {
        std::cerr << "--config parameter is required!" << std::endl;
        std::cerr << options.help();
        return EXIT_FAILURE;
    }

    std::string port_name = options_result["port"].as<std::string>();
    roboclaw::io::serial_controller controller(port_name, 0x80);
    motor_controller::init(controller);

    a4wd2::robot_properties robot_props;
    robot_props.wheel_radius = .05 * si::meters;
    robot_props.axle_length = 0.29 * si::meters;
    robot_props.quadrature_pulses_per_rotation = 12000;

    a4wd2::laser_scan_provider scan_provider(nh, "/scan");
    a4wd2::odometry_provider odometry_provider(nh, "/odom_rf2o");
    a4wd2::mrpt_nav_interface nav_interface(controller, robot_props, scan_provider,
                                            odometry_provider);
    CReactiveNavigationSystem nav_system(nav_interface);
    nav_system.loadConfigFile(CConfigFile(options_result["config"].as<std::string>()));
    nav_system.initialize();
    nav_timer timer(nh, 100ms, nav_system, controller, odometry_provider);

    boost::function<void(const geometry_msgs::PoseStampedConstPtr&)> new_pose_cb =
            [&](const geometry_msgs::PoseStampedConstPtr& pose) {
                auto& ros_q = pose->pose.orientation;
                Eigen::Quaternionf q(ros_q.w, ros_q.x, ros_q.y, ros_q.z);
                float theta = q.toRotationMatrix().eulerAngles(0, 1, 2)[2];  // get yaw

                TWaypointSequence waypoints;
                waypoints.waypoints = {TWaypoint(
                        pose->pose.position.x, pose->pose.position.y, 0.1, false, theta)};
                nav_system.navigateWaypoints(waypoints);
            };
    auto goal_server = nh.subscribe("/move_base_simple/goal", 10, new_pose_cb);

    ros::spin();
}

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        g_interruption_requested = true;
    }
}
