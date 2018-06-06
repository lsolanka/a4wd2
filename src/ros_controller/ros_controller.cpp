#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <cxxopts.hpp>

#include <a4wd2/control_command.h>

#include <roboclaw/io/io.hpp>
#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>
#include <roboclaw/logging.hpp>

#include <ros/ros.h>

using namespace std::literals::chrono_literals;

namespace read_commands = roboclaw::io::read_commands;
namespace write_commands = roboclaw::io::write_commands;
using roboclaw::io::serial_controller;

volatile std::atomic<bool> interruption_requested(false);

static const std::string log_prefix = "ros_controller - ";
static constexpr int QUEUE_SIZE = 100;

void signal_handler(int signal);
void interruption_point(serial_controller& controller);
bool setup_verbosity(const cxxopts::ParseResult& options);

void read_info(roboclaw::io::serial_controller& controller);

class StopTimer
{
  public:
    StopTimer(ros::NodeHandle nh, std::chrono::milliseconds timeout,
              serial_controller& controller)
        : m_controller(controller)
    {
        m_timer = nh.createTimer(ros::Duration(timeout.count() * 1e-3),
                                 &StopTimer::callback, this);
    }

    void callback(const ros::TimerEvent&)
    {
        ROS_DEBUG_STREAM(log_prefix << "Command timeout expired. Stopping rover");
        m_controller.write(write_commands::m1_m2_drive_duty{0});
    }

    void clear()
    {
        m_timer.stop();
        m_timer.start();
    }

  private:
    ros::Timer m_timer;
    serial_controller& m_controller;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_controller");
    ros::NodeHandle nh;
    ros::Subscriber control_subscriber;

    // clang-format off
    cxxopts::Options options("ros_controller", "ROS-based A4WD2 movement controller");
    options.add_options()
        ("v,verbosity",
            "Verbosity level [trace, debug, info, warning, error, fatal]",
            cxxopts::value<std::string>()->default_value("disabled"))
        ("p,port",
            "Serial port to use to connect to roboclaw",
            cxxopts::value<std::string>()->default_value("/dev/roboclaw-2x7A"));
    // clang-format on
    const char** parse_argv = const_cast<const char**>(argv);
    auto options_result = options.parse(argc, parse_argv);

    if (options_result.count("help"))
    {
        std::cout << options.help();
        return EXIT_SUCCESS;
    }
    if (!setup_verbosity(options_result))
    {
        std::cout << options.help();
        return EXIT_FAILURE;
    }

    std::string port_name = options_result["port"].as<std::string>();
    roboclaw::io::serial_controller controller(port_name, 0x80);
    read_info(controller);
    StopTimer stopTimer(nh, 500ms, controller);

    boost::function<void(const a4wd2::control_command&)> callback =
            [&controller,&stopTimer](const a4wd2::control_command& cmd) {
                switch (cmd.cmd)
                {
                    case a4wd2::control_command::FORWARD:
                        controller.write(write_commands::m1_m2_drive_qpps{
                                cmd.speed_qpps, cmd.speed_qpps});
                        break;

                    case a4wd2::control_command::BACKWARD:
                        controller.write(write_commands::m1_m2_drive_qpps{
                                -cmd.speed_qpps, -cmd.speed_qpps});
                        break;

                    case a4wd2::control_command::LEFT:
                        controller.write(write_commands::m1_m2_drive_qpps{
                                cmd.speed_qpps, -cmd.speed_qpps});
                        break;

                    case a4wd2::control_command::RIGHT:
                        controller.write(write_commands::m1_m2_drive_qpps{
                                -cmd.speed_qpps, cmd.speed_qpps});
                        break;

                    default:
                        ROS_ERROR_STREAM(log_prefix << "Unknown control command: "
                                                    << cmd.cmd << "Stopping the rover.");
                        controller.write(write_commands::m1_m2_drive_duty{0});
                        break;
                }

                stopTimer.clear();
            };

    std::signal(SIGINT, signal_handler);
    control_subscriber =
            nh.subscribe<a4wd2::control_command>("/control", QUEUE_SIZE, callback);

    // Make sure we stop when we get CTRL+C
    boost::function<void(const ros::TimerEvent&)> interrupt_check_cb =
            [&controller](const ros::TimerEvent&) {
                if (interruption_requested)
                {
                    controller.write(write_commands::m1_m2_drive_duty{0});
                    ros::shutdown();
                }
            };
    ros::Timer interrupt_timer = nh.createTimer(ros::Duration(0.1), interrupt_check_cb);

    ros::spin();

    return EXIT_SUCCESS;
}

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        interruption_requested = true;
    }
}

void read_info(roboclaw::io::serial_controller& controller)
{
    ROS_INFO_STREAM(log_prefix << "Firmware version: "
                               << controller.read<read_commands::firmware_version>());

    ROS_INFO_STREAM(log_prefix << "Main battery voltage: "
                               << controller.read<read_commands::main_battery_voltage>());
    ROS_INFO_STREAM(
            log_prefix << "Logic battery voltage: "
                       << controller.read<read_commands::logic_battery_voltage>());

    controller.write(write_commands::m1_encoder_mode{true, false});
    controller.write(write_commands::m2_encoder_mode{true, false});

    ROS_INFO_STREAM(
            log_prefix << "Encoder mode: "
                       << get_string(controller.read<read_commands::encoder_mode>()));

    controller.write(write_commands::m1_velocity_pid{1.f, 0.5f, 0.25f, 42000});
    controller.write(write_commands::m2_velocity_pid{1.f, 0.5f, 0.25f, 42000});

    ROS_INFO_STREAM(
            log_prefix << "M1 velocity PID: "
                       << get_string(controller.read<read_commands::m1_velocity_pid>()));
    ROS_INFO_STREAM(
            log_prefix << "M2 velocity PID: "
                       << get_string(controller.read<read_commands::m2_velocity_pid>()));
}

BOOST_LOG_GLOBAL_LOGGER_INIT(logger, logger_t)
{
    namespace expr = boost::log::expressions;
    logger_t lg;
    boost::log::add_common_attributes();
    boost::log::add_file_log(
            boost::log::keywords::file_name = "log.txt",
            boost::log::keywords::format =
                    (expr::stream
                     << expr::format_date_time<boost::posix_time::ptime>(
                                "TimeStamp", "%Y-%m-%d %H:%M:%S")
                     << " ["
                     << expr::attr<boost::log::trivial::severity_level>("Severity")
                     << "] " << expr::smessage));
    boost::log::add_console_log(
            std::cout,
            boost::log::keywords::format =
                    (expr::stream
                     << expr::format_date_time<boost::posix_time::ptime>(
                                "TimeStamp", "%Y-%m-%d %H:%M:%S")
                     << " ["
                     << expr::attr<boost::log::trivial::severity_level>("Severity")
                     << "] " << expr::smessage));
    return lg;
}

bool setup_verbosity(const cxxopts::ParseResult& options)
{
    int min_verbosity_level = boost::log::trivial::fatal;

    if (options.count("verbosity"))
    {
        auto verbosity = options["verbosity"].as<std::string>();
        if (verbosity == "disabled")
        {
            min_verbosity_level = boost::log::trivial::error;
        }
        else if (verbosity == "debug")
        {
            min_verbosity_level = boost::log::trivial::debug;
        }
        else if (verbosity == "info")
        {
            min_verbosity_level = boost::log::trivial::info;
        }
        else if (verbosity == "warning")
        {
            min_verbosity_level = boost::log::trivial::warning;
        }
        else if (verbosity == "error")
        {
            min_verbosity_level = boost::log::trivial::error;
        }
        else if (verbosity == "fatal")
        {
            min_verbosity_level = boost::log::trivial::fatal;
        }
        else
        {
            return false;
        }
    }

    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        min_verbosity_level);

    return true;
}

