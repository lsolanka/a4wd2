#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>

#include <boost/asio/serial_port.hpp>
#include <boost/function.hpp>
#include <cxxopts.hpp>

#include <a4wd2/config.h>
#include <a4wd2/motor_controller/init.h>
#include <a4wd2/control_command.h>

#include <roboclaw/io/io.hpp>
#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>

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

bool forward_movement_or_stopped(int32_t speed_right, int32_t speed_left)
{
    return speed_right + speed_left >= 0;
}

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
    if (!a4wd2::config::setup_verbosity(options_result))
    {
        std::cout << options.help();
        return EXIT_FAILURE;
    }

    std::string port_name = options_result["port"].as<std::string>();
    roboclaw::io::serial_controller controller(port_name, 0x80);
    a4wd2::motor_controller::init(controller);

    // StopTimer stopTimer(nh, 500ms, controller);
    int32_t speed_left = 0;
    int32_t speed_right = 0;

    boost::function<void(const a4wd2::control_command&)> callback =
            [&controller, &speed_left, &speed_right](const a4wd2::control_command& cmd) {
                switch (cmd.cmd)
                {
                    case a4wd2::control_command::FORWARD:
                    {
                        bool forward_before =
                                forward_movement_or_stopped(speed_right, speed_left);
                        speed_left += cmd.speed_qpps;
                        speed_right += cmd.speed_qpps;
                        bool forward_after =
                                forward_movement_or_stopped(speed_right, speed_left);
                        if (forward_after != forward_before)
                        {
                            std::swap(speed_left, speed_right);
                        }
                    }
                    break;

                    case a4wd2::control_command::BACKWARD:
                    {
                        bool forward_before =
                                forward_movement_or_stopped(speed_right, speed_left);
                        speed_left -= cmd.speed_qpps;
                        speed_right -= cmd.speed_qpps;
                        bool forward_after =
                                forward_movement_or_stopped(speed_right, speed_left);
                        if (forward_after != forward_before)
                        {
                            std::swap(speed_left, speed_right);
                        }
                    }
                    break;

                    case a4wd2::control_command::LEFT:
                        if (forward_movement_or_stopped(speed_right, speed_left))
                        {
                            speed_right += cmd.speed_qpps;
                            speed_left -= cmd.speed_qpps;
                        }
                        else
                        {
                            speed_right -= cmd.speed_qpps;
                            speed_left += cmd.speed_qpps;
                        }
                        break;

                    case a4wd2::control_command::RIGHT:
                    {
                        bool forward =
                                forward_movement_or_stopped(speed_right, speed_left);
                        ROS_INFO_STREAM("r: " << speed_right << ", l: " << speed_left
                                              << "forward: " << forward);
                        if (forward)
                        {
                            speed_right -= cmd.speed_qpps;
                            speed_left += cmd.speed_qpps;
                        }
                        else
                        {
                            speed_right += cmd.speed_qpps;
                            speed_left -= cmd.speed_qpps;
                        }
                    }
                    break;

                    case a4wd2::control_command::STOP:
                        speed_right = 0;
                        speed_left = 0;
                        break;

                    default:
                        ROS_ERROR_STREAM(log_prefix << "Unknown control command: "
                                                    << cmd.cmd << "Stopping the rover.");
                        controller.write(write_commands::m1_m2_drive_duty{0});
                        break;
                }
                if (cmd.cmd == a4wd2::control_command::STOP)
                {
                    controller.write(write_commands::m1_m2_drive_duty{0});
                }
                else
                {
                    controller.write(
                            write_commands::m1_m2_drive_qpps{speed_right, speed_left});
                }

                // stopTimer.clear();
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
