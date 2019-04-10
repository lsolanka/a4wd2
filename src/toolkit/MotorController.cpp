#include <a4wd2/toolkit/MotorController.h>

namespace write_commands = roboclaw::io::write_commands;
namespace read_commands = roboclaw::io::read_commands;

namespace a4wd2::toolkit
{

MotorController::MotorController(ros::NodeHandle& nh, std::chrono::milliseconds interval,
                                 roboclaw::io::serial_controller& controller,
                                 const Parameters& params)
    : m_nh(nh), m_controller(controller), m_params(params)
{
    m_right_qpps_pub = m_nh.advertise<std_msgs::Int64>("/wheel_speed/qpps_right", false);
    m_left_qpps_pub = m_nh.advertise<std_msgs::Int64>("/wheel_speed/qpps_left", false);
    m_timer = nh.createTimer(ros::Duration(interval.count() / 1000.),
                             &MotorController::cb, this);

    auto m_logger = spdlog::get("MotorController");
    if (!m_logger) m_logger = spdlog::stdout_color_mt("MotorController");
}

void MotorController::set_target_qpps_left(int32_t qpps)
{
    m_target_left = qpps;
    if (m_stop && qpps != 0) m_stop = false;
}
void MotorController::set_target_qpps_right(int32_t qpps)
{
    m_target_right = qpps;
    if (m_stop && qpps != 0) m_stop = false;
}

void MotorController::stop()
{
    m_stop = true;
    m_target_right = m_target_left = 0;
    m_current_duty_right = m_current_duty_left = 0;
    if (!m_controller.write(write_commands::m1_m2_drive_duty{0}))
    {
        m_logger->error(
                "!!! Cannot stop the rover during normal operation !!! Your robot is "
                "going to die !!!");
    }
}

void MotorController::cb(const ros::TimerEvent&)
{
    publish_filtered_to_ros();

    if (m_stop) return;

    try
    {
        // Left motor
        auto left_qpps_speed =
                m_controller.read<read_commands::m2_raw_speed>(CTRL_MAX_TRIES);
        float error_left = (float)(m_target_left - left_qpps_speed.speed) /
                           m_params.pid_left.max_qpps;
        m_current_duty_left += error_left * m_params.pid_left.p;

        // Right motor
        auto right_qpps_speed =
                m_controller.read<read_commands::m1_raw_speed>(CTRL_MAX_TRIES);
        float error_right = (float)(m_target_right - right_qpps_speed.speed) /
                            m_params.pid_right.max_qpps;
        m_current_duty_right += error_right * m_params.pid_right.p;

        if (!m_controller.write(write_commands::m1_m2_drive_duty{m_current_duty_right,
                                                                 m_current_duty_left},
                                CTRL_MAX_TRIES))
        {
            m_logger->error(
                    "Failed to send the control command. Trying to stop the robot.");
            if (!m_controller.write(write_commands::m1_m2_drive_duty{0, 0},
                                    CTRL_MAX_TRIES))
            {
                m_logger->error(
                        "!!! Cannot stop the rover during error handling !!! Your robot "
                        "is going to die!!!");
            }
        }
    }
    catch (std::runtime_error& e)
    {
        m_logger->error(
                "Could not read the current raw speeds. Trying to stop the robot!");
        if (!m_controller.write(write_commands::m1_m2_drive_duty{0, 0}, CTRL_MAX_TRIES))
        {
            m_logger->error(
                    "!!! Cannot stop the rover during error handling !!! Your robot is "
                    "going to die!!!");
        }
    }
}

void MotorController::publish_filtered_to_ros()
{
    // Publish filtered data to ROS
    try
    {
        auto left_qpps_speed =
                m_controller.read<read_commands::m2_encoder_speed>(CTRL_MAX_TRIES);
        auto right_qpps_speed =
                m_controller.read<read_commands::m1_encoder_speed>(CTRL_MAX_TRIES);
        std_msgs::Int64 ros_left;
        ros_left.data = left_qpps_speed.speed;

        std_msgs::Int64 ros_right;
        ros_right.data = right_qpps_speed.speed;

        m_left_qpps_pub.publish(ros_left);
        m_right_qpps_pub.publish(ros_right);
    }
    catch (std::runtime_error& e)
    {
        m_logger->error(
                "Could not read the current encoder speeds. Trying to stop the robot!");
        if (!m_controller.write(write_commands::m1_m2_drive_duty{0, 0}, CTRL_MAX_TRIES))
        {
            m_logger->error(
                    "!!! Cannot stop the rover during error handling !!! Your robot is "
                    "going to die!!!");
        }
    }
}

}  // namespace a4wd2::toolkit
