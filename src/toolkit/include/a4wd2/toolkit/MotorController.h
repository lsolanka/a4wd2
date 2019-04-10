#pragma once

#include <cstdint>
#include <memory>

#include <spdlog/spdlog.h>

#include <roboclaw/io/io.hpp>
#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>

#include <ros/ros.h>
#include <std_msgs/Int64.h>

namespace a4wd2::toolkit
{

class MotorController
{
public:
    struct PID
    {
        float p = 1.;
        float i = 0;
        float d = 0;
        int32_t max_qpps = 42000;
    };

    struct Parameters
    {
        PID pid_left;
        PID pid_right;
    };

    MotorController(ros::NodeHandle& nh, std::chrono::milliseconds interval,
                    roboclaw::io::serial_controller& controller,
                    const Parameters& params);

    void set_target_qpps_left(int32_t qpps);
    void set_target_qpps_right(int32_t qpps);

    void stop();

    void cb(const ros::TimerEvent&);

    void publish_filtered_to_ros();

private:
    static constexpr int CTRL_MAX_TRIES = 5;
    std::shared_ptr<spdlog::logger> m_logger;

    ros::NodeHandle m_nh;
    ros::Timer m_timer;
    ros::Publisher m_right_qpps_pub;
    ros::Publisher m_left_qpps_pub;

    roboclaw::io::serial_controller& m_controller;
    Parameters m_params;
    bool m_stop = true;
    int32_t m_target_left = 0;
    int32_t m_target_right = 0;
    float m_current_duty_left = 0;
    float m_current_duty_right = 0;
};

}  // namespace a4wd2::toolkit
