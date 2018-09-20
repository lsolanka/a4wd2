#include <chrono>
#include <iostream>

#include <boost/function.hpp>
#include <cxxopts.hpp>

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <a4wd2/config.h>

static const std::string log_prefix = "a4wd2_visualisation - ";
static constexpr int QUEUE_SIZE = 100;

class pose_tracker
{
public:
    using seconds = std::chrono::duration<double, std::chrono::seconds::period>;

    pose_tracker()
        : n_samples(0),
          a_x_sum(0),
          a_y_sum(0),
          v_phi_sum(0),
          m_timestamp(0),
          m_pose(0, 0, 0),
          v_x(0),
          v_y(0),
          v_phi(0)
    {
    }

    void accelerate_and_rotate(seconds timestamp, double a_x, double a_y, double vel_phi)
    {
        // Special case to avoid a glitch when no timestamp is available
        if (m_timestamp.count() == 0)
        {
            m_timestamp = timestamp;
            return;
        }

        n_samples++;
        a_x_sum += a_x;
        a_y_sum += a_y;
        v_phi_sum += v_phi;
        std::cout << "running avg a_x, a_y, v_phi: " << a_x_sum / n_samples << ", "
                  << a_y_sum / n_samples << ", " << v_phi_sum / n_samples << std::endl;

        double dt_sec = (timestamp - m_timestamp).count();
        std::cout << "dt_sec: " << dt_sec << std::endl;

        m_pose += mrpt::poses::CPose2D(v_x * dt_sec, v_y * dt_sec, v_phi * dt_sec);
        v_x += a_x * dt_sec;
        v_y += a_y * dt_sec;
        v_phi = vel_phi;

        m_timestamp = timestamp;
    }

    const mrpt::poses::CPose2D& get_pose() const { return m_pose; }

private:
    seconds m_timestamp;
    mrpt::poses::CPose2D m_pose;
    double v_x, v_y, v_phi;
    double a_x_sum, a_y_sum, v_phi_sum;
    int n_samples;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "a4wd2_visualisation");
    ros::NodeHandle nh;
    ros::Subscriber imu_subscriber;

    // clang-format off
    cxxopts::Options options("a4wd2_visualisation", "ROS visualisation module for A4WD2");
    options.add_options()
        ("h,help", "Print this help")
        ("v,verbosity",
            "Verbosity level [trace, debug, info, warning, error, fatal]",
            cxxopts::value<std::string>()->default_value("disabled"));
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

    pose_tracker tracker;
    boost::function<void(const sensor_msgs::Imu&)> callback =
            [&tracker](const sensor_msgs::Imu& imu_msg) {
                pose_tracker::seconds ts{imu_msg.header.stamp.toSec()};
                tracker.accelerate_and_rotate(ts, imu_msg.linear_acceleration.x,
                                              imu_msg.linear_acceleration.y,
                                              imu_msg.angular_velocity.z);
                std::cout << tracker.get_pose().asString() << std::endl;
            };

    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("imu", QUEUE_SIZE, callback);
    ros::spin();

    return EXIT_SUCCESS;
}

