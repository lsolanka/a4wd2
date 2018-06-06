#include <opencv2/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <ros/ros.h>

#include <a4wd2/control_command.h>

namespace viz = cv::viz;

static constexpr int QUEUE_SIZE = 100;
static constexpr int SPEED_QPPS = 10000;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_controller");
    ros::NodeHandle nh;
    cv::namedWindow("Test window");
    viz::Viz3d window("Robot position");
    window.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    ros::Publisher command_publisher =
            nh.advertise<a4wd2::control_command>("/control", QUEUE_SIZE, false);

    a4wd2::control_command cmd;
    cmd.speed_qpps = SPEED_QPPS;

    while (!window.wasStopped())
    {
        int key = -1;
        key = cv::waitKey(500);
        std::cout << "key: '" << key << std::endl;
        switch (key)
        {
            case 'k':
                cmd.speed_qpps = SPEED_QPPS;
                cmd.cmd = a4wd2::control_command::FORWARD;
                command_publisher.publish(cmd);
                break;

            case 'j':
                cmd.speed_qpps = SPEED_QPPS;
                cmd.cmd = a4wd2::control_command::BACKWARD;
                command_publisher.publish(cmd);
                break;

            case 'h':
                cmd.speed_qpps = SPEED_QPPS;
                cmd.cmd = a4wd2::control_command::LEFT;
                command_publisher.publish(cmd);
                break;

            case 'l':
                cmd.speed_qpps = SPEED_QPPS;
                cmd.cmd = a4wd2::control_command::RIGHT;
                command_publisher.publish(cmd);
                break;

            case ' ':
                cmd.speed_qpps = 0;
                cmd.cmd = a4wd2::control_command::FORWARD;
                command_publisher.publish(cmd);

            default:
                break;
        }
        window.spinOnce(10, true);
    }

    cmd.speed_qpps = 0;
    cmd.cmd = a4wd2::control_command::FORWARD;
    command_publisher.publish(cmd);

    return 0;
}
