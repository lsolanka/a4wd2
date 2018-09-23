#include <ros/ros.h>
#include <opencv2/highgui.hpp>

#include <a4wd2/control_command.h>

static constexpr int QUEUE_SIZE = 100;
static constexpr int SPEED_QPPS = 2000;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_controller");
    ros::NodeHandle nh;
    cv::namedWindow("Test window");

    ros::Publisher command_publisher =
            nh.advertise<a4wd2::control_command>("/control", QUEUE_SIZE, false);

    a4wd2::control_command cmd;
    cmd.speed_qpps = SPEED_QPPS;

    int key = -1;
    do
    {
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
                cmd.cmd = a4wd2::control_command::STOP;
                command_publisher.publish(cmd);
                break;

            default:
                break;
        }
    } while (key != 'q');

    cmd.speed_qpps = 0;
    cmd.cmd = a4wd2::control_command::FORWARD;
    command_publisher.publish(cmd);

    return 0;
}
