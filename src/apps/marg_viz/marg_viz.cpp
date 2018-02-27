#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include <boost/math/constants/constants.hpp>

#include <eigen3/Eigen/Geometry>

#include <MadgwickAHRS.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz/vizcore.hpp>

#include <sensor_reader/mpu9250.hpp>
#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/string_line_reader.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

using boost::math::constants::pi;

using namespace cv;

static constexpr float FONT_SIZE = 12.f;

static constexpr float DEG2RAD = 2. * pi<float>() / 360.;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "read_serial <port_name>" << std::endl;
        return EXIT_FAILURE;
    }

    const char* port_name = argv[1];
    serial_string_line_reader serial_reader(port_name, 9600);

    // Data vector shared between the window thread and serial thread
    std::mutex data_mutex;
    std::queue<sensors::mpu9250::data_t> imu_data;

    auto sensor_processor = [&]() {
        sensor_reader reader{serial_reader.get_stream()};

        reader.add_sensor<sensors::mpu9250>([&](const auto& data) {
            std::lock_guard<std::mutex> lock(data_mutex);
            imu_data.push(data);
        });
        reader.read_all();
    };
    std::thread sensor_thread(sensor_processor);

    // Visualisation
    viz::Viz3d window("Coordinate Frame");

    window.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    viz::WText sensor_text("IMU raw data: waiting for data...", cv::Point(10, 10),
                           FONT_SIZE);
    window.showWidget("sensor_text", sensor_text);

    Madgwick madgwick;
    madgwick.begin(1000.);
    window.showWidget("sensor_coordinate_frame", viz::WCoordinateSystem());

    while (!window.wasStopped())
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            while (imu_data.size() > 1)
            {
                imu_data.pop();
            }
        }

        if (!imu_data.empty())
        {
            auto data = imu_data.front();
            imu_data.pop();

            for (int i = 0; i < 50; ++i)
            {
                madgwick.update(-data.g.y, -data.g.x, -data.g.z, data.a.y, data.a.x,
                                data.a.z, data.m.x, data.m.y, -data.m.z);
            }

            std::stringstream ss;
            ss << "IMU accel: " << data.a << '\n';
            ss << "IMU gyro : " << data.g << '\n';
            ss << "IMU mag  : " << data.m << '\n';
            ss << "Madgwick : roll: " << std::setw(7) << std::fixed
               << std::setprecision(3) << madgwick.getRoll()
               << ", pitch: " << std::setw(7) << std::fixed << std::setprecision(3)
               << madgwick.getPitch() << ", yaw: " << std::setw(7) << std::fixed
               << std::setprecision(3) << madgwick.getYaw() << '\n';
            sensor_text.setText(ss.str());

            Eigen::Matrix3f rotation_matrix;
            rotation_matrix = Eigen::AngleAxisf(-madgwick.getYaw() * DEG2RAD,
                                                Eigen::Vector3f::UnitZ()) *
                              Eigen::AngleAxisf(madgwick.getPitch() * DEG2RAD,
                                                Eigen::Vector3f::UnitX()) *
                              Eigen::AngleAxisf(madgwick.getRoll() * DEG2RAD,
                                                Eigen::Vector3f::UnitY());
            cv::Mat cv_rotation_matrix;
            cv::eigen2cv(rotation_matrix, cv_rotation_matrix);

            window.setWidgetPose("sensor_coordinate_frame",
                                 cv::Affine3f(cv_rotation_matrix, cv::Vec3f(.5, .5, .5)));
        }

        window.spinOnce(10, true);
    }

    serial_reader.shutdown();
    sensor_thread.join();

    return 0;
}
