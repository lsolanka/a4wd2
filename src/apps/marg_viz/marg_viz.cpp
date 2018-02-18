#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/viz/vizcore.hpp>

#include <sensor_reader/mpu9250.hpp>
#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/string_line_reader.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

using namespace cv;
using namespace std;

static constexpr float FONT_SIZE = 12.f;

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

            std::stringstream ss;
            ss << "IMU accel: " << data.a << '\n';
            ss << "IMU gyro : " << data.g << '\n';
            ss << "IMU mag  : " << data.m;
            sensor_text.setText(ss.str());

            window.showWidget("sensor_x",
                              viz::WArrow(cv::Point3d(0, 0, 0),
                                          -cv::Point3d(data.a.x, data.a.y, data.a.z)));
        }

        window.spinOnce(10, true);
    }

    serial_reader.shutdown();
    sensor_thread.join();

    return 0;
}
