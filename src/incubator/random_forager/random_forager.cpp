#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include <boost/program_options.hpp>

#include <nlohmann/json.hpp>

#include <roboclaw/io/io.hpp>
#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>
#include <roboclaw/logging.hpp>

#include <sensor_reader/mpu9250.hpp>
#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/srf08.hpp>
#include <sensor_reader/string_line_reader.hpp>

using a4wd2::sensor_reader::sensor_reader;
using a4wd2::sensor_reader::serial_string_line_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

namespace po = boost::program_options;
namespace read_commands = roboclaw::io::read_commands;
namespace write_commands = roboclaw::io::write_commands;
using roboclaw::io::serial_controller;

using boost::asio::deadline_timer;
using boost::asio::io_service;
namespace pt = boost::posix_time;

static constexpr uint8_t SRF08_RIGHT_ADDR = 0x70;
static constexpr uint8_t SRF08_LEFT_ADDR = 0x71;
static constexpr uint8_t ROBOCLAW_ADDR = 0x80;

enum class ctrl_state
{
    STOPPED,
    FORWARD,
    TURNING
};

enum class turning_dir
{
    LEFT,
    RIGHT
};

struct parameters
{
    struct serial_config
    {
        std::string dev;
        int baud_rate;
    };

    serial_config sensors;
    serial_config motor_controller;
    int foraging_speed_qpps;
    int foraging_turn_speed_qpps;
    float range_obstacle_threshold_cm;
};

struct sensor_readings
{
    sensors::srf08::data_t l;
    sensors::srf08::data_t r;

    sensor_readings() : l{0, 0}, r{0, 0} {}
};

volatile std::atomic<bool> g_interruption_requested(false);

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        g_interruption_requested = true;
    }
}

bool setup_verbosity(const po::variables_map& vm);

void init_roboclaw(roboclaw::io::serial_controller& controller);

void from_json(const nlohmann::json& j, parameters& p)
{
    p.sensors.dev = j.at("sensors").at("device").get<std::string>();
    p.sensors.baud_rate = j.at("sensors").at("baud_rate").get<int>();

    p.motor_controller.dev = j.at("motor_controller").at("device").get<std::string>();
    p.motor_controller.baud_rate = j.at("motor_controller").at("baud_rate").get<int>();

    p.foraging_speed_qpps = j.at("foraging_speed_qpps").get<int>();
    p.foraging_turn_speed_qpps = j.at("foraging_turn_speed_qpps").get<int>();

    p.range_obstacle_threshold_cm = j.at("range_obstacle_threshold_cm").get<float>();
}

parameters parse_config_file(const std::string& file_path)
{
    std::fstream config_file(file_path, std::ios::in);
    nlohmann::json j;
    config_file >> j;

    return static_cast<parameters>(j);
}

turning_dir get_turning_direction(const sensor_readings& ranges)
{
    // Turn left if readings are equal
    if (ranges.l.range <= ranges.r.range)
    {
        return turning_dir::LEFT;
    }
    else
    {
        return turning_dir::RIGHT;
    }
}

bool obstacle_detected(const sensor_readings& ranges, float threshold_cm)
{
    return ranges.r.range <= threshold_cm || ranges.l.range < threshold_cm;
}

int main(int argc, char** argv)
{
    std::string verbosity;
    std::string config_path;

    // clang-format off
    po::options_description desc("Allowed options:");
    desc.add_options()
        ("help,h", "Print help message")
        ("verbosity,v", po::value<std::string>(&verbosity)->default_value("disabled"),
                        "Verbosity level [trace, debug, info, warning, error, fatal]")
        ("config,c", po::value<std::string>(&config_path), "Configuration file path.");
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc;
        return EXIT_SUCCESS;
    }
    if (!setup_verbosity(vm))
    {
        std::cout << desc;
        return EXIT_FAILURE;
    }
    if (!vm.count("config"))
    {
        std::cerr << desc << std::endl;
        return EXIT_FAILURE;
    }

    parameters params = parse_config_file(config_path);

    serial_string_line_reader arduino_serial_reader(params.sensors.dev,
                                                    params.sensors.baud_rate);

    // Run the sensors in separate thread
    // Use srf08_data_mutex to R/W data
    sensor_readings srf08_data;
    std::mutex srf08_data_mutex;

    auto sensor_processor = [&]() {
        sensor_reader arduino_sensor_reader{arduino_serial_reader.get_stream()};
        arduino_sensor_reader.add_sensor<sensors::srf08>(
                SRF08_RIGHT_ADDR, [&](const auto& data) {
                    std::lock_guard<std::mutex> lock(srf08_data_mutex);
                    srf08_data.r = data;
                });
        arduino_sensor_reader.add_sensor<sensors::srf08>(
                SRF08_LEFT_ADDR, [&](const auto& data) {
                    std::lock_guard<std::mutex> lock(srf08_data_mutex);
                    srf08_data.l = data;
                });
        arduino_sensor_reader.read_all();
    };
    std::thread sensor_thread(sensor_processor);

    // Setup and start the foraging / controller
    roboclaw::io::serial_controller controller(params.motor_controller.dev,
                                               ROBOCLAW_ADDR);
    init_roboclaw(controller);

    ctrl_state current_state = ctrl_state::STOPPED;
    auto spin = [&]() {
        std::lock_guard<std::mutex> lock(srf08_data_mutex);

        if (obstacle_detected(srf08_data, params.range_obstacle_threshold_cm))
        {
            current_state = ctrl_state::TURNING;
        }

        switch (current_state)
        {
            case ctrl_state::STOPPED:
            case ctrl_state::FORWARD:
                controller.write(
                        write_commands::m1_m2_drive_qpps{params.foraging_speed_qpps});
                break;

            case ctrl_state::TURNING:
                if (obstacle_detected(srf08_data, params.range_obstacle_threshold_cm))
                {
                    if (get_turning_direction(srf08_data) == turning_dir::LEFT)
                    {
                        controller.write(write_commands::m1_drive_qpps{
                                -params.foraging_turn_speed_qpps});
                        controller.write(write_commands::m2_drive_qpps{
                                params.foraging_turn_speed_qpps});
                    }
                    else
                    {
                        controller.write(write_commands::m1_drive_qpps{
                                params.foraging_turn_speed_qpps});
                        controller.write(write_commands::m2_drive_qpps{
                                -params.foraging_turn_speed_qpps});
                    }
                }
                else
                {
                    current_state = ctrl_state::FORWARD;
                }
                break;

            default:
                throw std::runtime_error("Unknown motor controller state encountered");
        };
    };

    std::signal(SIGINT, signal_handler);

    while (true)
    {
        if (g_interruption_requested)
        {
            break;
        }

        spin();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // clean up
    std::cerr << "SIGINT encountered. Shutting down. " << std::endl;
    controller.write(write_commands::m1_m2_drive_duty{0});
    arduino_serial_reader.shutdown();
    sensor_thread.join();

    return 0;
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

bool setup_verbosity(const po::variables_map& vm)
{
    int min_verbosity_level = boost::log::trivial::fatal;

    if (vm.count("verbosity"))
    {
        auto verbosity = vm["verbosity"].as<std::string>();
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
            std::cout << "invalid verbosity value: " << verbosity << std::endl;
            return false;
        }
    }

    boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                        min_verbosity_level);

    return true;
}

void init_roboclaw(roboclaw::io::serial_controller& controller)
{
    std::cout << "Firmware version: "
              << controller.read<read_commands::firmware_version>();

    std::cout << "Main battery voltage: "
              << controller.read<read_commands::main_battery_voltage>() << std::endl;
    std::cout << "Logic battery voltage: "
              << controller.read<read_commands::logic_battery_voltage>() << std::endl;

    controller.write(write_commands::m1_encoder_mode{true, false});
    controller.write(write_commands::m2_encoder_mode{true, false});

    std::cout << "Encoder mode: "
              << get_string(controller.read<read_commands::encoder_mode>()) << std::endl;

    controller.write(write_commands::m1_velocity_pid{1.f, 0.5f, 0.25f, 42000});
    controller.write(write_commands::m2_velocity_pid{1.f, 0.5f, 0.25f, 42000});

    std::cout << "M1 velocity PID: "
              << get_string(controller.read<read_commands::m1_velocity_pid>())
              << std::endl;
    std::cout << "M2 velocity PID: "
              << get_string(controller.read<read_commands::m2_velocity_pid>())
              << std::endl;
}
