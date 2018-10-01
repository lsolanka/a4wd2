#include <spdlog/spdlog.h>
#include <fmt/ostream.h>

#include <roboclaw/io/io.hpp>
#include <roboclaw/io/read_commands.hpp>
#include <roboclaw/io/write_commands.hpp>

#include <a4wd2/motor_controller/init.h>

namespace read_commands = roboclaw::io::read_commands;
namespace write_commands = roboclaw::io::write_commands;

namespace a4wd2::motor_controller
{

void log_roboclaw_status(::roboclaw::io::serial_controller& controller,
                         std::shared_ptr<spdlog::logger> logger)
{
    logger->info("Firmware version: {}",
                 controller.read<read_commands::firmware_version>());
    logger->info("Main battery voltage: {}",
                 controller.read<read_commands::main_battery_voltage>());
    logger->info("Logic battery voltage: {}",
                 controller.read<read_commands::logic_battery_voltage>());

    logger->info("Encoder mode: {}",
                 get_string(controller.read<read_commands::encoder_mode>()));

    logger->info("M1 velocity PID: {}",
                 get_string(controller.read<read_commands::m1_velocity_pid>()));
    logger->info("M2 velocity PID: {}",
                 get_string(controller.read<read_commands::m2_velocity_pid>()));
}

void init(::roboclaw::io::serial_controller& controller)
{
    auto logger = spdlog::get("a4wd2_main");
    if (!logger)
    {
        throw std::runtime_error(
                "Cannot get the 'a4wd2_main' logger. Create it before calling this "
                "function");
    }

    logger->info("Initialising roboclaw motor controller");

    controller.write(write_commands::m1_encoder_mode{true, false});
    controller.write(write_commands::m2_encoder_mode{true, false});

    controller.write(write_commands::m1_velocity_pid{1.f, 0.5f, 0.25f, 42000});
    controller.write(write_commands::m2_velocity_pid{1.f, 0.5f, 0.25f, 42000});

    log_roboclaw_status(controller, logger);
}

}  // namespace a4wd2::motor_controller
