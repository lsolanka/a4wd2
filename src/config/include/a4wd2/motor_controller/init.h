#pragma once

#include <roboclaw/io/io.hpp>

namespace a4wd2::motor_controller
{

void log_status(::roboclaw::io::serial_controller& controller,
                         std::shared_ptr<spdlog::logger> logger);
void init(::roboclaw::io::serial_controller& controller);

} // namespace a4wd2::motor_controller
