#include <a4wd2/config.h>

namespace a4wd2::config
{

bool setup_verbosity(const cxxopts::ParseResult& options)
{
    auto lg_roboclaw = spdlog::stdout_color_mt("roboclaw");
    auto lg_main = spdlog::stdout_color_mt("a4wd2_main");
    spdlog::set_async_mode(8192);

    if (options.count("verbosity"))
    {
        auto verbosity = options["verbosity"].as<std::string>();
        if (verbosity == "disabled")
        {
            spdlog::set_level(spdlog::level::off);
        }
        else if (verbosity == "debug")
        {
            spdlog::set_level(spdlog::level::debug);
        }
        else if (verbosity == "info")
        {
            spdlog::set_level(spdlog::level::info);
        }
        else if (verbosity == "warning")
        {
            spdlog::set_level(spdlog::level::warn);
        }
        else if (verbosity == "error")
        {
            spdlog::set_level(spdlog::level::err);
        }
        else if (verbosity == "critical")
        {
            spdlog::set_level(spdlog::level::critical);
        }
        else
        {
            std::cout << "invalid verbosity value: " << verbosity << std::endl;
            return false;
        }
    }

    return true;
}

}  // namespace a4wd2::config
