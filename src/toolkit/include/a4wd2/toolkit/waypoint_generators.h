#pragma once

#include <spdlog/spdlog.h>

#include <mrpt/nav/reactive/TWaypoint.h>

#include "odometry_provider.h"

namespace a4wd2::toolkit
{

class uniform_random_waypoint_generator
{
public:
    uniform_random_waypoint_generator(std::shared_ptr<spdlog::logger> logger)
        : m_logger(logger)
    {
    }

    mrpt::nav::TWaypointSequence operator()(
            const odometry_provider& odometry_provider) const;

private:
    std::shared_ptr<spdlog::logger> m_logger;
};

}  // namespace a4wd2::toolkit
