#pragma once

#include <algorithm>
#include <iterator>
#include <memory>
#include <vector>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace a4wd2
{

class laser_scan_provider
{
public:
    laser_scan_provider(ros::NodeHandle nh, const std::string& topic_name) : m_nh(nh)
    {
        m_logger = spdlog::get("laser_scan_provider");
        if (!m_logger)
        {
            m_logger = spdlog::stdout_color_mt("laser_scan_provider");
        }
        m_laser_subscriber = m_nh.subscribe(topic_name, QUEUE_SIZE,
                                            &laser_scan_provider::scan_callback, this);
        m_logger->info("subscribing to topic '{}'", topic_name);
    }

    bool get_range_scan(mrpt::obs::CObservation2DRangeScan& scan) const;

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_laser_subscriber;
    sensor_msgs::LaserScan m_last_scan;
    std::shared_ptr<spdlog::logger> m_logger;

    static constexpr uint32_t QUEUE_SIZE = 10;

    void scan_callback(const sensor_msgs::LaserScanConstPtr& scan)
    {
        m_last_scan = *scan;
    }
};

}  // namespace a4wd2
