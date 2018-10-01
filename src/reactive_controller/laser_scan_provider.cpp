#include "laser_scan_provider.h"

namespace a4wd2
{

bool laser_scan_provider::get_range_scan(mrpt::obs::CObservation2DRangeScan& scan) const
{
    if (m_last_scan.ranges.size() == 0)
    {
        return false;
    }

    std::vector<char> scan_validity;
    scan_validity.reserve(m_last_scan.ranges.size());
    std::transform(m_last_scan.ranges.begin(), m_last_scan.ranges.end(),
                   std::back_inserter(scan_validity), [this](float range) {
                       return range >= m_last_scan.range_min &&
                              range <= m_last_scan.range_max;
                   });

    scan.rightToLeft = true;
    scan.aperture = m_last_scan.angle_max - m_last_scan.angle_min;
    scan.loadFromVectors(m_last_scan.ranges.size(), m_last_scan.ranges.data(),
                         scan_validity.data());

    return true;
}

} // namespace a4wd2
