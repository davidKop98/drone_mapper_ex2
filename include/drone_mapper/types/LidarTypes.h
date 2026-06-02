#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <vector>

namespace drone_mapper::types {

struct LidarConfigData {
    LidarConfigData() = delete;

    LidarConfigData(PhysicalLength z_min_value,
                    PhysicalLength z_max_value,
                    PhysicalLength d_value,
                    std::size_t fov_circles_value)
        : z_min(z_min_value),
          z_max(z_max_value),
          d(d_value),
          fov_circles(fov_circles_value) {}

    PhysicalLength z_min;
    PhysicalLength z_max;
    PhysicalLength d;
    std::size_t fov_circles;
};

struct LidarHit {
    bool hit = false;
    // Misses use max double centimeters; prefer checking hit.
    PhysicalLength distance{};
    Orientation angle{};
};

using LidarScanResult = std::vector<LidarHit>;

} // namespace drone_mapper::types
