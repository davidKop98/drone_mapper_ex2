#pragma once

#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

class ScanResultToVoxels {
public:
    [[nodiscard]] static std::vector<types::MappedVoxel> convert(const Position3D& scan_origin,
                                                                 const Orientation& drone_heading,
                                                                 const types::LidarScanResult& scan);
};

} // namespace drone_mapper
