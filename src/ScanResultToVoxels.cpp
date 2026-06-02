#include <drone_mapper/ScanResultToVoxels.h>

namespace drone_mapper {

std::vector<types::MappedVoxel> ScanResultToVoxels::convert(const Position3D& scan_origin,
                                                            const Orientation& drone_heading,
                                                            const types::LidarScanResult& scan) {
    (void)scan_origin;
    (void)drone_heading;
    (void)scan;
    return {};
}

} // namespace drone_mapper
