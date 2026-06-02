#include <drone_mapper/MappingAlgorithmImpl.h>

#include <utility>

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(types::MissionConfigData mission)
    : mission_(std::move(mission)) {}

types::MovementCommand MappingAlgorithmImpl::nextMove(const types::DroneState& state,
                                               const types::LidarScanResult& latest_scan) {
    (void)state;
    (void)latest_scan;
    (void)mission_;
    return types::MovementCommand{};
}

void MappingAlgorithmImpl::applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) {
    (void)voxels;
}

} // namespace drone_mapper
