#pragma once

#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

// **Do not change this interface.**
class IMappingAlgorithm {
public:
    virtual ~IMappingAlgorithm() = default;

    [[nodiscard]] virtual types::MovementCommand nextMove(const types::DroneState& state,
                                                          const types::LidarScanResult& latest_scan) = 0;
    virtual void applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) = 0;
};

} // namespace drone_mapper
