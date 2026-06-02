#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    explicit MappingAlgorithmImpl(types::MissionConfigData mission);

    [[nodiscard]] types::MovementCommand nextMove(const types::DroneState& state,
                                                  const types::LidarScanResult& latest_scan) override;
    void applyVoxelUpdates(const std::vector<types::MappedVoxel>& voxels) override;

private:
    types::MissionConfigData mission_;
};

} // namespace drone_mapper
