#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>

#include <cstddef>
#include <optional>

namespace drone_mapper {

// Drives one mission step: build DroneState from the rounded GPS view, ask the algorithm
// for a command, execute the movement FIRST, then take the requested scan and apply it to
// the output map from the EXACT pose (so the converter places voxels where the lidar
// actually traced). Holds both GPS views: rounded for the believed state, exact for the
// scan origin.
class DroneControlImpl final : public IDroneControl {
public:
    DroneControlImpl(types::LidarConfigData lidar_config,
                     ILidar& lidar,
                     IGPS& rounded_gps,
                     IGPS& exact_gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    types::LidarConfigData lidar_config_;
    ILidar& lidar_;
    IGPS& rounded_gps_;
    IGPS& exact_gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;

    std::size_t step_index_ = 0;
    // The previous step's scan, threaded into the next nextStep (nullptr on the first step).
    std::optional<types::LidarScanResult> latest_scan_;
};

} // namespace drone_mapper
