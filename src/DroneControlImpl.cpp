#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <utility>

namespace drone_mapper {
namespace {

types::MovementResult executeMovement(IDroneMovement& movement, const types::MovementCommand& cmd) {
    switch (cmd.type) {
    case types::MovementCommandType::Rotate:
        return movement.rotate(cmd.rotation, cmd.angle);
    case types::MovementCommandType::Advance:
        return movement.advance(cmd.distance);
    case types::MovementCommandType::Elevate:
        return movement.elevate(cmd.distance);
    case types::MovementCommandType::Hover:
        return types::MovementResult{true, {}};
    }
    return types::MovementResult{true, {}};
}

} // namespace

DroneControlImpl::DroneControlImpl(types::LidarConfigData lidar_config,
                                   ILidar& lidar,
                                   IGPS& rounded_gps,
                                   IGPS& exact_gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : lidar_config_(std::move(lidar_config)),
      lidar_(lidar),
      rounded_gps_(rounded_gps),
      exact_gps_(exact_gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    // 1) Believed state from the rounded GPS view.
    const types::DroneState state{rounded_gps_.position(), rounded_gps_.heading(), step_index_};
    ++step_index_;

    // 2) Ask the algorithm; thread the previous scan (nullptr on the first step).
    const types::LidarScanResult* scan_ptr = latest_scan_.has_value() ? &(*latest_scan_) : nullptr;
    const types::MappingStepCommand cmd = mapping_algorithm_.nextStep(state, scan_ptr);

    // 3) Movement first; a failed move ends the step in Error (scores -1 upstream).
    if (cmd.movement.has_value()) {
        const types::MovementResult moved = executeMovement(movement_, *cmd.movement);
        if (!moved.success) {
            return types::DroneStepResult{types::DroneStepStatus::Error, moved.message};
        }
    }

    // 4) Scan second, applied from the EXACT (post-move) pose. The converter writes the
    //    voxels; we keep the scan to thread into the next nextStep.
    if (cmd.scan_orientation.has_value()) {
        types::LidarScanResult scan = lidar_.scan(*cmd.scan_orientation);
        ScanResultToVoxels::applyToMap(output_map_, exact_gps_.position(), exact_gps_.heading(),
                                       scan, lidar_config_);
        latest_scan_ = std::move(scan);
    }

    // 5) Map the algorithm status to the step status.
    switch (cmd.status) {
    case types::AlgorithmStatus::Working:
        return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
    case types::AlgorithmStatus::Finished:
    case types::AlgorithmStatus::FinishedWithUnmappableVoxels:
        return types::DroneStepResult{types::DroneStepStatus::Completed, {}};
    }
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{rounded_gps_.position(), rounded_gps_.heading(), step_index_};
}

} // namespace drone_mapper
