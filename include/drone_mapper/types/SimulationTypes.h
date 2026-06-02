#pragma once

#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper::types {

struct SimulationConfigData {
    SimulationConfigData() = delete;

    SimulationConfigData(std::filesystem::path map_filename_value,
                         PhysicalLength map_resolution_value,
                         Position3D initial_drone_position_value,
                         HorizontalAngle initial_angle_value)
        : map_filename(std::move(map_filename_value)),
          map_resolution(map_resolution_value),
          initial_drone_position(initial_drone_position_value),
          initial_angle(initial_angle_value) {}

    std::filesystem::path map_filename;
    PhysicalLength map_resolution;
    Position3D initial_drone_position;
    HorizontalAngle initial_angle;
};

struct SimulationCompositionData {
    SimulationCompositionData() = delete;

    SimulationCompositionData(std::filesystem::path composition_file_value,
                              std::vector<SimulationConfigData> simulations_value,
                              std::vector<MissionConfigData> missions_value,
                              std::vector<DroneConfigData> drones_value,
                              std::vector<LidarConfigData> lidars_value)
        : composition_file(std::move(composition_file_value)),
          simulations(std::move(simulations_value)),
          missions(std::move(missions_value)),
          drones(std::move(drones_value)),
          lidars(std::move(lidars_value)) {}

    std::filesystem::path composition_file;
    std::vector<SimulationConfigData> simulations;
    std::vector<MissionConfigData> missions;
    std::vector<DroneConfigData> drones;
    std::vector<LidarConfigData> lidars;
};

enum class ResolutionRequestStatus {
    Accepted,
    Ignored,
    IgnoredTooSmall,
};

struct MissionScoreGroup {
    MissionScoreGroup() = delete;

    MissionScoreGroup(MissionConfigData mission_value,
                      PhysicalLength resolution_value,
                      ResolutionRequestStatus resolution_request_status_value,
                      std::vector<MissionRunResult> runs_value)
        : mission(std::move(mission_value)),
          resolution(resolution_value),
          resolution_request_status(resolution_request_status_value),
          runs(std::move(runs_value)) {}

    MissionConfigData mission;
    PhysicalLength resolution;
    ResolutionRequestStatus resolution_request_status;
    std::vector<MissionRunResult> runs;
};

struct SimulationReport {
    std::filesystem::path composition_file{};
    std::string generated_at_utc{};
    std::vector<MissionScoreGroup> simulations{};
};

} // namespace drone_mapper::types
