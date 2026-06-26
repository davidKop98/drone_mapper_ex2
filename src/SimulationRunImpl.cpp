#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> exact_gps,
                                     std::unique_ptr<IGPS> rounded_gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file,
                                     types::ResolutionRequestStatus resolution_request_status)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      exact_gps_(std::move(exact_gps)),
      rounded_gps_(std::move(rounded_gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)),
      resolution_request_status_(resolution_request_status) {
    if (!hidden_map_ ||
        !output_map_ ||
        !exact_gps_ ||
        !rounded_gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.resolution_request_status = resolution_request_status_;
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_->getMapConfig();

    // A single bad run must never kill the batch: catch everything here.
    try {
        const types::MissionRunResult mission_result = mission_control_->runMission();
        result.mission_results.push_back(mission_result);

        if (mission_result.status == types::MissionRunStatus::Error) {
            result.mission_score = -1.0;
        } else {
            // Completed or MaxSteps -> score the output map against the hidden ground truth.
            const std::vector<double> scores =
                MapsComparison::compare(*hidden_map_, std::vector<IMap3D*>{output_map_.get()});
            result.mission_score = scores.empty() ? -1.0 : scores.front();
        }
    } catch (const std::exception& e) {
        result.mission_score = -1.0;
        result.mission_results.push_back(types::MissionRunResult{
            types::MissionRunStatus::Error, 0,
            {types::ErrorRef{"SIMULATION_RUN_EXCEPTION", e.what()}}});
    } catch (...) {
        result.mission_score = -1.0;
        result.mission_results.push_back(types::MissionRunResult{
            types::MissionRunStatus::Error, 0,
            {types::ErrorRef{"SIMULATION_RUN_EXCEPTION", "unknown error"}}});
    }
    return result;
}

} // namespace drone_mapper
