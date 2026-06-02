#include <drone_mapper/SimulationManager.h>

#include <stdexcept>
#include <utility>

namespace drone_mapper {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                               const std::filesystem::path& output_path) {
    std::vector<types::MissionScoreGroup> score_groups;

    for (const types::SimulationConfigData& simulation : composition.simulations) {
        for (const types::MissionConfigData& mission : composition.missions) {
            std::vector<types::MissionRunResult> results;

            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    std::unique_ptr<ISimulationRun> run =
                        run_factory_->create(simulation, mission, drone, lidar, output_path);
                    results.push_back(run->run());
                }
            }

            score_groups.emplace_back(
                mission,
                simulation.map_resolution,
                types::ResolutionRequestStatus::Ignored,
                std::move(results));
        }
    }

    return types::SimulationReport{composition.composition_file, "stub", std::move(score_groups)};
}

} // namespace drone_mapper
