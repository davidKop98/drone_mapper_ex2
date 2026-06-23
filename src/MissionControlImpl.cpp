#include <drone_mapper/MissionControlImpl.h>

#include <utility>
#include <vector>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    const std::size_t max_steps = mission_.max_steps;
    std::size_t steps = 0;
    // If the loop runs to completion without finishing, the run hit the step cap.
    types::MissionRunStatus status = types::MissionRunStatus::MaxSteps;
    std::vector<types::ErrorRef> errors;

    while (steps < max_steps) {
        const types::DroneStepResult result = drone_control_.step();
        ++steps;
        if (result.status == types::DroneStepStatus::Completed) {
            status = types::MissionRunStatus::Completed;
            break;
        }
        if (result.status == types::DroneStepStatus::Error) {
            status = types::MissionRunStatus::Error; // run scores -1 upstream
            errors.push_back(types::ErrorRef{"DRONE_STEP_ERROR", result.message});
            break;
        }
        // Continue -> keep stepping.
    }

    // Persist whatever was mapped, regardless of outcome.
    output_map_.save(output_map_file_);
    return types::MissionRunResult{status, steps, std::move(errors)};
}

} // namespace drone_mapper
