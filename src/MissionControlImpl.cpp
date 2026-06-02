#include <cpp_course/MissionControlImpl.h>

#include <cpp_course/MapsComparison.h>

#include <utility>

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
    output_map_.save(output_map_file_);
    const double score = MapsComparison::compare(
        hidden_map_,
        output_map_,
        mission_.gps_resolution);

    return types::MissionRunResult{
        types::MissionRunStatus::Error,
        0,
        score,
        output_map_file_,
        types::ErrorRef{"MISSION_CONTROL_NOT_IMPLEMENTED", "MissionControlImpl::runMission is a stub."},
    };
}

} // namespace drone_mapper
