#pragma once

#include <cpp_course/IDroneControl.h>
#include <cpp_course/IDroneMovement.h>
#include <cpp_course/IGPS.h>
#include <cpp_course/ILidar.h>
#include <cpp_course/IMappingAlgorithm.h>
#include <cpp_course/IMutableMap3D.h>

namespace drone_mapper {

class DroneControlImpl final : public IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
};

} // namespace drone_mapper
