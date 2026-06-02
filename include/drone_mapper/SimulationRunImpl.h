#pragma once

#include <cpp_course/IDroneControl.h>
#include <cpp_course/IDroneMovement.h>
#include <cpp_course/IGPS.h>
#include <cpp_course/ILidar.h>
#include <cpp_course/IMap3D.h>
#include <cpp_course/IMappingAlgorithm.h>
#include <cpp_course/IMissionControl.h>
#include <cpp_course/IMutableMap3D.h>
#include <cpp_course/ISimulationRun.h>

#include <memory>

namespace drone_mapper {

class SimulationRunImpl final : public ISimulationRun {
public:
    SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                      std::unique_ptr<IMutableMap3D> output_map,
                      std::unique_ptr<IGPS> gps,
                      std::unique_ptr<IDroneMovement> movement,
                      std::unique_ptr<ILidar> lidar,
                      std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<IDroneControl> drone_control,
                      std::unique_ptr<IMissionControl> mission_control);

    [[nodiscard]] types::MissionRunResult run() override;

private:
    std::unique_ptr<const IMap3D> hidden_map_;
    std::unique_ptr<IMutableMap3D> output_map_;
    std::unique_ptr<IGPS> gps_;
    std::unique_ptr<IDroneMovement> movement_;
    std::unique_ptr<ILidar> lidar_;
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<IDroneControl> drone_control_;
    std::unique_ptr<IMissionControl> mission_control_;
};

} // namespace drone_mapper
