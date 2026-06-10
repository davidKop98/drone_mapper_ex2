#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace drone_mapper {
namespace {

std::shared_ptr<NpyArray> loadNpyArray(const std::filesystem::path& path) {
    auto map = std::make_shared<NpyArray>();
    const std::string path_string = path.string();
    const char* error = map->LoadNPY(path_string.c_str());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file: ") + error);
    }
    return map;
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    const types::MapConfig hidden_map_config{
        types::MappingBounds{},
        simulation.map_offset,
        simulation.map_resolution,
    };
    auto hidden_map = std::make_unique<Map3DImpl>(
        loadNpyArray(simulation.map_filename),
        hidden_map_config);

    const types::MapConfig output_map_config{
        hidden_map_config.boundaries,
        hidden_map_config.offset,
        mission.gps_resolution,
    };
    auto output_map = std::make_unique<Map3DImpl>(
        std::make_shared<NpyArray>(),
        output_map_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]});
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    const std::filesystem::path output_map_file = output_path / "output_map_stub.npy";
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace drone_mapper
