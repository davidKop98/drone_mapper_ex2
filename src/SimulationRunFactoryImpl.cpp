#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace drone_mapper {

std::unique_ptr<Map3DImpl> SimulationRunFactoryImpl::loadHiddenMap(const types::SimulationConfigData& sim) {
    auto arr = std::make_shared<NpyArray>();
    const char* error = arr->LoadNPY(sim.map_filename.string());
    if (error != nullptr) {
        throw std::runtime_error("failed to load hidden map '" + sim.map_filename.string() + "': " + error);
    }
    const NpyArray::shape_t& shape = arr->Shape();
    if (shape.size() != 3) {
        throw std::runtime_error("hidden map '" + sim.map_filename.string() + "' is not a 3D array");
    }

    const double res = sim.map_resolution.force_numerical_value_in(cm);
    const double ox = sim.map_offset.x.force_numerical_value_in(cm);
    const double oy = sim.map_offset.y.force_numerical_value_in(cm);
    const double oz = sim.map_offset.z.force_numerical_value_in(cm);

    types::MapConfig cfg;
    cfg.resolution = sim.map_resolution;
    cfg.offset = sim.map_offset;
    // Boundaries = the map's full extent: [offset, offset + shape * resolution).
    cfg.boundaries = types::MappingBounds{
        ox * x_extent[cm], (ox + static_cast<double>(shape[0]) * res) * x_extent[cm],
        oy * y_extent[cm], (oy + static_cast<double>(shape[1]) * res) * y_extent[cm],
        oz * z_extent[cm], (oz + static_cast<double>(shape[2]) * res) * z_extent[cm],
    };
    return std::make_unique<Map3DImpl>(arr, cfg);
}

std::unique_ptr<Map3DImpl> SimulationRunFactoryImpl::makeOutputMap(const types::MappingBounds& bounds,
                                                                   PhysicalLength resolution) {
    const double res = resolution.force_numerical_value_in(cm);
    const double min_x = bounds.min_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm);
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double max_z = bounds.max_height.force_numerical_value_in(cm);
    //lambda func which computes how many grid cells fit between lo and hi                                                             
    const auto cellCount = [res](double lo, double hi) -> std::size_t {
        if (res <= 0.0) return 0;
        return static_cast<std::size_t>(std::max(0L, std::lround((hi - lo) / res)));
    };
    const std::size_t nx = cellCount(min_x, max_x);
    const std::size_t ny = cellCount(min_y, max_y);
    const std::size_t nz = cellCount(min_z, max_z);

    auto arr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz}, sizeof(std::uint8_t), 'u', false);
    arr->Allocate();
    std::fill(arr->Data<std::uint8_t>(), arr->Data<std::uint8_t>() + arr->NumValue(),
              static_cast<std::uint8_t>(255)); // all Unmapped

    types::MapConfig cfg;
    cfg.boundaries = bounds;
    cfg.offset = Position3D{bounds.min_x, bounds.min_y, bounds.min_height};
    cfg.resolution = resolution;
    return std::make_unique<Map3DImpl>(arr, cfg);
}

SimulationRunFactoryImpl::ResolutionDecision SimulationRunFactoryImpl::resolveOutputResolution(const types::MissionConfigData& mission,
                                                  PhysicalLength default_resolution) {
    // The requested ("expected") output resolution is gps_resolution / factor. We always
    // operate at default_resolution (the input-map resolution); the status only reports how
    // the request compares to what we actually use:
    //   expected == used        -> Accepted        (our default happens to honor the request)
    //   expected <  used (finer) -> IgnoredTooSmall (we won't map finer than supported)
    //   otherwise                -> Ignored         (coarser, or an invalid request)
    const double used = default_resolution.force_numerical_value_in(cm);
    const double gps = mission.gps_resolution.force_numerical_value_in(cm);
    const double factor = mission.output_mapping_resolution_factor;

    types::ResolutionRequestStatus status;
    if (factor <= 0.0 || gps <= 0.0) {
        status = types::ResolutionRequestStatus::Ignored; // undefined request -> use default
    } else {
        const double expected = gps / factor;
        if (std::abs(expected - used) < 1e-9) {
            status = types::ResolutionRequestStatus::Accepted;
        } else if (expected < used) {
            status = types::ResolutionRequestStatus::IgnoredTooSmall;
        } else {
            status = types::ResolutionRequestStatus::Ignored;
        }
    }
    return ResolutionDecision{default_resolution, status}; // always the default (input-map) resolution
}

std::unique_ptr<ISimulationRun> SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Real maps: hidden ground truth from the .npy, output box from the mission bounds.
    auto hidden_map = loadHiddenMap(simulation);
    const ResolutionDecision resolution = resolveOutputResolution(mission, simulation.map_resolution);
    auto output_map = makeOutputMap(mission.mission_bounds, resolution.resolution);

    // One shared exact truth with two views: exact feeds lidar/movement, rounded is what
    // the drone/algorithm observe.
    auto exact_gps = std::make_unique<MockGPS>(simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]}, 0.0 * cm);
    auto rounded_gps = std::make_unique<MockGPS>(exact_gps->truth(), mission.gps_resolution);

    auto movement = std::make_unique<MockMovement>(*exact_gps, *hidden_map, drone.radius);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *exact_gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        lidar, *lidar_impl, *rounded_gps, *exact_gps, *movement, *output_map, *mapping_algorithm);

    const std::filesystem::path output_map_file = output_path / "output_map.npy";
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(exact_gps),
        std::move(rounded_gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file,
        resolution.status);
}

} // namespace drone_mapper
