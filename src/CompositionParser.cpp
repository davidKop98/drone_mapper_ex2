#include <drone_mapper/CompositionParser.h>

#include <drone_mapper/Units.h>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

double num(const YAML::Node& node, const char* key, double fallback) {
    return (node && node[key]) ? node[key].as<double>(fallback) : fallback;
}

double atOr(const YAML::Node& seq, std::size_t i, double fallback) {
    return (seq && seq.IsSequence() && seq.size() > i) ? seq[i].as<double>(fallback) : fallback;
}

Position3D readPosition(const YAML::Node& seq) {
    return Position3D{
        atOr(seq, 0, 0.0) * x_extent[cm],
        atOr(seq, 1, 0.0) * y_extent[cm],
        atOr(seq, 2, 0.0) * z_extent[cm],
    };
}

// boundaries_cm = [min_x, min_y, min_z, max_x, max_y, max_z].
types::MappingBounds readBounds(const YAML::Node& seq) {
    return types::MappingBounds{
        atOr(seq, 0, 0.0) * x_extent[cm], atOr(seq, 3, 0.0) * x_extent[cm],
        atOr(seq, 1, 0.0) * y_extent[cm], atOr(seq, 4, 0.0) * y_extent[cm],
        atOr(seq, 2, 0.0) * z_extent[cm], atOr(seq, 5, 0.0) * z_extent[cm],
    };
}

} // namespace

std::filesystem::path resolveCompositionPath(const std::optional<std::string>& arg) {
    if (!arg || arg->empty()) {
        return std::filesystem::path{"simulation.yaml"};
    }
    // A relative path is resolved against the cwd when opened; an absolute path is used as-is.
    return std::filesystem::path{*arg};
}

types::SimulationCompositionData parseCompositionYaml(const std::filesystem::path& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to load composition '" + path.string() + "': " + e.what());
    }

    types::SimulationCompositionData composition;
    composition.composition_file = path;

    const YAML::Node simulations = root["simulations"];
    if (simulations && simulations.IsSequence()) {
        for (const YAML::Node& sim_node : simulations) {
            types::SimulationConfigData sim;
            sim.map_filename = sim_node["map_filename"] ? sim_node["map_filename"].as<std::string>() : std::string{};
            sim.map_resolution = num(sim_node, "map_resolution_cm", 10.0) * cm;
            sim.map_offset = readPosition(sim_node["map_offset_cm"]);
            sim.initial_drone_position = readPosition(sim_node["initial_drone_position_cm"]);
            sim.initial_angle = num(sim_node, "initial_angle_deg", 0.0) * horizontal_angle[deg];

            std::vector<types::MissionConfigData> missions;
            const YAML::Node mission_nodes = sim_node["missions"];
            if (mission_nodes && mission_nodes.IsSequence()) {
                for (const YAML::Node& m_node : mission_nodes) {
                    types::MissionConfigData mission;
                    mission.max_steps = m_node["max_steps"] ? m_node["max_steps"].as<std::size_t>(0) : 0;
                    mission.gps_resolution = num(m_node, "gps_resolution_cm", 10.0) * cm;
                    mission.output_mapping_resolution_factor = num(m_node, "output_mapping_resolution_factor", 1.0);
                    mission.mission_bounds = readBounds(m_node["boundaries_cm"]);
                    missions.push_back(mission);
                }
            }
            composition.simulation_mission_groups.emplace_back(sim, std::move(missions));
        }
    }

    const YAML::Node drones = root["drones"];
    if (drones && drones.IsSequence()) {
        for (const YAML::Node& d_node : drones) {
            types::DroneConfigData drone;
            drone.radius = 0.5 * num(d_node, "dimensions_cm", 0.0) * cm; // diameter -> radius
            drone.max_rotate = num(d_node, "max_rotate_deg", 0.0) * horizontal_angle[deg];
            drone.max_advance = num(d_node, "max_advance_cm", 0.0) * cm;
            drone.max_elevate = num(d_node, "max_elevate_cm", 0.0) * cm;
            composition.drones.push_back(drone);
        }
    }

    const YAML::Node lidars = root["lidars"];
    if (lidars && lidars.IsSequence()) {
        for (const YAML::Node& l_node : lidars) {
            types::LidarConfigData lidar;
            lidar.z_min = num(l_node, "z_min_cm", 0.0) * cm;
            lidar.z_max = num(l_node, "z_max_cm", 0.0) * cm;
            lidar.d = num(l_node, "circle_spacing_cm", 0.0) * cm;
            lidar.fov_circles = l_node["fov_circles"] ? l_node["fov_circles"].as<std::size_t>(0) : 0;
            composition.lidars.push_back(lidar);
        }
    }

    return composition;
}

} // namespace drone_mapper
