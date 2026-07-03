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

namespace fs = std::filesystem;

double num(const YAML::Node& node, const char* key, double fallback) {
    return (node && node[key]) ? node[key].as<double>(fallback) : fallback;
}

YAML::Node loadFile(const fs::path& file) {
    try {
        return YAML::LoadFile(file.string());
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to load '" + file.string() + "': " + e.what());
    }
}

// A relative reference in a composition is resolved against the composition file's directory
// (the "inputs/" root); an absolute path is used as-is.
fs::path resolveRef(const fs::path& base_dir, const std::string& ref) {
    const fs::path p{ref};
    return p.is_absolute() ? p : (base_dir / p);
}

// One {min_cm, max_cm} boundary sub-node -> (lo, hi).
std::pair<double, double> readAxisBounds(const YAML::Node& node) {
    return {num(node, "min_cm", 0.0), num(node, "max_cm", 0.0)};
}

// simulation/<name>.yaml : a single "simulation_config" block. map_filename is relative to the
// composition root (base_dir), resolved to an absolute path so the run works from any cwd.
types::SimulationConfigData parseSimulation(const fs::path& base_dir, const fs::path& file) {
    const YAML::Node root = loadFile(file);
    const YAML::Node c = root["simulation_config"] ? root["simulation_config"] : root;

    types::SimulationConfigData sim;
    if (c["map_filename"]) {
        sim.map_filename = resolveRef(base_dir, c["map_filename"].as<std::string>());
    }
    sim.map_resolution = num(c, "map_resolution_cm", 10.0) * cm;

    const YAML::Node off = c["map_axes_offset"];
    sim.map_offset = Position3D{
        num(off, "x_offset", 0.0) * x_extent[cm],
        num(off, "y_offset", 0.0) * y_extent[cm],
        num(off, "height_offset", 0.0) * z_extent[cm],
    };

    const YAML::Node pos = c["initial_drone_position"];
    sim.initial_drone_position = Position3D{
        num(pos, "x_cm", 0.0) * x_extent[cm],
        num(pos, "y_cm", 0.0) * y_extent[cm],
        num(pos, "height_cm", 0.0) * z_extent[cm],
    };
    sim.initial_angle = num(c, "initial_angle_deg", 0.0) * horizontal_angle[deg];
    return sim;
}

// mission/<name>.yaml : a single "mission_config" block with nested per-axis boundaries.
types::MissionConfigData parseMission(const fs::path& file) {
    const YAML::Node root = loadFile(file);
    const YAML::Node c = root["mission_config"] ? root["mission_config"] : root;

    types::MissionConfigData m;
    m.max_steps = c["max_steps"] ? c["max_steps"].as<std::size_t>(0) : 0;
    m.gps_resolution = num(c, "gps_resolution_cm", 10.0) * cm;
    // Not present in the staff schema -> default 1.0 (output map uses the input-map resolution).
    m.output_mapping_resolution_factor = num(c, "output_mapping_resolution_factor", 1.0);

    const YAML::Node b = c["boundaries"];
    const auto [x_lo, x_hi] = readAxisBounds(b ? b["x_boundary"] : YAML::Node());
    const auto [y_lo, y_hi] = readAxisBounds(b ? b["y_boundary"] : YAML::Node());
    const auto [z_lo, z_hi] = readAxisBounds(b ? b["height_boundary"] : YAML::Node());
    m.mission_bounds = types::MappingBounds{
        x_lo * x_extent[cm], x_hi * x_extent[cm],
        y_lo * y_extent[cm], y_hi * y_extent[cm],
        z_lo * z_extent[cm], z_hi * z_extent[cm],
    };
    return m;
}

// drone/<name>.yaml : a single "drone_config" block. dimensions_cm is the sphere DIAMETER.
types::DroneConfigData parseDrone(const fs::path& file) {
    const YAML::Node root = loadFile(file);
    const YAML::Node c = root["drone_config"] ? root["drone_config"] : root;

    types::DroneConfigData d;
    d.radius = 0.5 * num(c, "dimensions_cm", 0.0) * cm; // diameter -> radius
    d.max_rotate = num(c, "max_rotate_deg", 0.0) * horizontal_angle[deg];
    d.max_advance = num(c, "max_advance_cm", 0.0) * cm;
    d.max_elevate = num(c, "max_elevate_cm", 0.0) * cm;
    return d;
}

// lidar/<name>.yaml : a single "lidar_config" block. d_cm is the beam-circle spacing at z_min.
types::LidarConfigData parseLidar(const fs::path& file) {
    const YAML::Node root = loadFile(file);
    const YAML::Node c = root["lidar_config"] ? root["lidar_config"] : root;

    types::LidarConfigData l;
    l.z_min = num(c, "z_min_cm", 0.0) * cm;
    l.z_max = num(c, "z_max_cm", 0.0) * cm;
    l.d = num(c, "d_cm", 0.0) * cm;
    l.fov_circles = c["fov_circles"] ? c["fov_circles"].as<std::size_t>(0) : 0;
    return l;
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
    const YAML::Node root = loadFile(path);
    // All references inside the composition are relative to its own directory (the inputs root).
    const fs::path base_dir = path.has_parent_path() ? path.parent_path() : fs::path{"."};

    types::SimulationCompositionData composition;
    composition.composition_file = path;

    const YAML::Node comps = root["simulation_compositions"];
    if (!comps) {
        return composition; // nothing to run
    }

    const YAML::Node simulations = comps["simulations"];
    if (simulations && simulations.IsSequence()) {
        for (const YAML::Node& s : simulations) {
            if (!s["simulation_config"]) continue;
            types::SimulationConfigData sim =
                parseSimulation(base_dir, resolveRef(base_dir, s["simulation_config"].as<std::string>()));

            std::vector<types::MissionConfigData> missions;
            const YAML::Node mission_refs = s["mission_configs"];
            if (mission_refs && mission_refs.IsSequence()) {
                for (const YAML::Node& ref : mission_refs) {
                    missions.push_back(parseMission(resolveRef(base_dir, ref.as<std::string>())));
                }
            }
            composition.simulation_mission_groups.emplace_back(std::move(sim), std::move(missions));
        }
    }

    const YAML::Node drone_refs = comps["drone_configs"];
    if (drone_refs && drone_refs.IsSequence()) {
        for (const YAML::Node& ref : drone_refs) {
            composition.drones.push_back(parseDrone(resolveRef(base_dir, ref.as<std::string>())));
        }
    }

    const YAML::Node lidar_refs = comps["lidar_configs"];
    if (lidar_refs && lidar_refs.IsSequence()) {
        for (const YAML::Node& ref : lidar_refs) {
            composition.lidars.push_back(parseLidar(resolveRef(base_dir, ref.as<std::string>())));
        }
    }

    return composition;
}

} // namespace drone_mapper
