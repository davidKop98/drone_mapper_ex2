#pragma once

#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>
#include <optional>
#include <string>

namespace drone_mapper {

// Resolve the composition file path per the CLI rules: no argument -> "simulation.yaml"
// in the cwd; a relative path stays relative to the cwd; an absolute path is used as-is.
[[nodiscard]] std::filesystem::path resolveCompositionPath(const std::optional<std::string>& arg);

// Parse a composition YAML into the grouped SimulationCompositionData. Missing keys fall back
// to sane defaults; throws std::runtime_error when the composition file, or any file it
// references, cannot be loaded. All references (and each simulation's map_filename) are
// resolved relative to the composition file's own directory.
//
// Expected schema (hierarchical, multi-file):
//   sim_compose.yaml:
//     simulation_compositions:
//       simulations:
//         - simulation_config: "simulation/<name>.yaml"
//           mission_configs: ["mission/<name>.yaml", ...]
//       drone_configs: ["drone/<name>.yaml", ...]
//       lidar_configs: ["lidar/<name>.yaml", ...]
//
//   simulation/<name>.yaml:
//     simulation_config:
//       map_filename: <path relative to the composition dir>
//       map_resolution_cm: <num>
//       initial_drone_position: {x_cm, y_cm, height_cm}
//       initial_angle_deg: <num>
//       map_axes_offset: {x_offset, y_offset, height_offset}
//
//   mission/<name>.yaml:
//     mission_config:
//       max_steps: <int>
//       gps_resolution_cm: <num>
//       output_mapping_resolution_factor: <num>   # optional, default 1.0
//       boundaries: {x_boundary|y_boundary|height_boundary: {min_cm, max_cm}}
//
//   drone/<name>.yaml:
//     drone_config: {dimensions_cm, max_rotate_deg, max_advance_cm, max_elevate_cm}
//                    # dimensions_cm is the sphere DIAMETER; radius = dimensions_cm / 2
//
//   lidar/<name>.yaml:
//     lidar_config: {z_min_cm, z_max_cm, d_cm, fov_circles}
[[nodiscard]] types::SimulationCompositionData parseCompositionYaml(const std::filesystem::path& path);

} // namespace drone_mapper
