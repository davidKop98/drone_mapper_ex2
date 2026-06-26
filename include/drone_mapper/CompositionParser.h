#pragma once

#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>
#include <optional>
#include <string>

namespace drone_mapper {

// Resolve the composition file path per the CLI rules: no argument -> "simulation.yaml"
// in the cwd; a relative path stays relative to the cwd; an absolute path is used as-is.
[[nodiscard]] std::filesystem::path resolveCompositionPath(const std::optional<std::string>& arg);

// Parse a composition YAML into the grouped SimulationCompositionData. Missing keys fall
// back to sane defaults; throws std::runtime_error only when the file cannot be loaded.
//
// Expected schema:
//   simulations:
//     - map_filename: <path>
//       map_resolution_cm: <num>
//       map_offset_cm: [x, y, z]
//       initial_drone_position_cm: [x, y, z]
//       initial_angle_deg: <num>
//       missions:
//         - max_steps: <int>
//           gps_resolution_cm: <num>
//           output_mapping_resolution_factor: <num>
//           boundaries_cm: [min_x, min_y, min_z, max_x, max_y, max_z]
//   drones:
//     - dimensions_cm: <num>     # diameter; radius = dimensions_cm / 2
//       max_rotate_deg: <num>
//       max_advance_cm: <num>
//       max_elevate_cm: <num>
//   lidars:
//     - z_min_cm: <num>
//       z_max_cm: <num>
//       circle_spacing_cm: <num> # the lidar's d
//       fov_circles: <int>
[[nodiscard]] types::SimulationCompositionData parseCompositionYaml(const std::filesystem::path& path);

} // namespace drone_mapper
