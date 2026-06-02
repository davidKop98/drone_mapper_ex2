#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <utility>

namespace drone_mapper::types {

struct MappingBounds {
    MappingBounds() = delete;

    MappingBounds(XLength min_x_value,
                  XLength max_x_value,
                  YLength min_y_value,
                  YLength max_y_value,
                  ZLength min_height_value,
                  ZLength max_height_value)
        : min_x(min_x_value),
          max_x(max_x_value),
          min_y(min_y_value),
          max_y(max_y_value),
          min_height(min_height_value),
          max_height(max_height_value) {}

    XLength min_x;
    XLength max_x;
    YLength min_y;
    YLength max_y;
    ZLength min_height;
    ZLength max_height;
};

struct MissionConfigData {
    MissionConfigData() = delete;

    MissionConfigData(std::size_t max_steps_value,
                      MappingBounds boundaries_value,
                      PhysicalLength gps_resolution_value,
                      int output_mapping_resolution_factor_value)
        : max_steps(max_steps_value),
          boundaries(boundaries_value),
          gps_resolution(gps_resolution_value),
          output_mapping_resolution_factor(output_mapping_resolution_factor_value) {}

    std::size_t max_steps;
    MappingBounds boundaries;
    PhysicalLength gps_resolution;
    int output_mapping_resolution_factor;
};

enum class MissionRunStatus {
    Completed,
    MaxSteps,
    Error,
};

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    double score = 0.0;
    std::filesystem::path output_map_file{};
    ErrorRef error{};
};

} // namespace drone_mapper::types
