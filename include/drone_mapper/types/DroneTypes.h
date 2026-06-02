#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <string>

namespace drone_mapper::types {

struct DroneConfigData {
    DroneConfigData() = delete;

    DroneConfigData(PhysicalLength dimensions_value,
                    HorizontalAngle max_rotate_value,
                    PhysicalLength max_advance_value,
                    PhysicalLength max_elevate_value)
        : dimensions(dimensions_value),
          max_rotate(max_rotate_value),
          max_advance(max_advance_value),
          max_elevate(max_elevate_value) {}

    PhysicalLength dimensions;
    HorizontalAngle max_rotate;
    PhysicalLength max_advance;
    PhysicalLength max_elevate;
};

enum class RotationDirection {
    Left,
    Right,
};

enum class MovementCommandType {
    Hover,
    Rotate,
    Advance,
    Elevate,
};

struct MovementCommand {
    MovementCommandType type = MovementCommandType::Hover;
    RotationDirection rotation = RotationDirection::Left;
    HorizontalAngle angle = 0.0 * horizontal_angle[deg];
    PhysicalLength distance = 0.0 * cm;
};

struct MovementResult {
    bool success = true;
    std::string message{};

    [[nodiscard]] explicit operator bool() const noexcept {
        return success;
    }
};

struct DroneState {
    Position3D position{};
    Orientation heading{};
    std::size_t step_index = 0;
};

enum class DroneStepStatus {
    Continue,
    Completed,
    Error,
};

struct DroneStepResult {
    DroneStepStatus status = DroneStepStatus::Continue;
    std::string message{};
};

} // namespace drone_mapper::types
