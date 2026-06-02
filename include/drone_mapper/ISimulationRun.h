#pragma once

#include <cpp_course/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class ISimulationRun {
public:
    virtual ~ISimulationRun() = default;

    [[nodiscard]] virtual types::MissionRunResult run() = 0;
};

} // namespace drone_mapper
