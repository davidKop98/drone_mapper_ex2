#pragma once

#include <cpp_course/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class IMissionControl {
public:
    virtual ~IMissionControl() = default;

    [[nodiscard]] virtual types::MissionRunResult runMission() = 0;
};

} // namespace drone_mapper
