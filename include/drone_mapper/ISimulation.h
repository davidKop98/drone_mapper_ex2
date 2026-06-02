#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

// **Do not change this interface.**
class ISimulation {
public:
    virtual ~ISimulation() = default;

    [[nodiscard]] virtual types::SimulationReport run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) = 0;
};

} // namespace drone_mapper
