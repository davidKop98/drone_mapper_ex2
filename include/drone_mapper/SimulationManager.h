#pragma once

#include <cpp_course/ISimulation.h>
#include <cpp_course/ISimulationRunFactory.h>

#include <memory>

namespace drone_mapper {

class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory);

    [[nodiscard]] types::SimulationReport run(const types::SimulationCompositionData& composition,
                                              const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace drone_mapper
