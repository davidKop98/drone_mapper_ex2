#pragma once

#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/Units.h>

#include <memory>

namespace drone_mapper {

class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

    // The output resolution to use and whether the requested factor was honored.
    struct ResolutionDecision {
        PhysicalLength resolution;
        types::ResolutionRequestStatus status;
    };

    // ---- Construction pieces, exposed so they can be unit-tested in isolation ----

    // Load the hidden ground-truth .npy into an offset/resolution-aware Map3DImpl
    // (boundaries = the map's full extent). Throws std::runtime_error on load failure or
    // a non-3D array. Decode is Map3DImpl's single shared rule; ground-truth maps use
    // 0/nonzero (never the 253/255 output sentinels), so it yields the ground-truth
    // "0 -> Empty, else -> Occupied" semantics.
    [[nodiscard]] static std::unique_ptr<Map3DImpl> loadHiddenMap(const types::SimulationConfigData& sim);

    // An all-Unmapped output map over the mapping box: boundaries = bounds,
    // offset = bounds.min, the given resolution.
    [[nodiscard]] static std::unique_ptr<Map3DImpl> makeOutputMap(const types::MappingBounds& bounds,
                                                                  PhysicalLength resolution);

    // We always operate at `default_resolution` (the single supported output resolution);
    // the requested factor only determines the reported status (factor 1 -> Accepted,
    // 0<factor<1 -> IgnoredTooSmall, otherwise Ignored).
    [[nodiscard]] static ResolutionDecision resolveOutputResolution(const types::MissionConfigData& mission,
                                                                    PhysicalLength default_resolution);
};

} // namespace drone_mapper
