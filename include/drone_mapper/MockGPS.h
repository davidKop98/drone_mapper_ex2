#pragma once

#include <drone_mapper/IGPS.h>

#include <memory>

namespace drone_mapper {

// The exact, shared ground-truth pose. One truth can back many views (see MockGPS):
// e.g. an exact view for the lidar/movement and a gps_resolution-rounded view for
// the drone/algorithm. Only the truth is mutated; views never diverge.
struct GpsTruth {
    Position3D position{};
    Orientation heading{};
};

// A read view over a shared exact truth pose.
//   position(): rounded to the NEAREST multiple of `resolution` per axis
//               (the gps_resolution feature); resolution <= 0 returns the exact value.
//   heading():  always the unrounded truth heading.
class MockGPS final : public IGPS {
public:
    // View over an existing shared truth.
    MockGPS(std::shared_ptr<GpsTruth> truth, PhysicalLength resolution);
    // Convenience: create a fresh shared truth from an initial pose.
    MockGPS(Position3D position, Orientation heading, PhysicalLength resolution);

    [[nodiscard]] Position3D position() const override;
    [[nodiscard]] Orientation heading() const override;

    // Writes the shared exact truth (MockMovement uses these; tests call directly).
    void setPose(const Position3D& position, const Orientation& heading);
    void setPosition(const Position3D& position);
    void setHeading(const Orientation& heading);

    // The shared truth, so sibling views (e.g. exact + rounded) can be built over it.
    [[nodiscard]] std::shared_ptr<GpsTruth> truth() const { return truth_; }

private:
    std::shared_ptr<GpsTruth> truth_;
    PhysicalLength resolution_{};
};

} // namespace drone_mapper
