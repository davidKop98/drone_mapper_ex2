#pragma once
// Hand-rolled IGPS test fixture, deliberately independent of MockGPS: a fixed pose
// with direct setters -- no rounding logic, no shared-truth machinery, no code shared
// with MockGPS. Suites whose subject merely NEEDS a pose source (MockLidar,
// DroneControl) use this so no MockGPS mutation can propagate into them;
// SimulationRun keeps the real MockGPS -- it is the suite that grades it.
#include <drone_mapper/IGPS.h>

namespace drone_mapper::test_support {

class FakeGps final : public IGPS {
public:
    FakeGps(Position3D position, Orientation heading)
        : position_(position), heading_(heading) {}

    [[nodiscard]] Position3D position() const override { return position_; }
    [[nodiscard]] Orientation heading() const override { return heading_; }

    void setPosition(const Position3D& p) { position_ = p; }
    void setHeading(const Orientation& h) { heading_ = h; }

private:
    Position3D position_;
    Orientation heading_;
};

} // namespace drone_mapper::test_support
