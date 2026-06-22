#include <drone_mapper/MockGPS.h>

#include <drone_mapper/Units.h>

#include <cmath>
#include <utility>

namespace drone_mapper {
namespace {

// Round one axis value (in cm) to the nearest multiple of res (in cm).
// std::round is round-half-away-from-zero, which is what the TA example expects
// (e.g. 18 @ res 5 -> 20, and -17 @ res 5 -> -15).
[[nodiscard]] double roundToNearest(double value_cm, double res_cm) {
    return std::round(value_cm / res_cm) * res_cm;
}

} // namespace

MockGPS::MockGPS(std::shared_ptr<GpsTruth> truth, PhysicalLength resolution)
    : truth_(std::move(truth)), resolution_(resolution) {
    if (!truth_) {
        truth_ = std::make_shared<GpsTruth>();
    }
}

MockGPS::MockGPS(Position3D position, Orientation heading, PhysicalLength resolution)
    : truth_(std::make_shared<GpsTruth>(GpsTruth{position, heading})),
      resolution_(resolution) {}

Position3D MockGPS::position() const {
    const Position3D& p = truth_->position;
    const double res = resolution_.force_numerical_value_in(cm);
    if (res <= 0.0) {
        return p; // exact view
    }
    // Per-axis: extract to cm the way MockLidar::traceBeam does, round to nearest,
    // then rebuild the per-axis quantity. We never divide across mixed axis specs.
    const double rx = roundToNearest(p.x.force_numerical_value_in(cm), res);
    const double ry = roundToNearest(p.y.force_numerical_value_in(cm), res);
    const double rz = roundToNearest(p.z.force_numerical_value_in(cm), res);
    return Position3D{rx * x_extent[cm], ry * y_extent[cm], rz * z_extent[cm]};
}

Orientation MockGPS::heading() const {
    return truth_->heading;
}

void MockGPS::setPose(const Position3D& position, const Orientation& heading) {
    truth_->position = position;
    truth_->heading = heading;
}

void MockGPS::setPosition(const Position3D& position) {
    truth_->position = position;
}

void MockGPS::setHeading(const Orientation& heading) {
    truth_->heading = heading;
}

} // namespace drone_mapper
