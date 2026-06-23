#include <drone_mapper/MockMovement.h>

#include <drone_mapper/Units.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>

namespace drone_mapper {
namespace {

double Xc(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
double Yc(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
double Zc(const Position3D& p) { return p.z.force_numerical_value_in(cm); }

Position3D makePos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

} // namespace

MockMovement::MockMovement(MockGPS& exact_gps, const IMap3D& hidden_map, PhysicalLength radius)
    : exact_gps_(exact_gps), hidden_map_(hidden_map), radius_(radius) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    // A sphere is heading-independent, so rotation never collides.
    const Orientation current = exact_gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    exact_gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D start = exact_gps_.position();
    const Orientation heading = exact_gps_.heading();
    // Direction from the exact heading, using the same mp-units trig as the lidar/converter.
    const auto cos_h = si::cos(heading.horizontal);
    const auto sin_h = si::sin(heading.horizontal);
    const double dist_cm = distance.force_numerical_value_in(cm);
    const double dir_x = cos_h.force_numerical_value_in(mp::one);
    const double dir_y = sin_h.force_numerical_value_in(mp::one);
    const Position3D end =
        makePos(Xc(start) + dir_x * dist_cm, Yc(start) + dir_y * dist_cm, Zc(start));

    if (sweptSphereHitsOccupied(start, end)) {
        return types::MovementResult{false, "DRONE_HITS_OBSTACLE"};
    }
    exact_gps_.setPosition(end);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D start = exact_gps_.position();
    const double dz_cm = distance.force_numerical_value_in(cm); // signed: negative goes down
    const Position3D end = makePos(Xc(start), Yc(start), Zc(start) + dz_cm);

    if (sweptSphereHitsOccupied(start, end)) {
        return types::MovementResult{false, "DRONE_HITS_OBSTACLE"};
    }
    exact_gps_.setPosition(end);
    return types::MovementResult{true, {}};
}

bool MockMovement::sweptSphereHitsOccupied(const Position3D& start, const Position3D& end) const {
    const double map_res = hidden_map_.getMapConfig().resolution.force_numerical_value_in(cm);
    const double res = (map_res > 0.0) ? map_res : 1.0;
    const double r = std::max(0.0, radius_.force_numerical_value_in(cm));

    const double dx = Xc(end) - Xc(start);
    const double dy = Yc(end) - Yc(start);
    const double dz = Zc(end) - Zc(start);
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Sub-cell stepping along the path and a per-axis radius footprint box at each step,
    // matching the algorithm's clearance sampling so the two snap to the same voxels.
    const double sub = 0.5 * res;
    const int path_steps = std::max(1, static_cast<int>(std::ceil(dist / sub)));
    const int n = std::max(0, static_cast<int>(std::ceil(r / (0.5 * res))));
    const double s = (n > 0) ? r / n : 0.0;

    for (int ip = 0; ip <= path_steps; ++ip) {
        const double f = static_cast<double>(ip) / path_steps;
        const double cx = Xc(start) + f * dx;
        const double cy = Yc(start) + f * dy;
        const double cz = Zc(start) + f * dz;
        for (int ix = -n; ix <= n; ++ix) {
            for (int iy = -n; iy <= n; ++iy) {
                for (int iz = -n; iz <= n; ++iz) {
                    const Position3D sample = makePos(cx + ix * s, cy + iy * s, cz + iz * s);
                    if (hidden_map_.atVoxel(sample) == types::VoxelOccupancy::Occupied) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

} // namespace drone_mapper
