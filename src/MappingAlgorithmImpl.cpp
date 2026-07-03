#include <drone_mapper/MappingAlgorithmImpl.h>

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Units.h>

#include <algorithm>
#include <cmath>
#include <functional>

namespace drone_mapper {
namespace {

// Full-precision pi (rule d: a truncated pi drifts headings through
// atan2 -> deg -> rad -> cos/sin and leaks samples one cell sideways).
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsCm = 1e-4;
constexpr double kEpsDeg = 1e-4;

double toDeg(double r) { return r * 180.0 / kPi; }

// Wrap a signed angle to (-180, 180].
double wrapTo180(double d) {
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

double Xc(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
double Yc(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
double Zc(const Position3D& p) { return p.z.force_numerical_value_in(cm); }

Position3D makePos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

types::MovementCommand rotateCommand(double signed_deg) {
    types::MovementCommand c{};
    c.type = types::MovementCommandType::Rotate;
    c.rotation = (signed_deg >= 0.0) ? types::RotationDirection::Left
                                     : types::RotationDirection::Right;
    c.angle = std::abs(signed_deg) * horizontal_angle[deg];
    return c;
}

types::MovementCommand advanceCommand(double cm_) {
    types::MovementCommand c{};
    c.type = types::MovementCommandType::Advance;
    c.distance = cm_ * cm; // forward only -> always positive
    return c;
}

types::MovementCommand elevateCommand(double cm_) {
    types::MovementCommand c{};
    c.type = types::MovementCommandType::Elevate;
    c.distance = cm_ * cm; // signed: negative goes down
    return c;
}

types::MappingStepCommand scanStep(double xy_deg, double el_deg) {
    return types::MappingStepCommand{
        std::nullopt,
        Orientation{xy_deg * horizontal_angle[deg], el_deg * altitude_angle[deg]},
        types::AlgorithmStatus::Working,
    };
}

types::MappingStepCommand moveStep(const types::MovementCommand& cmd) {
    return types::MappingStepCommand{cmd, std::nullopt, types::AlgorithmStatus::Working};
}

} // namespace

std::size_t MappingAlgorithmImpl::CellKeyHash::operator()(const CellKey& key) const noexcept {
    std::size_t seed = std::hash<long long>{}(std::get<0>(key));
    seed ^= std::hash<long long>{}(std::get<1>(key)) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= std::hash<long long>{}(std::get<2>(key)) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
}

// 10 fixed candidate directions, clockwise convention (+Y = south). Diagonals are
// (+-1, +-1) so target = current + step*dir lands on the adjacent diagonal cell; the
// advance distance becomes step*sqrt(2) at the atan2-derived heading.
const std::array<std::array<double, 3>, 10> MappingAlgorithmImpl::kDirections = {{
    {1.0, 0.0, 0.0},   //   0 deg east
    {1.0, 1.0, 0.0},   //  45 deg southeast
    {0.0, 1.0, 0.0},   //  90 deg south
    {-1.0, 1.0, 0.0},  // 135 deg southwest
    {-1.0, 0.0, 0.0},  // 180 deg west
    {-1.0, -1.0, 0.0}, // 225 deg northwest
    {0.0, -1.0, 0.0},  // 270 deg north
    {1.0, -1.0, 0.0},  // 315 deg northeast
    {0.0, 0.0, 1.0},   // up
    {0.0, 0.0, -1.0},  // down
}};

double MappingAlgorithmImpl::moveStepCm() const {
    return output_map_.getMapConfig().resolution.force_numerical_value_in(cm);
}

double MappingAlgorithmImpl::clearanceCm() const {
    const double radius = drone_config_.radius.force_numerical_value_in(cm);
    const double gps = mission_config_.gps_resolution.force_numerical_value_in(cm);
    return std::max(0.0, radius) + 0.5 * std::max(0.0, gps); //might need to make 0.5 to 1 to be safe
}

double MappingAlgorithmImpl::stepAngleDeg() const {
    // 2x-wider scan angles (was [2,10] deg, *1) -> ~1/4 the beams per sphere scan, to cut
    // wall-clock. Coarser angular sampling may miss thin features; watch the score.
    constexpr double kMaxStep = 20.0;
    constexpr double kMinStep = 4.0;
    const double spacing = lidar_config_.d.force_numerical_value_in(cm);
    const double beam_min = lidar_config_.z_min.force_numerical_value_in(cm);
    if (spacing <= 0.0 || beam_min <= 0.0) return kMaxStep;
    const double step = toDeg(std::atan(spacing / beam_min)) * 2.0;
    return std::clamp(step, kMinStep, kMaxStep);
}

MappingAlgorithmImpl::CellKey MappingAlgorithmImpl::cellOf(const Position3D& pos) const {
    const types::MapConfig cfg = output_map_.getMapConfig();
    const double res = cfg.resolution.force_numerical_value_in(cm); //OUTPut map res
    if (res <= 0.0) return {0, 0, 0}; //res shouldnt be 0 but just in case
    const double ox = cfg.offset.x.force_numerical_value_in(cm);
    const double oy = cfg.offset.y.force_numerical_value_in(cm);
    const double oz = cfg.offset.z.force_numerical_value_in(cm);
    return {
        static_cast<long long>(std::floor((Xc(pos) - ox) / res + 1e-9)), //epsilon for FP error
        static_cast<long long>(std::floor((Yc(pos) - oy) / res + 1e-9)),
        static_cast<long long>(std::floor((Zc(pos) - oz) / res + 1e-9)),
    };
}

bool MappingAlgorithmImpl::isEmptyCell(const Position3D& pos) const {
    return output_map_.atVoxel(pos) == types::VoxelOccupancy::Empty;
}

// The full radius + gps/2 footprint around `center` must be entirely confirmed Empty:
// any non-Empty sample (Occupied / PotentiallyOccupied / OutOfBounds / Unmapped) blocks.
// This mirrors EX1's footprint rule (BuildingMap::get() is 0 iff Empty, and canAdvance
// rejects any non-Empty swept sample), so the drone's body plus gps uncertainty never
// overlaps wall OR unknown space -- it scans from a standoff to confirm space Empty,
// then advances into it. Sampled as a per-axis box so the margin holds on every axis.
bool MappingAlgorithmImpl::hasClearance(const Position3D& center) const {
    const double c = clearanceCm(); //The radius we are checking
    if (c <= 0.0) return true;
    const double half_cell = 0.5 * moveStepCm();
    const int n = std::max(1, static_cast<int>(std::ceil(c / std::max(half_cell, 1e-9))));
    const double s = c / n; // <= half a cell, and -n..n hits +-c exactly
    for (int ix = -n; ix <= n; ++ix) {
        for (int iy = -n; iy <= n; ++iy) {
            for (int iz = -n; iz <= n; ++iz) {
                const Position3D sample =
                    makePos(Xc(center) + ix * s, Yc(center) + iy * s, Zc(center) + iz * s);
                if (output_map_.atVoxel(sample) != types::VoxelOccupancy::Empty) return false;
            }
        }
    }
    return true;
}

// The swept path from start to target stays confirmed-Empty and clear of walls.
bool MappingAlgorithmImpl::pathSafe(const Position3D& start, const Position3D& target) const {
    const double dx = Xc(target) - Xc(start);
    const double dy = Yc(target) - Yc(start);
    const double dz = Zc(target) - Zc(start);
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double sub = 0.5 * moveStepCm();
    const int n = std::max(1, static_cast<int>(std::ceil(dist / std::max(sub, 1e-9)))); //number of steps
    for (int i = 0; i <= n; ++i) {
        const double f = static_cast<double>(i) / n;
        const Position3D p = makePos(Xc(start) + f * dx, Yc(start) + f * dy, Zc(start) + f * dz);
        if (output_map_.atVoxel(p) != types::VoxelOccupancy::Empty) return false; // non-traversable
        if (!hasClearance(p)) return false;
    }
    return true;
}

bool MappingAlgorithmImpl::anyInBoundsUnmapped() const {
    const types::MapConfig cfg = output_map_.getMapConfig();
    const double res = cfg.resolution.force_numerical_value_in(cm);
    if (res <= 0.0) return false;
    const auto& b = cfg.boundaries;
    const double min_x = b.min_x.force_numerical_value_in(cm);
    const double max_x = b.max_x.force_numerical_value_in(cm);
    const double min_y = b.min_y.force_numerical_value_in(cm);
    const double max_y = b.max_y.force_numerical_value_in(cm);
    const double min_z = b.min_height.force_numerical_value_in(cm);
    const double max_z = b.max_height.force_numerical_value_in(cm);
    for (double x = min_x + 0.5 * res; x < max_x; x += res) {
        for (double y = min_y + 0.5 * res; y < max_y; y += res) {
            for (double z = min_z + 0.5 * res; z < max_z; z += res) {
                if (output_map_.atVoxel(makePos(x, y, z)) == types::VoxelOccupancy::Unmapped) {
                    return true;
                }
            }
        }
    }
    return false;
}
//rounded position based on gps_resolution
bool MappingAlgorithmImpl::findNextTarget(const Position3D& pos) {
    const double step = moveStepCm();
    const double px = Xc(pos);
    const double py = Yc(pos);
    const double pz = Zc(pos);
    for (const auto& dir : kDirections) {
        const Position3D cand = makePos(px + step * dir[0], py + step * dir[1], pz + step * dir[2]);
        if (visited_.count(cellOf(cand))) continue;
        if (!isEmptyCell(cand)) continue;     // target must be confirmed Empty
        if (!pathSafe(pos, cand)) continue;   // swept-sphere clearance + traversable path
        next_target_ = cand;
        return true;
    }
    return false;
}

types::MovementCommand MappingAlgorithmImpl::emitRotateChunk(double& remaining) const {
    const double cap = drone_config_.max_rotate.force_numerical_value_in(deg);
    double chunk;
    if (cap <= 0.0 || std::abs(remaining) <= cap) {
        chunk = remaining;
        remaining = 0.0;
    } else {
        chunk = (remaining > 0.0) ? cap : -cap;
        remaining -= chunk;
    }
    return rotateCommand(chunk);
}

types::MovementCommand MappingAlgorithmImpl::emitAdvanceChunk(double& remaining) const {
    const double cap = drone_config_.max_advance.force_numerical_value_in(cm);
    double chunk;
    if (cap <= 0.0 || std::abs(remaining) <= cap) {
        chunk = remaining;
        remaining = 0.0;
    } else {
        chunk = (remaining > 0.0) ? cap : -cap;
        remaining -= chunk;
    }
    return advanceCommand(chunk);
}

types::MovementCommand MappingAlgorithmImpl::emitElevateChunk(double& remaining) const {
    const double cap = drone_config_.max_elevate.force_numerical_value_in(cm);
    double chunk;
    if (cap <= 0.0 || std::abs(remaining) <= cap) {
        chunk = remaining;
        remaining = 0.0;
    } else {
        chunk = (remaining > 0.0) ? cap : -cap;
        remaining -= chunk;
    }
    return elevateCommand(chunk);
}

void MappingAlgorithmImpl::pushChunkedRotate(double total) {
    const double cap = drone_config_.max_rotate.force_numerical_value_in(deg);
    if (std::abs(total) <= kEpsDeg) return;
    if (cap <= 0.0) {
        inverse_stack_.push_back(InverseMove{false, rotateCommand(total)});
        return;
    }
    double remaining = total;
    while (std::abs(remaining) > cap) {
        const double chunk = (remaining > 0.0) ? cap : -cap;
        inverse_stack_.push_back(InverseMove{false, rotateCommand(chunk)});
        remaining -= chunk;
    }
    if (std::abs(remaining) > kEpsDeg) {
        inverse_stack_.push_back(InverseMove{false, rotateCommand(remaining)});
    }
}

void MappingAlgorithmImpl::pushChunkedAdvance(double total) {
    const double cap = drone_config_.max_advance.force_numerical_value_in(cm);
    if (std::abs(total) <= kEpsCm) return;
    if (cap <= 0.0) {
        inverse_stack_.push_back(InverseMove{false, advanceCommand(total)});
        return;
    }
    double remaining = total;
    while (std::abs(remaining) > cap) {
        const double chunk = (remaining > 0.0) ? cap : -cap;
        inverse_stack_.push_back(InverseMove{false, advanceCommand(chunk)});
        remaining -= chunk;
    }
    if (std::abs(remaining) > kEpsCm) {
        inverse_stack_.push_back(InverseMove{false, advanceCommand(remaining)});
    }
}

void MappingAlgorithmImpl::pushChunkedElevate(double total) {
    const double cap = drone_config_.max_elevate.force_numerical_value_in(cm);
    if (std::abs(total) <= kEpsCm) return;
    if (cap <= 0.0) {
        inverse_stack_.push_back(InverseMove{false, elevateCommand(total)});
        return;
    }
    double remaining = total;
    while (std::abs(remaining) > cap) {
        const double chunk = (remaining > 0.0) ? cap : -cap;
        inverse_stack_.push_back(InverseMove{false, elevateCommand(chunk)});
        remaining -= chunk;
    }
    if (std::abs(remaining) > kEpsCm) {
        inverse_stack_.push_back(InverseMove{false, elevateCommand(remaining)});
    }
}

void MappingAlgorithmImpl::startScan() {
    scan_xy_deg_ = 0.0;
    scan_el_deg_ = -90.0;
    phase_ = Phase::Scanning;
}

types::MappingStepCommand MappingAlgorithmImpl::finishedStep() const {
    const types::AlgorithmStatus status = anyInBoundsUnmapped()
                                              ? types::AlgorithmStatus::FinishedWithUnmappableVoxels
                                              : types::AlgorithmStatus::Finished;
    return types::MappingStepCommand{std::nullopt, std::nullopt, status};
}
//position here is rounded based on gps_resolution
types::MappingStepCommand MappingAlgorithmImpl::step(const Position3D& pos, double heading_deg) {
    switch (phase_) {
    case Phase::Scanning: {
        const double xy = scan_xy_deg_;
        const double el = scan_el_deg_;
        const double step_deg = stepAngleDeg();
        double next_xy = xy + step_deg;
        double next_el = el;
        if (next_xy >= 360.0) {
            next_xy = 0.0;
            next_el += step_deg;
            if (next_el > 90.0) {
                // Sphere scan complete at this cell.
                visited_.insert(cellOf(pos));
                phase_ = Phase::ChoosingNext;
            }
        }
        scan_xy_deg_ = next_xy;
        scan_el_deg_ = next_el;
        return scanStep(xy, el);
    }

    case Phase::ChoosingNext: {
        if (findNextTarget(pos)) {
            inverse_stack_.push_back(InverseMove{true, {}}); // level marker
            phase_ = Phase::Moving;
            return step(pos, heading_deg);
        }
        if (inverse_stack_.empty()) {
            phase_ = Phase::Finished;
            return finishedStep();
        }
        phase_ = Phase::Backtracking;
        return step(pos, heading_deg);
    }

    case Phase::Moving: {
        // Drain pending chunks first (only one is ever non-zero).
        if (std::abs(pending_rotate_deg_) > kEpsDeg) {
            return moveStep(emitRotateChunk(pending_rotate_deg_));
        }
        if (std::abs(pending_advance_cm_) > kEpsCm) {
            const types::MovementCommand cmd = emitAdvanceChunk(pending_advance_cm_);
            if (std::abs(pending_advance_cm_) <= kEpsCm) startScan();
            return moveStep(cmd);
        }
        if (std::abs(pending_elevate_cm_) > kEpsCm) {
            const types::MovementCommand cmd = emitElevateChunk(pending_elevate_cm_);
            if (std::abs(pending_elevate_cm_) <= kEpsCm) startScan();
            return moveStep(cmd);
        }

        // No pending chunk: compute deltas to the target.
        const double dx = Xc(next_target_) - Xc(pos);
        const double dy = Yc(next_target_) - Yc(pos);
        const double dz = Zc(next_target_) - Zc(pos);

        if (std::abs(dz) > kEpsCm) { //Z delta > 0 means we elevate
            pending_elevate_cm_ = dz;
            pushChunkedElevate(-dz); // inverse:push opposite sign
            return step(pos, heading_deg);
        }

        const double horiz = std::sqrt(dx * dx + dy * dy);
        if (horiz > kEpsCm) {
            const double required_deg = toDeg(std::atan2(dy, dx));
            const double delta = wrapTo180(required_deg - heading_deg);
            if (std::abs(delta) > kEpsDeg) {
                pending_rotate_deg_ = delta;
                pushChunkedRotate(-delta); // inverse rotation
                return step(pos, heading_deg);
            }
            pending_advance_cm_ = horiz;
            // 3-part inverse for advance (backtrack executes Rot(180), Adv, Rot(-180)).
            pushChunkedRotate(-180.0);
            pushChunkedAdvance(horiz);
            pushChunkedRotate(180.0);
            return step(pos, heading_deg);
        }

        // Already at target (degenerate). Pop the level marker and re-choose.
        if (!inverse_stack_.empty() && inverse_stack_.back().is_level_marker) {
            inverse_stack_.pop_back();
        }
        phase_ = Phase::ChoosingNext;
        return step(pos, heading_deg);
    }

    case Phase::Backtracking: {
        if (inverse_stack_.empty()) {
            phase_ = Phase::Finished;
            return finishedStep();
        }
        const InverseMove move = inverse_stack_.back();
        inverse_stack_.pop_back();
        if (move.is_level_marker) {
            // One DFS level fully undone -> back at the parent; try a sibling
            // direction without rescanning.
            phase_ = Phase::ChoosingNext;
            return step(pos, heading_deg);
        }
        return moveStep(move.command);
    }

    case Phase::Finished:
    default:
        return finishedStep();
    }
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                        const types::LidarScanResult* latest_scan) {
    // latest_scan is intentionally unused for decisions: the output map is the source
    // of truth (the converter writes scans into it before the next call).
    (void)latest_scan;
    return step(state.position, state.heading.horizontal.force_numerical_value_in(deg));
}

} // namespace drone_mapper
