#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <array>
#include <cstddef>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace drone_mapper {

// Fixed-direction DFS exploration ported from EX1, reshaped into a resume-per-call
// state machine. One nextStep() emits exactly one MappingStepCommand. Occupancy is
// read live through output_map_ (the locked const IMap3D&); there is no voxel mirror.
//
// Per position the drone sweeps a full sphere of scan orientations (Scanning), then
// picks the first reachable confirmed-Empty neighbor (ChoosingNext), rotates+advances
// or elevates toward it in capped chunks (Moving), and unwinds the inverse-move stack
// when a branch dead-ends (Backtracking). Collision is the sphere model: a move is only
// planned if the entire (radius + gps_resolution/2) footprint around every center on
// the swept path is confirmed Empty -- any non-Empty cell (Occupied, PotentiallyOccupied,
// OutOfBounds, or Unmapped) blocks it. So the footprint never overlaps unknown or wall
// space (matching EX1); the drone scans from a standoff to confirm space Empty, then
// advances into it, and the margin absorbs gps rounding so a safe-as-believed move can't
// crash as-true.
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm; // locked ctor (mission, lidar, drone, output_map)

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    enum class Phase { Scanning, ChoosingNext, Moving, Backtracking, Finished };

    using CellKey = std::tuple<long long, long long, long long>;
    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const noexcept;
    };
    // Inverse stack entry: either a movement to replay during backtracking, or a
    // level marker delimiting one DFS level.
    struct InverseMove {
        bool is_level_marker = false;
        types::MovementCommand command{};
    };

    // Recursive per-call decision: resumes from phase_ and emits one command.
    types::MappingStepCommand step(const Position3D& pos, double heading_deg);

    [[nodiscard]] bool findNextTarget(const Position3D& pos);
    [[nodiscard]] bool isEmptyCell(const Position3D& pos) const;          // traversable == confirmed Empty
    [[nodiscard]] bool pathSafe(const Position3D& start, const Position3D& target) const;
    [[nodiscard]] bool hasClearance(const Position3D& center) const;      // sphere margin vs walls
    [[nodiscard]] bool anyInBoundsUnmapped() const;                       // for completion status
    [[nodiscard]] CellKey cellOf(const Position3D& pos) const;

    [[nodiscard]] double moveStepCm() const;     // one voxel == map resolution
    [[nodiscard]] double clearanceCm() const;    // radius + gps_resolution/2
    [[nodiscard]] double stepAngleDeg() const;   // sphere-scan angular step

    // Forward chunk emission (caps by max_rotate / max_advance / max_elevate).
    [[nodiscard]] types::MovementCommand emitRotateChunk(double& remaining) const;
    [[nodiscard]] types::MovementCommand emitAdvanceChunk(double& remaining) const;
    [[nodiscard]] types::MovementCommand emitElevateChunk(double& remaining) const;

    // Inverse-push helpers: push capped chunks summing to `total` for backtracking.
    void pushChunkedRotate(double total);
    void pushChunkedAdvance(double total);
    void pushChunkedElevate(double total);

    [[nodiscard]] types::MappingStepCommand finishedStep() const;
    void startScan();

    static const std::array<std::array<double, 3>, 10> kDirections;

    Phase phase_ = Phase::Scanning;

    // Sphere-scan progress at the current position (relative to drone heading).
    double scan_xy_deg_ = 0.0;
    double scan_el_deg_ = -90.0;

    Position3D next_target_{};

    // Per-call chunking remainders. Only one is non-zero at a time.
    double pending_rotate_deg_ = 0.0;
    double pending_advance_cm_ = 0.0;
    double pending_elevate_cm_ = 0.0;

    std::vector<InverseMove> inverse_stack_;
    std::unordered_set<CellKey, CellKeyHash> visited_;
};

} // namespace drone_mapper
