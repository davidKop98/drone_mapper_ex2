// MappingAlgorithmImpl drives one MappingStepCommand per call against a seeded
// FakeMap3D output map (we don't run the converter here, so the map stays as seeded;
// the map is scaffolding, so Map3DImpl mutations cannot fail this suite). Lidar config uses d == z_min so the sphere sweep step caps at 5 deg.
//
// Clearance footprint = radius + gps_resolution/2. With radius 2cm and gps 8cm that is
// 6cm > half a 10cm cell, so a planned center's footprint spills into the 3x3x3 block
// of cells around it; every one of those cells must be confirmed Empty. The "corridor"
// helper seeds exactly the Empty cells that let the drone advance along +x but not
// branch into the surrounding unmapped space.
#include "FakeMap3D.h"

#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using drone_mapper::test_support::FakeMap3D;


MapConfig mapCfg(double res, std::size_t nx, std::size_t ny, std::size_t nz) {
    MapConfig c;
    c.offset = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    c.resolution = res * cm;
    c.boundaries = MappingBounds{
        0.0 * x_extent[cm], static_cast<double>(nx) * res * x_extent[cm],
        0.0 * y_extent[cm], static_cast<double>(ny) * res * y_extent[cm],
        0.0 * z_extent[cm], static_cast<double>(nz) * res * z_extent[cm],
    };
    return c;
}

DroneConfigData droneCfg(double radius, double max_rot, double max_adv, double max_ele) {
    DroneConfigData d;
    d.radius = radius * cm;
    d.max_rotate = max_rot * horizontal_angle[deg];
    d.max_advance = max_adv * cm;
    d.max_elevate = max_ele * cm;
    return d;
}

MissionConfigData missionCfg(double gps_res) {
    MissionConfigData m;
    m.max_steps = 1000000;
    m.gps_resolution = gps_res * cm;
    m.output_mapping_resolution_factor = 1;
    return m;
}

LidarConfigData lidarCfg() {
    LidarConfigData l;
    l.z_min = 10.0 * cm;
    l.z_max = 100.0 * cm;
    l.d = 10.0 * cm; // d >= z_min -> sweep step caps at 5 deg (bounded sweep)
    l.fov_circles = 1;
    return l;
}

Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

DroneState droneState(double x, double y, double z, double heading_deg) {
    DroneState s;
    s.position = P(x, y, z);
    s.heading = Orientation{heading_deg * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    s.step_index = 0;
    return s;
}

// Empty corridor along +x in a 7^3 grid (10cm cells), wide enough for the 6cm footprint
// to advance along x but not branch: x cells 2..6, y cells 2..4, z cells 2..4 are Empty;
// everything else stays Unmapped. The drone starts at cell (3,3,3) == (35,35,35).
void fillCorridor(IMutableMap3D& m) {
    for (int xi = 2; xi <= 6; ++xi) {
        for (int yi = 2; yi <= 4; ++yi) {
            for (int zi = 2; zi <= 4; ++zi) {
                m.set(P(xi * 10.0 + 5.0, yi * 10.0 + 5.0, zi * 10.0 + 5.0), VoxelOccupancy::Empty);
            }
        }
    }
}

MappingStepCommand driveToDecision(IMappingAlgorithm& algo, const DroneState& st) {
    for (int i = 0; i < 500000; ++i) {
        const MappingStepCommand c = algo.nextStep(st, nullptr);
        if (c.movement.has_value() || c.status != AlgorithmStatus::Working) return c;
    }
    ADD_FAILURE() << "no decision reached within iteration cap";
    return MappingStepCommand{};
}

AlgorithmStatus driveToTerminal(IMappingAlgorithm& algo, const DroneState& st) {
    for (int i = 0; i < 1000000; ++i) {
        const MappingStepCommand c = algo.nextStep(st, nullptr);
        if (c.status != AlgorithmStatus::Working) return c.status;
    }
    ADD_FAILURE() << "did not terminate within iteration cap";
    return AlgorithmStatus::Working;
}
} // namespace

// (1) Bootstrap: null scan + all-Unmapped -> a scan, no movement, Working.
TEST(MappingAlgorithm, BootstrapScanNoMovement) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    const MappingStepCommand cmd = algo.nextStep(droneState(35, 35, 35, 0), nullptr);
    EXPECT_FALSE(cmd.movement.has_value());
    EXPECT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_EQ(cmd.status, AlgorithmStatus::Working);
}

// (2) Frontier pursuit: the drone advances toward the frontier only because the forward
// footprint is confirmed Empty (the corridor). It scans first, then advances.
TEST(MappingAlgorithm, FrontierPursuitAdvancesIntoConfirmedEmpty) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out); // +x footprint confirmed Empty; unmapped beyond -> frontier
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
    const DroneState st = droneState(35, 35, 35, 0); // facing +x

    const MappingStepCommand first = algo.nextStep(st, nullptr);
    EXPECT_FALSE(first.movement.has_value());
    EXPECT_TRUE(first.scan_orientation.has_value()); // scans from standoff first

    const MappingStepCommand decision = driveToDecision(algo, st);
    ASSERT_TRUE(decision.movement.has_value());
    EXPECT_EQ(decision.movement->type, MovementCommandType::Advance); // into confirmed-Empty space
    EXPECT_GT(decision.movement->distance.force_numerical_value_in(cm), 0.0);
    EXPECT_EQ(decision.status, AlgorithmStatus::Working);
}

// (2-invariant) The algorithm never plans a move whose radius+gps/2 footprint overlaps
// an Unmapped cell. Here only the drone cell and +x are Empty, so the +x target's
// footprint reaches into surrounding unmapped space -> the drone scans, never advances.
TEST(MappingAlgorithm, FootprintNeverOverlapsUnmapped) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    out.set(P(35, 35, 35), VoxelOccupancy::Empty);
    out.set(P(45, 35, 35), VoxelOccupancy::Empty); // Empty, but its footprint touches unmapped
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
    const DroneState st = droneState(35, 35, 35, 0);

    const MappingStepCommand first = algo.nextStep(st, nullptr);
    EXPECT_TRUE(first.scan_orientation.has_value());
    EXPECT_FALSE(first.movement.has_value());

    const MappingStepCommand decision = driveToDecision(algo, st);
    EXPECT_FALSE(decision.movement.has_value()); // footprint would overlap Unmapped -> no move
}

// (3) Non-traversable neighbors are never entered (drone at an edge so -x is OOB).
TEST(MappingAlgorithm, NonTraversableNeighborsNeverEntered) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    out.set(P(5, 35, 35), VoxelOccupancy::Empty);              // drone cell (0,3,3): -x is OOB
    out.set(P(15, 35, 35), VoxelOccupancy::Occupied);          // +x wall
    out.set(P(5, 45, 35), VoxelOccupancy::PotentiallyOccupied); // +y uncertain wall
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    const MappingStepCommand decision = driveToDecision(algo, droneState(5, 35, 35, 0));
    EXPECT_FALSE(decision.movement.has_value());
    EXPECT_NE(decision.status, AlgorithmStatus::Working);
}

// (3) PotentiallyOccupied is explicitly treated as a wall: in the same corridor an Empty
// +x cell is entered, but a PotentiallyOccupied +x cell is not.
TEST(MappingAlgorithm, PotentiallyOccupiedTreatedAsWall) {
    {
        FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
        fillCorridor(out); // +x cell (4,3,3) Empty
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        ASSERT_TRUE(d.movement.has_value());
        EXPECT_EQ(d.movement->type, MovementCommandType::Advance); // control: Empty entered
    }
    {
        FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
        fillCorridor(out);
        out.set(P(45, 35, 35), VoxelOccupancy::PotentiallyOccupied); // +x target cell now PO
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        EXPECT_FALSE(d.movement.has_value()); // PotentiallyOccupied blocks like a wall
    }
}

// (4) Chunking: a frontier one cell (10cm) away with max_advance 3cm -> capped Advance.
TEST(MappingAlgorithm, ChunkingAdvanceRespectsCap) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out);
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, /*max_adv*/ 3, 100), out);

    const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0)); // faces +x
    ASSERT_TRUE(d.movement.has_value());
    EXPECT_EQ(d.movement->type, MovementCommandType::Advance);
    EXPECT_GT(d.movement->distance.force_numerical_value_in(cm), 0.0);
    EXPECT_LE(d.movement->distance.force_numerical_value_in(cm), 3.0 + 1e-6);
}

// (4) Chunking: a frontier needing ~180deg of rotation with max_rotate 20deg -> capped Rotate.
TEST(MappingAlgorithm, ChunkingRotateRespectsCap) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out); // +x is the only valid target; drone faces -x so it must rotate first
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, /*max_rot*/ 20, 100, 100), out);

    const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 180));
    ASSERT_TRUE(d.movement.has_value());
    EXPECT_EQ(d.movement->type, MovementCommandType::Rotate);
    EXPECT_GT(d.movement->angle.force_numerical_value_in(deg), 0.0);
    EXPECT_LE(d.movement->angle.force_numerical_value_in(deg), 20.0 + 1e-6);
}

// (5) Occupied within the footprint blocks the move even inside an otherwise-Empty
// corridor: control advances; an Occupied cell in the +x footprint -> avoided.
TEST(MappingAlgorithm, OccupiedInFootprintAvoidsUnsafeMove) {
    {
        FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
        fillCorridor(out);
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        ASSERT_TRUE(d.movement.has_value());
        EXPECT_EQ(d.movement->type, MovementCommandType::Advance); // safe -> taken
    }
    {
        FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
        fillCorridor(out);
        out.set(P(55, 35, 35), VoxelOccupancy::Occupied); // cell (5,3,3): inside the +x footprint
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        EXPECT_FALSE(d.movement.has_value()); // wall within clearance -> avoided
    }
}

// (6) Determinism: identical (map, state) twice -> identical command.
TEST(MappingAlgorithm, DeterministicForSameInputs) {
    FakeMap3D out1(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out1);
    FakeMap3D out2(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out2);

    MappingAlgorithmImpl a1(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out1);
    MappingAlgorithmImpl a2(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out2);
    const DroneState st = droneState(35, 35, 35, 0);

    const MappingStepCommand d1 = driveToDecision(a1, st);
    const MappingStepCommand d2 = driveToDecision(a2, st);
    ASSERT_TRUE(d1.movement.has_value());
    ASSERT_TRUE(d2.movement.has_value());
    EXPECT_EQ(d1.movement->type, d2.movement->type);
    EXPECT_EQ(d1.movement->rotation, d2.movement->rotation);
    EXPECT_DOUBLE_EQ(d1.movement->angle.force_numerical_value_in(deg),
                     d2.movement->angle.force_numerical_value_in(deg));
    EXPECT_DOUBLE_EQ(d1.movement->distance.force_numerical_value_in(cm),
                     d2.movement->distance.force_numerical_value_in(cm));
    EXPECT_EQ(d1.status, d2.status);
}

// (7) Completion Finished: a fully-mapped pocket (no Unmapped anywhere) -> Finished.
TEST(MappingAlgorithm, CompletionFinishedWhenFullyMapped) {
    FakeMap3D out(mapCfg(10, 5, 5, 5), 5, 5, 5, VoxelOccupancy::Occupied); // all Occupied
    out.set(P(25, 25, 25), VoxelOccupancy::Empty);             // single Empty cell, no Unmapped
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    EXPECT_EQ(driveToTerminal(algo, droneState(25, 25, 25, 0)), AlgorithmStatus::Finished);
}

// (8) Completion FinishedWithUnmappableVoxels: the drone won't push its footprint into
// unmapped space, so an unmapped region it can't safely enter is left unmapped.
TEST(MappingAlgorithm, CompletionUnmappableWhenFootprintWouldEnterUnmapped) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7); // mostly Unmapped
    out.set(P(35, 35, 35), VoxelOccupancy::Empty);               // drone in a 1-cell Empty pocket
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    EXPECT_EQ(driveToTerminal(algo, droneState(35, 35, 35, 0)),
              AlgorithmStatus::FinishedWithUnmappableVoxels);
}

// ---- Pose-tracking harness -----------------------------------------------------------
// The tests above feed a frozen DroneState, so they can't see whether exploration
// continues past a dead-end or whether backtracking physically replays inverse moves.
// This harness applies every emitted MovementCommand from the current pose with
// MockMovement's semantics (Left rotation increases heading; advance moves along
// cos/sin of the heading; elevate is signed) and feeds the updated state back, so the
// believed track is exactly what the emitted command sequence implies.
namespace {
constexpr double kTestPi = 3.14159265358979323846;

using Cell = std::array<long long, 3>;

Cell cellAt(double x, double y, double z, double res = 10.0) {
    return {static_cast<long long>(std::floor(x / res + 1e-9)),
            static_cast<long long>(std::floor(y / res + 1e-9)),
            static_cast<long long>(std::floor(z / res + 1e-9))};
}

struct PoseRun {
    AlgorithmStatus final_status = AlgorithmStatus::Working;
    bool completed = false;                       // reached a terminal status within the cap
    std::vector<Cell> entered_cells;              // cell appended every time it changes
    std::vector<std::vector<std::array<double, 2>>> sweeps; // {xy_deg, el_deg} between moves
    double end_x = 0.0, end_y = 0.0, end_z = 0.0;
};

PoseRun runWithPose(IMappingAlgorithm& algo, double x, double y, double z, double heading_deg) {
    PoseRun run;
    std::vector<std::array<double, 2>> sweep;
    run.entered_cells.push_back(cellAt(x, y, z));
    for (int i = 0; i < 200000; ++i) {
        const MappingStepCommand cmd = algo.nextStep(droneState(x, y, z, heading_deg), nullptr);
        if (cmd.status != AlgorithmStatus::Working) {
            run.final_status = cmd.status;
            run.completed = true;
            break;
        }
        if (cmd.scan_orientation.has_value()) {
            sweep.push_back({cmd.scan_orientation->horizontal.force_numerical_value_in(deg),
                             cmd.scan_orientation->altitude.force_numerical_value_in(deg)});
            continue;
        }
        if (!cmd.movement.has_value()) continue;
        if (!sweep.empty()) {
            run.sweeps.push_back(std::move(sweep));
            sweep.clear();
        }
        switch (cmd.movement->type) {
        case MovementCommandType::Rotate: {
            const double a = cmd.movement->angle.force_numerical_value_in(deg);
            heading_deg += (cmd.movement->rotation == RotationDirection::Left) ? a : -a;
            break;
        }
        case MovementCommandType::Advance: {
            const double d = cmd.movement->distance.force_numerical_value_in(cm);
            const double h = heading_deg * kTestPi / 180.0;
            x += d * std::cos(h);
            y += d * std::sin(h);
            break;
        }
        case MovementCommandType::Elevate:
            z += cmd.movement->distance.force_numerical_value_in(cm);
            break;
        default:
            break; // unknown command applies nothing
        }
        const Cell cell = cellAt(x, y, z);
        if (cell != run.entered_cells.back()) run.entered_cells.push_back(cell);
    }
    if (!sweep.empty()) run.sweeps.push_back(std::move(sweep));
    run.end_x = x;
    run.end_y = y;
    run.end_z = z;
    return run;
}

// Closed T-shaped topology: every cell is Occupied except two carved Empty arms, so
// there is no Unmapped anywhere and a complete exploration must end plain Finished.
//  - east arm : cells x 1..7, y 3..5, z 2..4 -> movable centerline (2..6, 4, 3)
//  - south arm: cells x 3..5, y 5..9, z 2..4 -> movable centerline (4, 5..8, 3)
// From the start (2,4,3) the direction order tries east before south, so the drone
// runs the east arm to its dead-end at (6,4,3) first and can only reach the south arm
// by backtracking out of it.
void fillTee(IMutableMap3D& m) {
    for (int xi = 1; xi <= 7; ++xi)
        for (int yi = 3; yi <= 5; ++yi)
            for (int zi = 2; zi <= 4; ++zi)
                m.set(P(xi * 10.0 + 5.0, yi * 10.0 + 5.0, zi * 10.0 + 5.0), VoxelOccupancy::Empty);
    for (int xi = 3; xi <= 5; ++xi)
        for (int yi = 5; yi <= 9; ++yi)
            for (int zi = 2; zi <= 4; ++zi)
                m.set(P(xi * 10.0 + 5.0, yi * 10.0 + 5.0, zi * 10.0 + 5.0), VoxelOccupancy::Empty);
}

long long timesEntered(const PoseRun& run, long long cx, long long cy, long long cz) {
    return std::count(run.entered_cells.begin(), run.entered_cells.end(), Cell{cx, cy, cz});
}
} // namespace

// (9) Exploration completeness: hitting the east dead-end must not end the run; the
// drone backtracks and maps the south branch too, then reports Finished.
TEST(MappingAlgorithm, BacktracksOutOfDeadEndAndMapsSecondBranch) {
    FakeMap3D out(mapCfg(10, 9, 11, 7), 9, 11, 7, VoxelOccupancy::Occupied); // all Occupied
    fillTee(out);
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    const PoseRun run = runWithPose(algo, 25, 45, 35, 0); // start cell (2,4,3), facing east
    ASSERT_TRUE(run.completed) << "algorithm did not reach a terminal status";
    EXPECT_EQ(run.final_status, AlgorithmStatus::Finished); // closed map: nothing unmappable

    // East arm explored to its dead-end...
    EXPECT_GE(timesEntered(run, 5, 4, 3), 1);
    EXPECT_GE(timesEntered(run, 6, 4, 3), 1);
    // ...and the south branch still reached afterwards (requires backtracking).
    EXPECT_GE(timesEntered(run, 4, 6, 3), 1);
    EXPECT_GE(timesEntered(run, 4, 7, 3), 1);
    EXPECT_GE(timesEntered(run, 4, 8, 3), 1);
}

// (9) Positional continuity: backtracking must physically replay inverse moves from the
// current position -- the dead-end arm is re-traversed on the way out (no teleporting),
// every implied position stays inside carved-Empty space, and the full DFS unwind
// returns the believed position to the start.
TEST(MappingAlgorithm, BacktrackReplaysInverseMovesFromCurrentPosition) {
    FakeMap3D out(mapCfg(10, 9, 11, 7), 9, 11, 7, VoxelOccupancy::Occupied);
    fillTee(out);
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    const PoseRun run = runWithPose(algo, 25, 45, 35, 0);
    ASSERT_TRUE(run.completed) << "algorithm did not reach a terminal status";

    for (const Cell& c : run.entered_cells) {
        EXPECT_EQ(out.atVoxel(P(c[0] * 10.0 + 5.0, c[1] * 10.0 + 5.0, c[2] * 10.0 + 5.0)),
                  VoxelOccupancy::Empty)
            << "implied position left Empty space at cell (" << c[0] << "," << c[1] << "," << c[2] << ")";
    }
    EXPECT_GE(timesEntered(run, 5, 4, 3), 2); // corridor cell re-entered while unwinding
    EXPECT_EQ(cellAt(run.end_x, run.end_y, run.end_z), (Cell{2, 4, 3})); // unwind ends at start
}

// (10) Sweep coverage: every sphere sweep -- including rescans started after a move --
// must span the full elevation range (straight down to straight up) and 360 deg in xy.
// Lidar d == z_min caps the step at 20 deg here, so a full sweep is xy 0..340, el -90..90.
TEST(MappingAlgorithm, SweepCoversFullElevationRange) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out);
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    const PoseRun run = runWithPose(algo, 35, 35, 35, 0);
    ASSERT_TRUE(run.completed) << "algorithm did not reach a terminal status";
    ASSERT_GE(run.sweeps.size(), 2u) << "expected a rescan after moving";

    for (std::size_t i = 0; i < run.sweeps.size(); ++i) {
        double min_xy = 1e9, max_xy = -1e9, min_el = 1e9, max_el = -1e9;
        for (const auto& beam : run.sweeps[i]) {
            min_xy = std::min(min_xy, beam[0]);
            max_xy = std::max(max_xy, beam[0]);
            min_el = std::min(min_el, beam[1]);
            max_el = std::max(max_el, beam[1]);
        }
        EXPECT_LE(min_el, -89.5) << "sweep " << i << " misses the downward hemisphere";
        EXPECT_GE(max_el, 89.5) << "sweep " << i << " misses the upward hemisphere";
        EXPECT_LE(min_xy, 0.5) << "sweep " << i << " misses the start of the xy circle";
        EXPECT_GE(max_xy, 339.5) << "sweep " << i << " does not span 360 deg in xy";
    }
}

// (4) Chunking, both halves: every advance chunk respects max_advance AND the chunks
// accumulate to the full requested distance (one 10cm cell -> 3+3+3+1 with cap 3).
TEST(MappingAlgorithm, ChunkedAdvancesSumToRequestedDistance) {
    FakeMap3D out(mapCfg(10, 7, 7, 7), 7, 7, 7);
    fillCorridor(out);
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, /*max_adv*/ 3, 100), out);
    const DroneState st = droneState(35, 35, 35, 0); // target one 10cm cell east, no rotation

    MappingStepCommand cmd = driveToDecision(algo, st);
    double total = 0.0;
    int chunks = 0;
    while (cmd.movement.has_value()) {
        ASSERT_EQ(cmd.movement->type, MovementCommandType::Advance);
        const double d = cmd.movement->distance.force_numerical_value_in(cm);
        EXPECT_GT(d, 0.0);
        EXPECT_LE(d, 3.0 + 1e-6); // per-command cap
        total += d;
        ASSERT_LE(++chunks, 100) << "chunk stream did not terminate";
        cmd = algo.nextStep(st, nullptr);
    }
    EXPECT_TRUE(cmd.scan_orientation.has_value()); // move complete -> rescan begins
    EXPECT_NEAR(total, 10.0, 1e-6);                // chunks sum to the full cell step
    EXPECT_EQ(chunks, 4);
}
