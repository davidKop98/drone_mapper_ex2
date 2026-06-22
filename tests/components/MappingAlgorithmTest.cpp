// MappingAlgorithmImpl drives one MappingStepCommand per call against a seeded,
// static Map3DImpl output map (we don't run the converter here, so the map stays as
// seeded). Lidar config uses d == z_min so the sphere sweep step caps at 5 deg.
//
// Clearance footprint = radius + gps_resolution/2. With radius 2cm and gps 8cm that is
// 6cm > half a 10cm cell, so a planned center's footprint spills into the 3x3x3 block
// of cells around it; every one of those cells must be confirmed Empty. The "corridor"
// helper seeds exactly the Empty cells that let the drone advance along +x but not
// branch into the surrounding unmapped space.
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;

std::shared_ptr<NpyArray> makeArray(std::size_t nx, std::size_t ny, std::size_t nz, std::uint8_t fill) {
    auto arr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz}, sizeof(std::uint8_t), 'u', false);
    arr->Allocate();
    std::fill(arr->Data<std::uint8_t>(), arr->Data<std::uint8_t>() + arr->NumValue(), fill);
    return arr;
}

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
void fillCorridor(Map3DImpl& m) {
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
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    const MappingStepCommand cmd = algo.nextStep(droneState(35, 35, 35, 0), nullptr);
    EXPECT_FALSE(cmd.movement.has_value());
    EXPECT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_EQ(cmd.status, AlgorithmStatus::Working);
}

// (2) Frontier pursuit: the drone advances toward the frontier only because the forward
// footprint is confirmed Empty (the corridor). It scans first, then advances.
TEST(MappingAlgorithm, FrontierPursuitAdvancesIntoConfirmedEmpty) {
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
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
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
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
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
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
        Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
        fillCorridor(out); // +x cell (4,3,3) Empty
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        ASSERT_TRUE(d.movement.has_value());
        EXPECT_EQ(d.movement->type, MovementCommandType::Advance); // control: Empty entered
    }
    {
        Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
        fillCorridor(out);
        out.set(P(45, 35, 35), VoxelOccupancy::PotentiallyOccupied); // +x target cell now PO
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        EXPECT_FALSE(d.movement.has_value()); // PotentiallyOccupied blocks like a wall
    }
}

// (4) Chunking: a frontier one cell (10cm) away with max_advance 3cm -> capped Advance.
TEST(MappingAlgorithm, ChunkingAdvanceRespectsCap) {
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
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
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
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
        Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
        fillCorridor(out);
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        ASSERT_TRUE(d.movement.has_value());
        EXPECT_EQ(d.movement->type, MovementCommandType::Advance); // safe -> taken
    }
    {
        Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
        fillCorridor(out);
        out.set(P(55, 35, 35), VoxelOccupancy::Occupied); // cell (5,3,3): inside the +x footprint
        MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);
        const MappingStepCommand d = driveToDecision(algo, droneState(35, 35, 35, 0));
        EXPECT_FALSE(d.movement.has_value()); // wall within clearance -> avoided
    }
}

// (6) Determinism: identical (map, state) twice -> identical command.
TEST(MappingAlgorithm, DeterministicForSameInputs) {
    Map3DImpl out1(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
    fillCorridor(out1);
    Map3DImpl out2(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
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
    Map3DImpl out(makeArray(5, 5, 5, 1), mapCfg(10, 5, 5, 5)); // all Occupied
    out.set(P(25, 25, 25), VoxelOccupancy::Empty);             // single Empty cell, no Unmapped
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    EXPECT_EQ(driveToTerminal(algo, droneState(25, 25, 25, 0)), AlgorithmStatus::Finished);
}

// (8) Completion FinishedWithUnmappableVoxels: the drone won't push its footprint into
// unmapped space, so an unmapped region it can't safely enter is left unmapped.
TEST(MappingAlgorithm, CompletionUnmappableWhenFootprintWouldEnterUnmapped) {
    Map3DImpl out(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7)); // mostly Unmapped
    out.set(P(35, 35, 35), VoxelOccupancy::Empty);               // drone in a 1-cell Empty pocket
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), out);

    EXPECT_EQ(driveToTerminal(algo, droneState(35, 35, 35, 0)),
              AlgorithmStatus::FinishedWithUnmappableVoxels);
}
