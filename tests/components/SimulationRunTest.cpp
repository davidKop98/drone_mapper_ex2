// SimulationRun component tests. Per the PDF this suite also exercises the GPS and
// Movement mocks: the two-view-over-one-truth MockGPS (gps_resolution) and MockMovement's
// ground-truth sphere collision against the hidden map.
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;

std::shared_ptr<GpsTruth> makeTruth(double x, double y, double z, double h, double a) {
    auto t = std::make_shared<GpsTruth>();
    t->position = Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
    t->heading = Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
    return t;
}
Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
Orientation O(double h, double a) {
    return Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
}
double X(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
double Y(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
double Z(const Position3D& p) { return p.z.force_numerical_value_in(cm); }
double H(const Orientation& o) { return o.horizontal.force_numerical_value_in(deg); }
double A(const Orientation& o) { return o.altitude.force_numerical_value_in(deg); }

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
    l.d = 10.0 * cm;
    l.fov_circles = 1;
    return l;
}
DroneState droneStateAt(double x, double y, double z, double h) {
    DroneState s;
    s.position = P(x, y, z);
    s.heading = O(h, 0);
    s.step_index = 0;
    return s;
}
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
    ADD_FAILURE() << "no decision within iteration cap";
    return MappingStepCommand{};
}
} // namespace

// ---- MockGPS: two views over one truth ----------------------------------------

TEST(SimulationRun, GpsRoundedViewRoundsToNearest) {
    auto truth = makeTruth(17, 18, 12, 0, 0);
    MockGPS rounded(truth, 5.0 * cm);
    const Position3D p = rounded.position();
    EXPECT_DOUBLE_EQ(X(p), 15.0); // 17 -> 15
    EXPECT_DOUBLE_EQ(Y(p), 20.0); // 18 -> 20 (nearest, not floor)
    EXPECT_DOUBLE_EQ(Z(p), 10.0); // 12 -> 10

    rounded.setPosition(P(-17, 18, 12));
    EXPECT_DOUBLE_EQ(X(rounded.position()), -15.0); // -17 -> -15
}

TEST(SimulationRun, GpsExactViewReturnsRaw) {
    auto truth = makeTruth(17, 18, 12, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    const Position3D p = exact.position();
    EXPECT_DOUBLE_EQ(X(p), 17.0);
    EXPECT_DOUBLE_EQ(Y(p), 18.0);
    EXPECT_DOUBLE_EQ(Z(p), 12.0);
}

TEST(SimulationRun, GpsSharedTruthSingleUpdateBothViews) {
    auto truth = makeTruth(17, 18, 12, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 5.0 * cm);

    rounded.setPose(P(22, 23, 24), O(90, 0)); // update through ONE view

    EXPECT_DOUBLE_EQ(X(exact.position()), 22.0);
    EXPECT_DOUBLE_EQ(Y(exact.position()), 23.0);
    EXPECT_DOUBLE_EQ(Z(exact.position()), 24.0);
    EXPECT_DOUBLE_EQ(X(rounded.position()), 20.0); // 22 -> 20
    EXPECT_DOUBLE_EQ(Y(rounded.position()), 25.0); // 23 -> 25
    EXPECT_DOUBLE_EQ(Z(rounded.position()), 25.0); // 24 -> 25
}

TEST(SimulationRun, GpsHeadingUnrounded) {
    auto truth = makeTruth(0, 0, 0, 47.0, 13.0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 5.0 * cm);
    EXPECT_DOUBLE_EQ(H(exact.heading()), 47.0);
    EXPECT_DOUBLE_EQ(A(exact.heading()), 13.0);
    EXPECT_DOUBLE_EQ(H(rounded.heading()), 47.0);
    EXPECT_DOUBLE_EQ(A(rounded.heading()), 13.0);
}

// ---- MockMovement: ground-truth sphere collision ------------------------------

TEST(SimulationRun, AdvanceIntoHiddenWallCrashesAndDoesNotMove) {
    Map3DImpl hidden(makeArray(20, 5, 5, 0), mapCfg(10, 20, 5, 5)); // free space
    hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied);           // wall at cell 10
    auto truth = makeTruth(55, 25, 25, 0, 0);                       // cell 5, facing +x
    MockGPS exact(truth, 0.0 * cm);
    MockMovement mover(exact, hidden, 2.0 * cm);

    const MovementResult r = mover.advance(60.0 * cm); // sweeps through the wall at x=100
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.message, "DRONE_HITS_OBSTACLE");
    EXPECT_DOUBLE_EQ(X(exact.position()), 55.0); // truth unchanged
}

TEST(SimulationRun, AdvanceThroughClearUpdatesTruthAndBothViews) {
    Map3DImpl hidden(makeArray(20, 5, 5, 0), mapCfg(10, 20, 5, 5)); // no walls
    auto truth = makeTruth(55, 25, 25, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(exact.truth(), 10.0 * cm); // sibling view over the same truth
    MockMovement mover(exact, hidden, 2.0 * cm);

    const MovementResult r = mover.advance(20.0 * cm); // +x -> x = 75
    EXPECT_TRUE(r.success);
    EXPECT_DOUBLE_EQ(X(exact.position()), 75.0);   // exact truth committed
    EXPECT_DOUBLE_EQ(Y(exact.position()), 25.0);
    EXPECT_DOUBLE_EQ(X(rounded.position()), 80.0); // rounded view follows: 75 -> 80
}

TEST(SimulationRun, ElevateNegativeClearThenCeilingCrash) {
    Map3DImpl hidden(makeArray(5, 5, 20, 0), mapCfg(10, 5, 5, 20));
    hidden.set(P(25, 25, 155), VoxelOccupancy::Occupied); // ceiling at z cell 15
    auto truth = makeTruth(25, 25, 95, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockMovement mover(exact, hidden, 2.0 * cm);

    const MovementResult down = mover.elevate(-30.0 * cm); // z 95 -> 65, clear
    EXPECT_TRUE(down.success);
    EXPECT_DOUBLE_EQ(Z(exact.position()), 65.0);

    const MovementResult up = mover.elevate(90.0 * cm); // z 65 -> 155, into the ceiling
    EXPECT_FALSE(up.success);
    EXPECT_EQ(up.message, "DRONE_HITS_OBSTACLE");
    EXPECT_DOUBLE_EQ(Z(exact.position()), 65.0); // unchanged after crash
}

TEST(SimulationRun, RotateAlwaysSucceedsAndUpdatesHeading) {
    Map3DImpl hidden(makeArray(5, 5, 5, 1), mapCfg(10, 5, 5, 5)); // walls everywhere
    auto truth = makeTruth(25, 25, 25, 90, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockMovement mover(exact, hidden, 2.0 * cm);

    const MovementResult left = mover.rotate(RotationDirection::Left, O(30, 0).horizontal);
    EXPECT_TRUE(left.success);
    EXPECT_DOUBLE_EQ(H(exact.heading()), 120.0); // 90 + 30

    const MovementResult right = mover.rotate(RotationDirection::Right, O(50, 0).horizontal);
    EXPECT_TRUE(right.success);
    EXPECT_DOUBLE_EQ(H(exact.heading()), 70.0); // 120 - 50
}

// Parity: a move the algorithm's (radius + gps/2) clearance approves must not
// false-crash in MockMovement (radius). The hidden map's real walls sit outside the
// margin the algorithm enforced.
TEST(SimulationRun, MovementParityWithAlgorithmClearance) {
    Map3DImpl output(makeArray(7, 7, 7, 255), mapCfg(10, 7, 7, 7));
    fillCorridor(output); // algorithm will approve a +x advance into the corridor
    MappingAlgorithmImpl algo(missionCfg(8), lidarCfg(), droneCfg(2, 45, 100, 100), output);
    const MappingStepCommand cmd = driveToDecision(algo, droneStateAt(35, 35, 35, 0));
    ASSERT_TRUE(cmd.movement.has_value());
    ASSERT_EQ(cmd.movement->type, MovementCommandType::Advance);
    const double dist = cmd.movement->distance.force_numerical_value_in(cm);

    // Hidden ground truth: the same corridor is free, everything else is a wall.
    Map3DImpl hidden(makeArray(7, 7, 7, 1), mapCfg(10, 7, 7, 7));
    fillCorridor(hidden);
    auto truth = makeTruth(35, 35, 35, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockMovement mover(exact, hidden, 2.0 * cm);

    const MovementResult moved = mover.advance(dist * cm);
    EXPECT_TRUE(moved.success);            // no false crash
    EXPECT_GT(X(exact.position()), 35.0);  // advanced along +x
}

// ---- Factory: real .npy + MapConfig construction ------------------------------

namespace {
// Write a 5x5x5 hidden .npy with one Occupied voxel at index [2][3][1], return its path.
std::filesystem::path writeHiddenNpy(const char* name) {
    auto arr = makeArray(5, 5, 5, 0);                       // all Empty (0)
    arr->Data<std::uint8_t>()[2 * 5 * 5 + 3 * 5 + 1] = 1;   // voxel (2,3,1) Occupied
    const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
    EXPECT_EQ(arr->SaveNPY(path.string()), nullptr);
    return path;
}
MappingBounds boundsBox(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z) {
    return MappingBounds{
        min_x * x_extent[cm], max_x * x_extent[cm],
        min_y * y_extent[cm], max_y * y_extent[cm],
        min_z * z_extent[cm], max_z * z_extent[cm],
    };
}
} // namespace

// Hidden map: a known Occupied voxel reads Occupied through the built map; config carries
// the resolution and full-extent boundaries.
TEST(SimulationRun, FactoryLoadsHiddenMapWithResolution) {
    const std::filesystem::path path = writeHiddenNpy("dm_hidden_load.npy");
    SimulationConfigData sim;
    sim.map_filename = path;
    sim.map_resolution = 10.0 * cm;
    sim.map_offset = P(0, 0, 0);

    auto hidden = SimulationRunFactoryImpl::loadHiddenMap(sim);
    EXPECT_EQ(hidden->atVoxel(P(25, 35, 15)), VoxelOccupancy::Occupied); // voxel (2,3,1) center
    EXPECT_EQ(hidden->atVoxel(P(5, 5, 5)), VoxelOccupancy::Empty);       // voxel (0,0,0)

    const MapConfig cfg = hidden->getMapConfig();
    EXPECT_DOUBLE_EQ(cfg.resolution.force_numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_x.force_numerical_value_in(cm), 50.0); // 5 * 10
    std::filesystem::remove(path);
}

// Hidden map: offset shifts where each voxel lives in world space.
TEST(SimulationRun, FactoryHiddenMapHonorsOffset) {
    const std::filesystem::path path = writeHiddenNpy("dm_hidden_offset.npy");
    SimulationConfigData sim;
    sim.map_filename = path;
    sim.map_resolution = 10.0 * cm;
    sim.map_offset = P(-50, -50, -50); // world extent now [-50, 0) per axis

    auto hidden = SimulationRunFactoryImpl::loadHiddenMap(sim);
    EXPECT_EQ(hidden->atVoxel(P(-25, -15, -35)), VoxelOccupancy::Occupied); // voxel (2,3,1)
    EXPECT_TRUE(hidden->isInBounds(P(-25, -15, -35)));
    EXPECT_FALSE(hidden->isInBounds(P(0, 0, 0))); // outside [-50, 0)
    std::filesystem::remove(path);
}

// Output map: starts all-Unmapped over the box, isInBounds matches the box, and the
// mission bounds land on the output MapConfig with offset = bounds.min.
TEST(SimulationRun, FactoryOutputMapAllUnmappedOverBox) {
    auto output = SimulationRunFactoryImpl::makeOutputMap(boundsBox(0, 70, 0, 70, 0, 70), 10.0 * cm);

    EXPECT_EQ(output->atVoxel(P(5, 5, 5)), VoxelOccupancy::Unmapped);
    EXPECT_EQ(output->atVoxel(P(65, 65, 65)), VoxelOccupancy::Unmapped);
    EXPECT_TRUE(output->isInBounds(P(0, 0, 0)));
    EXPECT_TRUE(output->isInBounds(P(69, 69, 69)));
    EXPECT_FALSE(output->isInBounds(P(70, 0, 0)));
    EXPECT_FALSE(output->isInBounds(P(-1, 0, 0)));

    const MapConfig cfg = output->getMapConfig();
    EXPECT_DOUBLE_EQ(cfg.boundaries.min_x.force_numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(cfg.boundaries.max_x.force_numerical_value_in(cm), 70.0);
    EXPECT_DOUBLE_EQ(cfg.offset.x.force_numerical_value_in(cm), 0.0);
    EXPECT_DOUBLE_EQ(cfg.resolution.force_numerical_value_in(cm), 10.0);
}

// Output map: a non-zero-min box snaps offset to bounds.min.
TEST(SimulationRun, FactoryOutputMapOffsetIsBoundsMin) {
    auto output = SimulationRunFactoryImpl::makeOutputMap(boundsBox(-20, 30, -20, 30, 0, 40), 10.0 * cm);
    const MapConfig cfg = output->getMapConfig();
    EXPECT_DOUBLE_EQ(cfg.offset.x.force_numerical_value_in(cm), -20.0);
    EXPECT_DOUBLE_EQ(cfg.offset.z.force_numerical_value_in(cm), 0.0);
    EXPECT_TRUE(output->isInBounds(P(-20, -20, 0)));         // min corner in bounds
    EXPECT_FALSE(output->isInBounds(P(30, 0, 0)));           // max edge OOB
    EXPECT_EQ(output->atVoxel(P(-15, -15, 5)), VoxelOccupancy::Unmapped);
}

// Resolution status compares the requested resolution (gps_resolution / factor) against the
// resolution we actually use (the default = 10cm). The resolution returned is the default
// in every case; only the status changes.
TEST(SimulationRun, FactoryResolutionStatusFromExpectedResolution) {
    const PhysicalLength def = 10.0 * cm; // the resolution we actually use
    MissionConfigData m;

    // expected = 20 / 2 = 10 == used -> Accepted (even though factor != 1).
    m.gps_resolution = 20.0 * cm;
    m.output_mapping_resolution_factor = 2.0;
    const auto accepted = SimulationRunFactoryImpl::resolveOutputResolution(m, def);
    EXPECT_EQ(accepted.status, ResolutionRequestStatus::Accepted);
    EXPECT_DOUBLE_EQ(accepted.resolution.force_numerical_value_in(cm), 10.0);

    // expected = 5 / 1 = 5 < used -> IgnoredTooSmall (even though factor == 1).
    m.gps_resolution = 5.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    const auto too_small = SimulationRunFactoryImpl::resolveOutputResolution(m, def);
    EXPECT_EQ(too_small.status, ResolutionRequestStatus::IgnoredTooSmall);
    EXPECT_DOUBLE_EQ(too_small.resolution.force_numerical_value_in(cm), 10.0); // still default

    // expected = 40 / 2 = 20 > used -> Ignored (coarser than we use).
    m.gps_resolution = 40.0 * cm;
    m.output_mapping_resolution_factor = 2.0;
    const auto ignored = SimulationRunFactoryImpl::resolveOutputResolution(m, def);
    EXPECT_EQ(ignored.status, ResolutionRequestStatus::Ignored);
    EXPECT_DOUBLE_EQ(ignored.resolution.force_numerical_value_in(cm), 10.0); // still default
}

// Full graph: create() wires the whole object without throwing (additional smoke test).
TEST(SimulationRun, FactoryCreateWiresFullGraph) {
    const std::filesystem::path path = writeHiddenNpy("dm_factory_create.npy");
    SimulationConfigData sim;
    sim.map_filename = path;
    sim.map_resolution = 10.0 * cm;
    sim.map_offset = P(0, 0, 0);
    sim.initial_drone_position = P(25, 25, 25);
    sim.initial_angle = 0.0 * horizontal_angle[deg];

    MissionConfigData mission;
    mission.max_steps = 10;
    mission.gps_resolution = 8.0 * cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = boundsBox(0, 50, 0, 50, 0, 50);

    DroneConfigData drone;
    drone.radius = 5.0 * cm;
    drone.max_rotate = 45.0 * horizontal_angle[deg];
    drone.max_advance = 10.0 * cm;
    drone.max_elevate = 10.0 * cm;

    LidarConfigData lidar;
    lidar.z_min = 10.0 * cm;
    lidar.z_max = 100.0 * cm;
    lidar.d = 10.0 * cm;
    lidar.fov_circles = 1;

    SimulationRunFactoryImpl factory;
    auto run = factory.create(sim, mission, drone, lidar, std::filesystem::temp_directory_path());
    EXPECT_NE(run, nullptr);
    std::filesystem::remove(path);
}
