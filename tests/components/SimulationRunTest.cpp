// SimulationRun component tests. Per the PDF this suite also exercises the GPS and
// Movement mocks: the two-view-over-one-truth MockGPS (gps_resolution) and MockMovement's
// ground-truth sphere collision against the hidden map.
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
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
