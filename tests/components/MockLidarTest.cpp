// The hidden map is scaffolding here (MockLidar's subject is ray marching), so it is
// a FakeMap3D: Map3DImpl mutations cannot fail this suite.
#include "FakeMap3D.h"

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using drone_mapper::test_support::FakeMap3D;

// All-Empty fake hidden map of [nx][ny][nz] cells (the old arrays were 0-filled).
FakeMap3D emptyHidden(std::size_t nx, std::size_t ny, std::size_t nz, const MapConfig& cfg) {
    return FakeMap3D(cfg, nx, ny, nz, VoxelOccupancy::Empty);
}
MapConfig mapCfg(double res) {
    MapConfig c;
    c.offset = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    c.resolution = res * cm;
    return c;
}
Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
Orientation O(double h, double a) {
    return Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
}
LidarConfigData lidarCfg(double z_min, double z_max) {
    LidarConfigData c;
    c.z_min = z_min * cm;
    c.z_max = z_max * cm;
    c.d = 1.0 * cm;
    c.fov_circles = 1; // center beam only
    return c;
}
std::shared_ptr<GpsTruth> truthAt(double x, double y, double z, double h) {
    auto t = std::make_shared<GpsTruth>();
    t->position = P(x, y, z);
    t->heading = O(h, 0);
    return t;
}
double distCm(const LidarHit& hit) { return hit.distance.force_numerical_value_in(cm); }
bool isMiss(const LidarHit& hit) {
    return distCm(hit) == std::numeric_limits<double>::max();
}
} // namespace

// (a) straight scan into a wall -> hit at the right distance.
TEST(MockLidar, StraightScanHit) {
    FakeMap3D hidden = emptyHidden(20, 5, 5, mapCfg(10.0)); // world x[0,200), res 10
    hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied); // wall cell x=10 [100,110)

    MockGPS exact(truthAt(5, 25, 25, 0), 0.0 * cm);
    MockLidar lidar(lidarCfg(1.0, 300.0), hidden, exact);

    const LidarScanResult scan = lidar.scan(O(0, 0)); // beam +x
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_FALSE(isMiss(scan[0]));
    EXPECT_NEAR(distCm(scan[0]), 95.0, 1.0); // wall face 100 - origin 5
}

// (a) open space within z_max -> miss (max double).
TEST(MockLidar, MissReturnsMaxDouble) {
    FakeMap3D hidden = emptyHidden(20, 20, 20, mapCfg(10.0));
    hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied); // wall only along +x

    MockGPS exact(truthAt(5, 25, 25, 0), 0.0 * cm);
    MockLidar lidar(lidarCfg(1.0, 100.0), hidden, exact);

    const LidarScanResult scan = lidar.scan(O(90, 0)); // beam +y, no wall
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_TRUE(isMiss(scan[0]));
}

// (a) wall closer than z_min -> too-close (distance 0).
TEST(MockLidar, TooCloseReturnsZero) {
    FakeMap3D hidden = emptyHidden(20, 5, 5, mapCfg(10.0));
    hidden.set(P(15, 25, 25), VoxelOccupancy::Occupied); // wall cell x=1 [10,20), 5cm away

    MockGPS exact(truthAt(5, 25, 25, 0), 0.0 * cm);
    MockLidar lidar(lidarCfg(20.0, 300.0), hidden, exact); // z_min 20 > 5

    const LidarScanResult scan = lidar.scan(O(0, 0));
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_DOUBLE_EQ(distCm(scan[0]), 0.0);
}

// (a) hits carry RELATIVE angles (independent of heading).
TEST(MockLidar, HitsCarryRelativeAngles) {
    FakeMap3D hidden = emptyHidden(20, 20, 20, mapCfg(10.0));

    MockGPS exact(truthAt(50, 50, 50, 30), 0.0 * cm); // nonzero heading 30deg
    LidarConfigData lc = lidarCfg(1.0, 50.0);
    lc.fov_circles = 2; // center + a ring of 4 beams
    MockLidar lidar(lc, hidden, exact);

    const LidarScanResult scan = lidar.scan(O(10, 0)); // scan dir 10deg (relative)
    ASSERT_GT(scan.size(), 1u);
    // Center beam's reported angle is the scan orientation (relative), NOT scan+heading.
    EXPECT_DOUBLE_EQ(scan[0].angle.horizontal.force_numerical_value_in(deg), 10.0);
    EXPECT_DOUBLE_EQ(scan[0].angle.altitude.force_numerical_value_in(deg), 0.0);
}

// (b) the lidar traces from the EXACT origin, not a gps-rounded one.
TEST(MockLidar, UsesExactPositionNotRounded) {
    FakeMap3D hidden = emptyHidden(40, 5, 5, mapCfg(10.0)); // world x[0,400), res 10
    hidden.set(P(305, 25, 25), VoxelOccupancy::Occupied); // wall cell x=30 [300,310)

    auto truth = truthAt(252, 25, 25, 0); // x=252 rounds to 250 at res 5
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 5.0 * cm);
    ASSERT_DOUBLE_EQ(exact.position().x.force_numerical_value_in(cm), 252.0);
    ASSERT_DOUBLE_EQ(rounded.position().x.force_numerical_value_in(cm), 250.0);

    MockLidar lidar(lidarCfg(1.0, 300.0), hidden, exact);
    const LidarScanResult scan = lidar.scan(O(0, 0));
    ASSERT_EQ(scan.size(), 1u);
    const double dist = distCm(scan[0]);
    EXPECT_NEAR(dist, 48.0, 1.0); // 300 - 252 (exact), not 50 (300 - 250 rounded)
    EXPECT_LT(dist, 49.5);        // strictly distinguishes the exact origin from rounded
}
