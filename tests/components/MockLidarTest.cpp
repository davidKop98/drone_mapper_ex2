// The hidden map and the GPS are scaffolding here (MockLidar's subject is ray
// marching), so they are FakeMap3D / FakeGps: Map3DImpl and MockGPS mutations cannot
// fail this suite.
#include "FakeGps.h"
#include "FakeMap3D.h"

#include <drone_mapper/MockLidar.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using drone_mapper::test_support::FakeGps;
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
double distCm(const LidarHit& hit) { return hit.distance.force_numerical_value_in(cm); }
bool isMiss(const LidarHit& hit) {
    return distCm(hit) == std::numeric_limits<double>::max();
}
} // namespace

// (a) straight scan into a wall -> hit at the right distance.
TEST(MockLidar, StraightScanHit) {
    FakeMap3D hidden = emptyHidden(20, 5, 5, mapCfg(10.0)); // world x[0,200), res 10
    hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied); // wall cell x=10 [100,110)

    FakeGps exact(P(5, 25, 25), O(0, 0));
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

    FakeGps exact(P(5, 25, 25), O(0, 0));
    MockLidar lidar(lidarCfg(1.0, 100.0), hidden, exact);

    const LidarScanResult scan = lidar.scan(O(90, 0)); // beam +y, no wall
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_TRUE(isMiss(scan[0]));
}

// (a) wall closer than z_min -> too-close (distance 0).
TEST(MockLidar, TooCloseReturnsZero) {
    FakeMap3D hidden = emptyHidden(20, 5, 5, mapCfg(10.0));
    hidden.set(P(15, 25, 25), VoxelOccupancy::Occupied); // wall cell x=1 [10,20), 5cm away

    FakeGps exact(P(5, 25, 25), O(0, 0));
    MockLidar lidar(lidarCfg(20.0, 300.0), hidden, exact); // z_min 20 > 5

    const LidarScanResult scan = lidar.scan(O(0, 0));
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_DOUBLE_EQ(distCm(scan[0]), 0.0);
}

// (a) hits carry RELATIVE angles (independent of heading).
TEST(MockLidar, HitsCarryRelativeAngles) {
    FakeMap3D hidden = emptyHidden(20, 20, 20, mapCfg(10.0));

    FakeGps exact(P(50, 50, 50), O(30, 0)); // nonzero heading 30deg
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

    // x=252 would round to 250 at gps res 5; the lidar must trace from the exact 252.
    FakeGps exact(P(252, 25, 25), O(0, 0));

    MockLidar lidar(lidarCfg(1.0, 300.0), hidden, exact);
    const LidarScanResult scan = lidar.scan(O(0, 0));
    ASSERT_EQ(scan.size(), 1u);
    const double dist = distCm(scan[0]);
    EXPECT_NEAR(dist, 48.0, 1.0); // 300 - 252 (exact), not 50 (300 - 250 rounded)
    EXPECT_LT(dist, 49.5);        // strictly distinguishes the exact origin from rounded
}

// ---- Survivor-killing additions (mutation campaign) --------------------------------

// The beam direction composes scan orientation WITH the sensor heading: at heading 90
// a straight scan fires along +y and must hit the +y wall at exactly 95cm. Dropping
// the heading fires it along +x into empty space.
TEST(MockLidar, ScanAtHeading90HitsWallInY) {
    FakeMap3D hidden = emptyHidden(5, 40, 5, mapCfg(10.0)); // world y[0,400)
    hidden.set(P(25, 105, 25), VoxelOccupancy::Occupied);   // wall cell y=10, face at y=100
    FakeGps exact(P(25, 5, 25), O(90, 0));                  // facing +y

    MockLidar lidar(lidarCfg(1.0, 300.0), hidden, exact);
    const LidarScanResult scan = lidar.scan(O(0, 0));
    ASSERT_EQ(scan.size(), 1u);
    ASSERT_FALSE(isMiss(scan[0]));
    EXPECT_DOUBLE_EQ(distCm(scan[0]), 95.0); // wall face 100 - origin y 5
    EXPECT_DOUBLE_EQ(scan[0].angle.horizontal.force_numerical_value_in(deg), 0.0); // relative
}

// The beam fan's STRUCTURE: fov_circles = 5 emits exactly 1+4+16+64+256 = 341 beams;
// entry 0 is the center at the scan orientation itself; circle k sits at radius k*d
// measured at z_min, so with d = 1 and z_min = 1 the theta=0 beam of circle 1 is
// offset atan2(1,1) = 45 deg and of circle 2 atan2(2,1) = atan(2) ~ 63.435 deg.
// Angles are RELATIVE (heading 30 must not leak into them).
TEST(MockLidar, BeamFanStructureAndSpread) {
    FakeMap3D hidden = emptyHidden(20, 20, 20, mapCfg(10.0));
    FakeGps exact(P(100, 100, 100), O(30, 0)); // nonzero heading: must NOT appear in angles
    LidarConfigData lc = lidarCfg(1.0, 20.0);  // d = 1, z_min = 1 (from the helper)
    lc.fov_circles = 5;
    MockLidar lidar(lc, hidden, exact);

    const LidarScanResult scan = lidar.scan(O(10, 0));
    ASSERT_EQ(scan.size(), 341u); // 1 + 4 + 16 + 64 + 256

    // Entry 0: the center beam, exactly the (relative) scan orientation.
    EXPECT_DOUBLE_EQ(scan[0].angle.horizontal.force_numerical_value_in(deg), 10.0);
    EXPECT_DOUBLE_EQ(scan[0].angle.altitude.force_numerical_value_in(deg), 0.0);
    // Entry 1: circle 1, theta = 0 -> horizontal offset atan2(1*d, z_min) = 45 deg.
    EXPECT_DOUBLE_EQ(scan[1].angle.horizontal.force_numerical_value_in(deg),
                     10.0 + std::atan2(1.0, 1.0) * 180.0 / M_PI);
    EXPECT_DOUBLE_EQ(scan[1].angle.altitude.force_numerical_value_in(deg), 0.0);
    // Entry 5: circle 2, theta = 0 -> radius 2*d -> atan2(2, 1) ~ 63.435 deg.
    EXPECT_DOUBLE_EQ(scan[5].angle.horizontal.force_numerical_value_in(deg),
                     10.0 + std::atan2(2.0, 1.0) * 180.0 / M_PI);
}

// Range boundaries hit EXACTLY: a wall whose first in-wall sample lands precisely at
// z_max is still in range (<=), and one landing precisely at z_min is NOT too-close
// (the too-close rule is strict <) -- both report the true 95cm distance.
TEST(MockLidar, HitExactlyAtRangeBoundaries) {
    { // hit distance == z_max: still a valid hit
        FakeMap3D hidden = emptyHidden(20, 5, 5, mapCfg(10.0));
        hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied); // face at x=100
        FakeGps exact(P(5, 25, 25), O(0, 0));
        MockLidar lidar(lidarCfg(1.0, /*z_max*/ 95.0), hidden, exact);
        const LidarScanResult scan = lidar.scan(O(0, 0));
        ASSERT_EQ(scan.size(), 1u);
        ASSERT_FALSE(isMiss(scan[0]));
        EXPECT_DOUBLE_EQ(distCm(scan[0]), 95.0);
    }
    { // hit distance == z_min: NOT too-close, true distance (not 0)
        FakeMap3D hidden = emptyHidden(20, 5, 5, mapCfg(10.0));
        hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied);
        FakeGps exact(P(5, 25, 25), O(0, 0));
        MockLidar lidar(lidarCfg(/*z_min*/ 95.0, 300.0), hidden, exact);
        const LidarScanResult scan = lidar.scan(O(0, 0));
        ASSERT_EQ(scan.size(), 1u);
        EXPECT_DOUBLE_EQ(distCm(scan[0]), 95.0);
    }
}

// An elevated beam scales the horizontal components by cos(altitude): at 45 deg the
// direction is (0.7071, 0, 0.7071), so the ceiling cell (5,2,3) is first sampled at
// exactly d = 36 (x = 50.46 >= 50 and z = 30.46 >= 30; at d = 35 x is still 49.75).
// Dropping the cos(altitude) factor sends the beam past the cell (x overshoots).
TEST(MockLidar, AltitudeBeamHitsCeilingAtExactDistance) {
    FakeMap3D hidden = emptyHidden(8, 5, 5, mapCfg(10.0)); // x[0,80) z[0,50)
    hidden.set(P(55, 25, 35), VoxelOccupancy::Occupied);   // ceiling cell (5,2,3)
    FakeGps exact(P(25, 25, 5), O(0, 0));

    MockLidar lidar(lidarCfg(1.0, 300.0), hidden, exact);
    const LidarScanResult scan = lidar.scan(O(0, 45)); // 45 deg up, along +x
    ASSERT_EQ(scan.size(), 1u);
    ASSERT_FALSE(isMiss(scan[0]));
    EXPECT_DOUBLE_EQ(distCm(scan[0]), 36.0);
}
