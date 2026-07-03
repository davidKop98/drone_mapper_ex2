// Verifies the course-PROVIDED ScanResultToVoxels::applyToMap against a real
// Map3DImpl output map. We do not reimplement or modify the converter, and we do
// not involve DroneControl (that is Step 4).
#include <drone_mapper/Map3DImpl.h>
#include "FakeGps.h"
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/ScanResultToVoxels.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;

// A fresh all-Unmapped output map: shape [nx][ny][nz], offset 0, given resolution.
std::shared_ptr<NpyArray> unmappedArray(std::size_t nx, std::size_t ny, std::size_t nz) {
    auto arr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz}, sizeof(std::uint8_t), 'u', false);
    arr->Allocate();
    std::fill(arr->Data<std::uint8_t>(), arr->Data<std::uint8_t>() + arr->NumValue(),
              static_cast<std::uint8_t>(255));
    return arr;
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
    c.fov_circles = 1;
    return c;
}
// Hand-build a single-beam scan (distance + RELATIVE angle).
LidarScanResult oneBeam(PhysicalLength distance, Orientation relative_angle) {
    return LidarScanResult{LidarHit{distance, relative_angle}};
}
PhysicalLength missDistance() {
    return std::numeric_limits<double>::max() * cm;
}
} // namespace

// (a) Normal hit: Empty along the ray, Occupied at the endpoint cell.
TEST(ScanResultToVoxels, NormalHitMarksRayEmptyAndEndpointOccupied) {
    Map3DImpl out(unmappedArray(20, 5, 5), mapCfg(10.0)); // world x[0,200)
    const Position3D origin = P(5, 25, 25);
    // Beam +x, hit 95cm away -> endpoint (100,25,25) = cell (10,2,2).
    ScanResultToVoxels::applyToMap(out, origin, O(0, 0), oneBeam(95.0 * cm, O(0, 0)), lidarCfg(1, 300));

    EXPECT_EQ(out.atVoxel(P(5, 25, 25)), VoxelOccupancy::Empty);   // origin cell, along ray
    EXPECT_EQ(out.atVoxel(P(35, 25, 25)), VoxelOccupancy::Empty);  // cell 3
    EXPECT_EQ(out.atVoxel(P(95, 25, 25)), VoxelOccupancy::Empty);  // cell 9
    EXPECT_EQ(out.atVoxel(P(105, 25, 25)), VoxelOccupancy::Occupied); // cell 10 = endpoint
    EXPECT_EQ(out.atVoxel(P(155, 25, 25)), VoxelOccupancy::Unmapped); // beyond endpoint
}

// (b) Miss: Empty out to z_max (sentinel clamped), nothing Occupied.
TEST(ScanResultToVoxels, MissMarksEmptyToZMaxNothingOccupied) {
    Map3DImpl out(unmappedArray(20, 5, 5), mapCfg(10.0));
    const Position3D origin = P(5, 25, 25);
    // z_max = 80 -> Empty along +x out to x = 5 + 80 = 85 (cell 8).
    ScanResultToVoxels::applyToMap(out, origin, O(0, 0), oneBeam(missDistance(), O(0, 0)), lidarCfg(1, 80));

    EXPECT_EQ(out.atVoxel(P(35, 25, 25)), VoxelOccupancy::Empty);  // cell 3
    EXPECT_EQ(out.atVoxel(P(85, 25, 25)), VoxelOccupancy::Empty);  // cell 8 (last, at z_max)
    EXPECT_EQ(out.atVoxel(P(95, 25, 25)), VoxelOccupancy::Unmapped); // cell 9, beyond z_max
    for (int xi = 0; xi < 20; ++xi) {
        EXPECT_NE(out.atVoxel(P(xi * 10.0 + 5.0, 25, 25)), VoxelOccupancy::Occupied)
            << "no voxel should be Occupied on a miss (cell " << xi << ")";
    }
}

// (c) Too-close (dist 0): PotentiallyOccupied over [0, z_min] near the origin.
TEST(ScanResultToVoxels, TooCloseMarksPotentiallyOccupiedToZMin) {
    Map3DImpl out(unmappedArray(20, 5, 5), mapCfg(10.0));
    const Position3D origin = P(5, 25, 25);
    // z_min = 30 -> PotentiallyOccupied along +x out to x = 5 + 30 = 35 (cell 3).
    ScanResultToVoxels::applyToMap(out, origin, O(0, 0), oneBeam(0.0 * cm, O(0, 0)), lidarCfg(30, 300));

    EXPECT_EQ(out.atVoxel(P(5, 25, 25)), VoxelOccupancy::PotentiallyOccupied);  // origin cell
    EXPECT_EQ(out.atVoxel(P(35, 25, 25)), VoxelOccupancy::PotentiallyOccupied); // cell 3 (at z_min)
    EXPECT_EQ(out.atVoxel(P(45, 25, 25)), VoxelOccupancy::Unmapped);            // cell 4, beyond z_min
    for (int xi = 0; xi < 20; ++xi) {
        EXPECT_NE(out.atVoxel(P(xi * 10.0 + 5.0, 25, 25)), VoxelOccupancy::Occupied);
    }
}

// (d) Stickiness: an Empty ray cannot erase an Occupied wall; Empty does fill Unmapped.
TEST(ScanResultToVoxels, StickinessOccupiedSurvivesEmptyRay) {
    Map3DImpl out(unmappedArray(20, 5, 5), mapCfg(10.0));
    out.set(P(55, 25, 25), VoxelOccupancy::Occupied); // wall at cell 5, on the ray's path

    ScanResultToVoxels::applyToMap(out, P(5, 25, 25), O(0, 0), oneBeam(95.0 * cm, O(0, 0)), lidarCfg(1, 300));

    EXPECT_EQ(out.atVoxel(P(55, 25, 25)), VoxelOccupancy::Occupied); // Occupied > Empty -> survives
    EXPECT_EQ(out.atVoxel(P(35, 25, 25)), VoxelOccupancy::Empty);    // Empty overwrites Unmapped
    EXPECT_EQ(out.atVoxel(P(105, 25, 25)), VoxelOccupancy::Occupied); // endpoint still marked
}

// (d) Stickiness: a later PotentiallyOccupied cannot overwrite an Occupied wall.
TEST(ScanResultToVoxels, PotentiallyOccupiedDoesNotOverwriteOccupied) {
    Map3DImpl out(unmappedArray(20, 5, 5), mapCfg(10.0));
    out.set(P(15, 25, 25), VoxelOccupancy::Occupied); // wall at cell 1, within z_min reach

    // too-close scan paints PotentiallyOccupied over cells 0..3
    ScanResultToVoxels::applyToMap(out, P(5, 25, 25), O(0, 0), oneBeam(0.0 * cm, O(0, 0)), lidarCfg(30, 300));

    EXPECT_EQ(out.atVoxel(P(15, 25, 25)), VoxelOccupancy::Occupied); // Occupied > PotentiallyOccupied
}

// (e) isInBounds gating: out-of-extent samples (and an OOB endpoint) are not written.
TEST(ScanResultToVoxels, OutOfBoundsSamplesNotWritten) {
    Map3DImpl out(unmappedArray(10, 5, 5), mapCfg(10.0)); // world x[0,100)
    // Beam +x from cell 5, hit endpoint at x = 55 + 80 = 135 (cell 13, OUT of extent).
    ScanResultToVoxels::applyToMap(out, P(55, 25, 25), O(0, 0), oneBeam(80.0 * cm, O(0, 0)), lidarCfg(1, 300));

    EXPECT_EQ(out.atVoxel(P(55, 25, 25)), VoxelOccupancy::Empty);       // cell 5, in bounds
    EXPECT_EQ(out.atVoxel(P(95, 25, 25)), VoxelOccupancy::Empty);       // cell 9, last in-bounds
    EXPECT_EQ(out.atVoxel(P(135, 25, 25)), VoxelOccupancy::OutOfBounds); // endpoint, never stored
    for (int xi = 0; xi < 10; ++xi) {
        EXPECT_NE(out.atVoxel(P(xi * 10.0 + 5.0, 25, 25)), VoxelOccupancy::Occupied)
            << "OOB endpoint must not produce an Occupied voxel (cell " << xi << ")";
    }
}

// (f) Lidar -> converter -> map parity: a real scan of a known wall marks the
//     SAME cell the wall occupies. Highest-value end-to-end check.
TEST(ScanResultToVoxels, LidarToConverterToMapParity) {
    // Hidden ground truth with one wall at cell (10,2,2).
    Map3DImpl hidden(unmappedArray(20, 5, 5), mapCfg(10.0));
    hidden.set(P(105, 25, 25), VoxelOccupancy::Occupied);

    // Fixed pose source (the GPS is scaffolding here); real MockLidar over the hidden map.
    test_support::FakeGps exact(P(5, 25, 25), O(0, 0));
    MockLidar lidar(lidarCfg(1, 300), hidden, exact);

    const LidarScanResult scan = lidar.scan(O(0, 0)); // straight at the wall
    ASSERT_EQ(scan.size(), 1u);
    ASSERT_NE(scan[0].distance.force_numerical_value_in(cm), std::numeric_limits<double>::max());

    // Feed the EXACT origin + heading + scan into the converter on a fresh output map.
    Map3DImpl out(unmappedArray(20, 5, 5), mapCfg(10.0));
    ScanResultToVoxels::applyToMap(out, exact.position(), exact.heading(), scan, lidarCfg(1, 300));

    // The Occupied voxel lands on the wall's cell (10,2,2) in both maps.
    EXPECT_EQ(out.atVoxel(P(105, 25, 25)), VoxelOccupancy::Occupied);
    EXPECT_EQ(hidden.atVoxel(P(105, 25, 25)), VoxelOccupancy::Occupied);
    // ...and only there: the cell before is Empty, the cell after is untouched.
    EXPECT_EQ(out.atVoxel(P(95, 25, 25)), VoxelOccupancy::Empty);     // cell 9
    EXPECT_EQ(out.atVoxel(P(115, 25, 25)), VoxelOccupancy::Unmapped); // cell 11
}
