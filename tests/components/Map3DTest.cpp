#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;

// All-`fill` uint8 array of shape [nx][ny][nz].
std::shared_ptr<NpyArray> makeArray(std::size_t nx, std::size_t ny, std::size_t nz, std::uint8_t fill) {
    auto arr = std::make_shared<NpyArray>(NpyArray::shape_t{nx, ny, nz}, sizeof(std::uint8_t), 'u', false);
    arr->Allocate();
    std::uint8_t* data = arr->Data<std::uint8_t>();
    std::fill(data, data + arr->NumValue(), fill);
    return arr;
}

MapConfig makeConfig(double ox, double oy, double oz, double res) {
    MapConfig c;
    c.offset = Position3D{ox * x_extent[cm], oy * y_extent[cm], oz * z_extent[cm]};
    c.resolution = res * cm;
    return c;
}

Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
} // namespace

// (a) snapping at 1 cm: world position buckets into the right cell.
TEST(Map3D, SnappingAt1cm) {
    auto arr = makeArray(10, 10, 10, 255);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 1.0));
    map.set(P(3.5, 0.5, 0.5), VoxelOccupancy::Occupied); // cell (3,0,0)
    EXPECT_EQ(map.atVoxel(P(3.0, 0.0, 0.0)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(3.9, 0.9, 0.9)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(2.9, 0.0, 0.0)), VoxelOccupancy::Unmapped); // cell (2,0,0)
    EXPECT_EQ(map.atVoxel(P(4.0, 0.0, 0.0)), VoxelOccupancy::Unmapped); // cell (4,0,0)
}

// (a) snapping at 10 cm (the real cell size): cell width is 10, not 1.
TEST(Map3D, SnappingAt10cm) {
    auto arr = makeArray(5, 5, 5, 255);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    map.set(P(35.0, 5.0, 5.0), VoxelOccupancy::Occupied); // cell (3,0,0), x in [30,40)
    EXPECT_EQ(map.atVoxel(P(30.0, 0.0, 0.0)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(39.999, 9.0, 9.0)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(29.999, 0.0, 0.0)), VoxelOccupancy::Unmapped); // cell (2,..)
    EXPECT_EQ(map.atVoxel(P(40.0, 0.0, 0.0)), VoxelOccupancy::Unmapped);   // cell (4,..)
}

// (b) raw-byte decode: 0->Empty, 1/9/254->Occupied, 253->PotentiallyOccupied,
// 255->Unmapped. The 254 case pins BOTH sentinel constants against +-1 drift: it is
// one off from each and must fall through to Occupied, never to a sentinel state.
TEST(Map3D, DecodesRawBytes) {
    auto arr = makeArray(6, 1, 1, 0); // ny=nz=1 -> linear index == x
    std::uint8_t* d = arr->Data<std::uint8_t>();
    d[0] = 0;
    d[1] = 1;
    d[2] = 9;   // hidden-map wall written as a nonzero != 1
    d[3] = 253;
    d[4] = 254; // one off from each sentinel -> plain Occupied
    d[5] = 255;
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    EXPECT_EQ(map.atVoxel(P(5, 5, 5)), VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(15, 5, 5)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(25, 5, 5)), VoxelOccupancy::Occupied);            // 9 -> Occupied
    EXPECT_EQ(map.atVoxel(P(35, 5, 5)), VoxelOccupancy::PotentiallyOccupied); // 253
    EXPECT_EQ(map.atVoxel(P(45, 5, 5)), VoxelOccupancy::Occupied);            // 254
    EXPECT_EQ(map.atVoxel(P(55, 5, 5)), VoxelOccupancy::Unmapped);            // 255
}

// (c) isInBounds: inside true, outside false.
TEST(Map3D, IsInBounds) {
    auto arr = makeArray(5, 5, 5, 255);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0)); // world [0,50)^3
    EXPECT_TRUE(map.isInBounds(P(0, 0, 0)));
    EXPECT_TRUE(map.isInBounds(P(25, 25, 25)));
    EXPECT_TRUE(map.isInBounds(P(49.9, 49.9, 49.9)));
    EXPECT_FALSE(map.isInBounds(P(-0.1, 0, 0)));
    EXPECT_FALSE(map.isInBounds(P(50, 0, 0)));  // x cell 5 -> out
    EXPECT_FALSE(map.isInBounds(P(0, 0, 55)));
}

// (d) atVoxel out of extent -> OutOfBounds.
TEST(Map3D, OutOfBoundsReturnsOutOfBounds) {
    auto arr = makeArray(5, 5, 5, 0);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    EXPECT_EQ(map.atVoxel(P(-5, 0, 0)), VoxelOccupancy::OutOfBounds);
    EXPECT_EQ(map.atVoxel(P(50, 0, 0)), VoxelOccupancy::OutOfBounds);
    EXPECT_EQ(map.atVoxel(P(0, 0, 100)), VoxelOccupancy::OutOfBounds);
}

// (e) negative offset snaps a negative-world position into [0,n).
TEST(Map3D, NegativeOffsetSnapsIntoRange) {
    auto arr = makeArray(10, 10, 10, 255);
    Map3DImpl map(arr, makeConfig(-50, -50, -50, 10.0)); // world [-50,50)^3
    map.set(P(-45, -45, -45), VoxelOccupancy::Occupied); // -> cell (0,0,0)
    EXPECT_TRUE(map.isInBounds(P(-45, -45, -45)));
    EXPECT_EQ(map.atVoxel(P(-50, -50, -50)), VoxelOccupancy::Occupied); // cell 0
    EXPECT_EQ(map.atVoxel(P(-41, -41, -41)), VoxelOccupancy::Occupied); // still cell 0 ([-50,-40))
    EXPECT_EQ(map.atVoxel(P(-39, -50, -50)), VoxelOccupancy::Unmapped); // x cell 1
    EXPECT_FALSE(map.isInBounds(P(-51, 0, 0)));
    EXPECT_FALSE(map.isInBounds(P(50, 0, 0)));
}

// (f) set a pattern incl. 253, save, reload -> identical voxels.
TEST(Map3D, SaveReloadRoundTrip) {
    auto arr = makeArray(5, 5, 5, 255);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    map.set(P(5, 5, 5), VoxelOccupancy::Empty);
    map.set(P(15, 5, 5), VoxelOccupancy::Occupied);
    map.set(P(25, 25, 25), VoxelOccupancy::PotentiallyOccupied); // 253
    map.set(P(45, 45, 45), VoxelOccupancy::Unmapped);

    // The raw stored byte for PotentiallyOccupied is exactly 253 -- a symmetric
    // encode/decode corruption would round-trip cleanly and hide otherwise.
    EXPECT_EQ(arr->Data<std::uint8_t>()[2 * 5 * 5 + 2 * 5 + 2], 253);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "dm_map3d_roundtrip.npy";
    map.save(path);

    auto reloaded = std::make_shared<NpyArray>();
    ASSERT_EQ(reloaded->LoadNPY(path.string()), nullptr); // null == success
    Map3DImpl reloaded_map(reloaded, makeConfig(0, 0, 0, 10.0));

    for (double x = 5; x < 50; x += 10) {
        for (double y = 5; y < 50; y += 10) {
            for (double z = 5; z < 50; z += 10) {
                EXPECT_EQ(reloaded_map.atVoxel(P(x, y, z)), map.atVoxel(P(x, y, z)))
                    << "mismatch at " << x << "," << y << "," << z;
            }
        }
    }
    EXPECT_EQ(reloaded_map.atVoxel(P(25, 25, 25)), VoxelOccupancy::PotentiallyOccupied);
    std::filesystem::remove(path);
}

// (g) epsilon, both sides: a coordinate on/just below a cell boundary lands in the
// UPPER cell (epsilon present, correct sign), while a coordinate solidly mid-cell is
// unmoved (epsilon not oversized). Catches removal, sign-flip, and ~1e6x inflation.
TEST(Map3D, BoundaryCoordLandsInUpperCell) {
    auto arr = makeArray(5, 5, 5, 255);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    map.set(P(35, 5, 5), VoxelOccupancy::Occupied); // cell (3,0,0), x in [30,40)
    // Exactly on the boundary: upper cell (a negative epsilon pushes it to cell 2).
    EXPECT_EQ(map.atVoxel(P(30.0, 5, 5)), VoxelOccupancy::Occupied);
    // A value that has drifted just below the 30.0 boundary must still land in
    // the upper cell (3) thanks to the +epsilon; without it floor() gives cell 2.
    const double drifted = std::nextafter(30.0, 0.0);
    EXPECT_EQ(map.atVoxel(P(drifted, 5, 5)), VoxelOccupancy::Occupied);
    // 0.05cm below the boundary is genuinely cell 2: an inflated epsilon (1e-3 of a
    // cell or more) would wrongly promote it into cell 3.
    EXPECT_EQ(map.atVoxel(P(29.995, 5, 5)), VoxelOccupancy::Unmapped);
    EXPECT_EQ(map.atVoxel(P(25.0, 5, 5)), VoxelOccupancy::Unmapped); // cell (2,0,0)
}

// (h) distinct per-axis offsets: each axis must subtract ITS OWN offset. The probe
// point lands in cell (2,3,4) only if x uses 10, y uses 20 and z uses 30; any axis
// mix-up (or a world+offset sign flip) mis-snaps at least one axis. Verified against
// hand-planted/hand-read raw bytes so a set/atVoxel pair sharing the same broken
// snap cannot round-trip its way to a pass.
TEST(Map3D, DistinctPerAxisOffsetSnapsCorrectly) {
    auto arr = makeArray(6, 6, 6, 255);
    std::uint8_t* d = arr->Data<std::uint8_t>();
    d[2 * 6 * 6 + 3 * 6 + 4] = 1; // plant Occupied at cell (2,3,4), C-order
    Map3DImpl map(arr, makeConfig(10, 20, 30, 10.0)); // world x[10,70) y[20,80) z[30,90)

    // (35,55,75): x (35-10)/10 -> 2, y (55-20)/10 -> 3, z (75-30)/10 -> 4.
    EXPECT_EQ(map.atVoxel(P(35, 55, 75)), VoxelOccupancy::Occupied);
    // Cell boundary under nonzero offsets keeps the epsilon honest: x=30 is the lower
    // face of cell 2 (offset 10 + 2*10), still inside the planted cell.
    EXPECT_EQ(map.atVoxel(P(std::nextafter(30.0, 0.0), 55, 75)), VoxelOccupancy::Occupied);
    // Inverse path: set through the map, then hand-read the raw byte at (1,2,3).
    map.set(P(25, 45, 65), VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(d[1 * 6 * 6 + 2 * 6 + 3], 253);
}

// (i) C-order memory contract on a NON-CUBIC grid: linear = x*ny*nz + y*nz + z.
// set/atVoxel round-trips are self-consistent under any index formula, so the
// contract is pinned against hand-computed raw-byte indices; the exhaustive sweep
// catches aliasing (dropped factors, swapped terms, dimension-order bugs).
TEST(Map3D, RawByteAtNonzeroIndexReadsThroughAtVoxel) {
    auto arr = makeArray(2, 3, 4, 255);
    std::uint8_t* d = arr->Data<std::uint8_t>();
    d[1 * 3 * 4 + 2 * 4 + 3] = 1; // cell (1,2,3) -> linear 23
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));

    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 3; ++y) {
            for (int z = 0; z < 4; ++z) {
                const VoxelOccupancy expect = (x == 1 && y == 2 && z == 3)
                                                  ? VoxelOccupancy::Occupied
                                                  : VoxelOccupancy::Unmapped;
                EXPECT_EQ(map.atVoxel(P(x * 10 + 5, y * 10 + 5, z * 10 + 5)), expect)
                    << "cell (" << x << "," << y << "," << z << ")";
            }
        }
    }
    // Inverse: set at nonzero (x,y,z), raw byte lands at the hand-computed index.
    map.set(P(15, 5, 25), VoxelOccupancy::PotentiallyOccupied); // cell (1,0,2)
    EXPECT_EQ(d[1 * 3 * 4 + 0 * 4 + 2], 253);
}
