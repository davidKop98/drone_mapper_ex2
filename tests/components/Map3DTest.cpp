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

// (b) raw-byte decode: 0->Empty, 1/9->Occupied, 253->PotentiallyOccupied, 255->Unmapped.
TEST(Map3D, DecodesRawBytes) {
    auto arr = makeArray(5, 1, 1, 0); // ny=nz=1 -> linear index == x
    std::uint8_t* d = arr->Data<std::uint8_t>();
    d[0] = 0;
    d[1] = 1;
    d[2] = 9;   // hidden-map wall written as a nonzero != 1
    d[3] = 253;
    d[4] = 255;
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    EXPECT_EQ(map.atVoxel(P(5, 5, 5)), VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(15, 5, 5)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(25, 5, 5)), VoxelOccupancy::Occupied);            // 9 -> Occupied
    EXPECT_EQ(map.atVoxel(P(35, 5, 5)), VoxelOccupancy::PotentiallyOccupied); // 253
    EXPECT_EQ(map.atVoxel(P(45, 5, 5)), VoxelOccupancy::Unmapped);            // 255
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

// (g) a coordinate exactly on a cell boundary lands in the UPPER cell (epsilon).
TEST(Map3D, BoundaryCoordLandsInUpperCell) {
    auto arr = makeArray(5, 5, 5, 255);
    Map3DImpl map(arr, makeConfig(0, 0, 0, 10.0));
    map.set(P(35, 5, 5), VoxelOccupancy::Occupied); // cell (3,0,0), x in [30,40)
    // A value that has drifted just below the 30.0 boundary must still land in
    // the upper cell (3) thanks to the +epsilon; without it floor() gives cell 2.
    const double drifted = std::nextafter(30.0, 0.0);
    EXPECT_EQ(map.atVoxel(P(drifted, 5, 5)), VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(25.0, 5, 5)), VoxelOccupancy::Unmapped); // cell (2,0,0)
}
