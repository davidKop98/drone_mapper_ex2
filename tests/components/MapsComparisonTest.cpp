#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

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
Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
double scoreOf(const IMap3D& origin, IMap3D& target) {
    std::vector<IMap3D*> targets{&target};
    return MapsComparison::compare(origin, targets).at(0);
}
} // namespace

// Identical maps -> 100.
TEST(MapsComparison, IdenticalMapsScore100) {
    Map3DImpl origin(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    Map3DImpl target(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    for (const auto& w : {P(25, 25, 25), P(5, 5, 5), P(45, 45, 45)}) {
        origin.set(w, VoxelOccupancy::Occupied);
        target.set(w, VoxelOccupancy::Occupied);
    }
    EXPECT_DOUBLE_EQ(scoreOf(origin, target), 100.0);
}

// One voxel different -> below 100 but high.
TEST(MapsComparison, OneVoxelDifferentHighButBelow100) {
    Map3DImpl origin(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5)); // all Empty
    Map3DImpl target(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    target.set(P(25, 25, 25), VoxelOccupancy::Occupied); // one cell disagrees

    const double s = scoreOf(origin, target);
    EXPECT_LT(s, 100.0);
    EXPECT_NEAR(s, 124.0 / 125.0 * 100.0, 1e-9); // 99.2
}

// Fully inverted (all Occupied vs all Empty) -> 0.
TEST(MapsComparison, FullyInvertedScoresZero) {
    Map3DImpl origin(makeArray(4, 4, 4, 1), mapCfg(10, 4, 4, 4)); // all Occupied
    Map3DImpl target(makeArray(4, 4, 4, 0), mapCfg(10, 4, 4, 4)); // all Empty
    EXPECT_DOUBLE_EQ(scoreOf(origin, target), 0.0);
}

// Partially mapped: empties confirmed, half the walls left Unmapped -> mid-score penalty.
TEST(MapsComparison, PartiallyMappedMidScore) {
    Map3DImpl origin(makeArray(4, 4, 4, 0), mapCfg(10, 4, 4, 4));   // 64 cells, Empty
    Map3DImpl target(makeArray(4, 4, 4, 255), mapCfg(10, 4, 4, 4)); // start all Unmapped

    // origin: z-layer 0 is a wall (16 Occupied cells).
    for (int xi = 0; xi < 4; ++xi) {
        for (int yi = 0; yi < 4; ++yi) {
            origin.set(P(xi * 10 + 5, yi * 10 + 5, 5), VoxelOccupancy::Occupied);
        }
    }
    // target: confirm all 48 empties (z-layers 1..3) as Empty.
    for (int xi = 0; xi < 4; ++xi) {
        for (int yi = 0; yi < 4; ++yi) {
            for (int zi = 1; zi < 4; ++zi) {
                target.set(P(xi * 10 + 5, yi * 10 + 5, zi * 10 + 5), VoxelOccupancy::Empty);
            }
        }
    }
    // target: map only 8 of the 16 walls; the other 8 stay Unmapped (the penalty).
    int mapped = 0;
    for (int xi = 0; xi < 4; ++xi) {
        for (int yi = 0; yi < 4; ++yi) {
            if (mapped < 8) {
                target.set(P(xi * 10 + 5, yi * 10 + 5, 5), VoxelOccupancy::Occupied);
                ++mapped;
            }
        }
    }

    // correct = 48 empties + 8 mapped walls = 56; the 8 Unmapped walls are wrong -> 87.5.
    const double s = scoreOf(origin, target);
    EXPECT_NEAR(s, 56.0 / 64.0 * 100.0, 1e-9);
    EXPECT_LT(s, 100.0);
    EXPECT_GT(s, 0.0);
}

// A target with Unmapped over EMPTY origin cells still agrees (both "not Occupied"):
// only unmapped WALLS cost score (documents the EX2 rule vs EX1's exact match).
TEST(MapsComparison, UnmappedOverEmptyStillAgrees) {
    Map3DImpl origin(makeArray(4, 4, 4, 0), mapCfg(10, 4, 4, 4));   // all Empty, no walls
    Map3DImpl target(makeArray(4, 4, 4, 255), mapCfg(10, 4, 4, 4)); // entirely Unmapped
    EXPECT_DOUBLE_EQ(scoreOf(origin, target), 100.0); // not-Occupied vs not-Occupied everywhere
}

// ---- Standalone-comparison hardening (robustness, no rule change) -------------

// Origin smaller than target: scores over the overlapping in-bounds region only.
TEST(MapsComparison, OriginSmallerThanTargetScoresOverOverlap) {
    Map3DImpl origin(makeArray(3, 3, 3, 0), mapCfg(10, 3, 3, 3)); // box [0,30)
    Map3DImpl target(makeArray(6, 6, 6, 0), mapCfg(10, 6, 6, 6)); // box [0,60)
    EXPECT_DOUBLE_EQ(scoreOf(origin, target), 100.0);             // identical over the 27-cell overlap
    target.set(P(5, 5, 5), VoxelOccupancy::Occupied);             // one diff inside the overlap
    EXPECT_NEAR(scoreOf(origin, target), 26.0 / 27.0 * 100.0, 1e-9);
}

// Disjoint boxes (no overlap) -> negative sentinel, never a crash.
TEST(MapsComparison, ZeroOverlapReturnsNegative) {
    MapConfig tc;
    tc.resolution = 10.0 * cm;
    tc.offset = P(100, 100, 100);
    tc.boundaries = MappingBounds{
        100 * x_extent[cm], 130 * x_extent[cm], 100 * y_extent[cm],
        130 * y_extent[cm], 100 * z_extent[cm], 130 * z_extent[cm]};
    Map3DImpl origin(makeArray(3, 3, 3, 0), mapCfg(10, 3, 3, 3)); // box [0,30)
    Map3DImpl target(makeArray(3, 3, 3, 0), tc);                  // box [100,130)
    EXPECT_LT(scoreOf(origin, target), 0.0);
}

// A zero-sized origin grid (effectively empty) -> negative, no crash.
TEST(MapsComparison, EmptyGridReturnsNegative) {
    Map3DImpl origin(makeArray(0, 5, 5, 0), mapCfg(10, 0, 5, 5)); // nx = 0, no cells
    Map3DImpl target(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    EXPECT_LT(scoreOf(origin, target), 0.0);
}
