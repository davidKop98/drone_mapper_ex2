#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <gtest/gtest.h>

#include <sys/wait.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
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

namespace {
// A config whose box starts at a nonzero world minimum: offset (ox,oy,oz) and
// boundaries [ox, ox+nx*res) x [oy, oy+ny*res) x [oz, oz+nz*res).
MapConfig boxCfg(double ox, double oy, double oz, double res,
                 std::size_t nx, std::size_t ny, std::size_t nz) {
    MapConfig c;
    c.offset = P(ox, oy, oz);
    c.resolution = res * cm;
    c.boundaries = MappingBounds{
        ox * x_extent[cm], (ox + static_cast<double>(nx) * res) * x_extent[cm],
        oy * y_extent[cm], (oy + static_cast<double>(ny) * res) * y_extent[cm],
        oz * z_extent[cm], (oz + static_cast<double>(nz) * res) * z_extent[cm],
    };
    return c;
}
} // namespace

// Partially overlapping boxes score over the OVERLAP only: origin [0,30)^3 and target
// [10,40)^3 overlap in [10,30)^3 = 8 cells; one target wall inside the overlap
// disagrees -> exactly 7/8 = 87.5. Counting cells outside the overlap (a dropped
// target-in-bounds guard) would dilute this to 26/27.
TEST(MapsComparison, PartialOverlapScoresOverOverlapOnly) {
    Map3DImpl origin(makeArray(3, 3, 3, 0), mapCfg(10, 3, 3, 3));           // [0,30)^3
    Map3DImpl target(makeArray(3, 3, 3, 0), boxCfg(10, 10, 10, 10, 3, 3, 3)); // [10,40)^3
    target.set(P(15, 15, 15), VoxelOccupancy::Occupied); // one overlap cell disagrees
    EXPECT_NEAR(scoreOf(origin, target), 7.0 / 8.0 * 100.0, 1e-9);
}

// Boxes starting at a NONZERO world minimum (z-range mirrors the real house configs)
// must compare exactly like zero-based ones -- any bounds-local-vs-world confusion in
// either map's lookup lands in the wrong cell or off-map and breaks these values.
TEST(MapsComparison, NonzeroMinBoundsCompareCorrectly) {
    { // matching offsets, identical content incl. walls -> exactly 100
        Map3DImpl origin(makeArray(10, 10, 13, 0), boxCfg(100, 100, 300, 10, 10, 10, 13));
        Map3DImpl target(makeArray(10, 10, 13, 0), boxCfg(100, 100, 300, 10, 10, 10, 13));
        for (const auto& w : {P(105, 105, 305), P(195, 195, 425)}) {
            origin.set(w, VoxelOccupancy::Occupied);
            target.set(w, VoxelOccupancy::Occupied);
        }
        EXPECT_DOUBLE_EQ(scoreOf(origin, target), 100.0);

        // Flip exactly one voxel of the 10*10*13 = 1300 -> (1299/1300) * 100.
        target.set(P(155, 155, 365), VoxelOccupancy::Occupied);
        EXPECT_NEAR(scoreOf(origin, target), 1299.0 / 1300.0 * 100.0, 1e-9);
    }
    { // DIFFERENT offsets, content aligned in world space -> still 100
        Map3DImpl origin(makeArray(5, 5, 5, 0), boxCfg(100, 100, 300, 10, 5, 5, 5)); // [100,150)^2 z[300,350)
        Map3DImpl target(makeArray(10, 10, 10, 0), boxCfg(50, 50, 250, 10, 10, 10, 10)); // [50,150)^2 z[250,350)
        origin.set(P(105, 105, 305), VoxelOccupancy::Occupied); // same WORLD cell in both
        target.set(P(105, 105, 305), VoxelOccupancy::Occupied);
        EXPECT_DOUBLE_EQ(scoreOf(origin, target), 100.0);
    }
}

// PotentiallyOccupied in the target scores as NOT-wall, pinned from both directions
// with asymmetric counts so flipping the rule moves the score: 64 cells, 4 origin
// walls all covered by target-PO (each WRONG: wall vs not-wall) and 2 target-PO over
// origin-Empty (each CORRECT: not-wall vs not-wall) -> exactly 60/64 = 93.75.
// PO-as-wall would score (64-2)/64 = 96.875 instead.
TEST(MapsComparison, PotentiallyOccupiedScoresAsNotWall) {
    Map3DImpl origin(makeArray(4, 4, 4, 0), mapCfg(10, 4, 4, 4));
    Map3DImpl target(makeArray(4, 4, 4, 0), mapCfg(10, 4, 4, 4));
    for (const auto& w : {P(5, 5, 5), P(15, 5, 5), P(25, 5, 5), P(35, 5, 5)}) {
        origin.set(w, VoxelOccupancy::Occupied);              // origin wall...
        target.set(w, VoxelOccupancy::PotentiallyOccupied);   // ...target only suspects it
    }
    target.set(P(5, 35, 35), VoxelOccupancy::PotentiallyOccupied);  // PO over Empty
    target.set(P(15, 35, 35), VoxelOccupancy::PotentiallyOccupied); // PO over Empty
    EXPECT_NEAR(scoreOf(origin, target), 60.0 / 64.0 * 100.0, 1e-9);
}

// ---- Standalone exe contract (the PDF's acceptance checks, run for real) --------

namespace {
// Run the built maps_comparison exe; capture stdout/stderr; return the exit code.
int runComparisonExe(const std::string& args, std::string& out, std::string& err) {
    namespace fs = std::filesystem;
    const fs::path out_p = fs::temp_directory_path() / "dm_cmp_out.txt";
    const fs::path err_p = fs::temp_directory_path() / "dm_cmp_err.txt";
    const std::string cmd = std::string(MAPS_COMPARISON_EXE_PATH) + " " + args + " > " +
                            out_p.string() + " 2> " + err_p.string();
    const int raw = std::system(cmd.c_str());
    const auto slurp = [](const fs::path& p) {
        std::ifstream f(p);
        return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    };
    out = slurp(out_p);
    err = slurp(err_p);
    fs::remove(out_p);
    fs::remove(err_p);
    return WIFEXITED(raw) ? WEXITSTATUS(raw) : -1;
}

std::filesystem::path writeWallCubeNpy(const char* name) {
    auto arr = makeArray(3, 3, 3, 0);
    arr->Data<std::uint8_t>()[1 * 3 * 3 + 1 * 3 + 1] = 1; // center wall
    const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
    EXPECT_EQ(arr->SaveNPY(path.string()), nullptr);
    return path;
}
} // namespace

// Identical maps: stdout is EXACTLY one parseable number equal to 100 (any prefix or
// suffix text breaks the parse or the trailing-garbage check), exit code 0.
TEST(MapsComparison, ExePrintsBareScoreOnStdout) {
    const auto origin = writeWallCubeNpy("dm_cmp_origin.npy");
    const auto target = writeWallCubeNpy("dm_cmp_target.npy");

    std::string out, err;
    const int rc = runComparisonExe(origin.string() + " " + target.string(), out, err);
    EXPECT_EQ(rc, 0);

    std::istringstream iss(out);
    double value = -999.0;
    ASSERT_TRUE(static_cast<bool>(iss >> value)) << "stdout is not a number: '" << out << "'";
    EXPECT_DOUBLE_EQ(value, 100.0);
    std::string rest;
    iss >> rest;
    EXPECT_TRUE(rest.empty()) << "extra text on stdout: '" << out << "'";

    std::filesystem::remove(origin);
    std::filesystem::remove(target);
}

// Load failure: stdout is exactly "-1", stderr carries a description, exit nonzero.
TEST(MapsComparison, ExeFailurePrintsMinusOneOnStdout) {
    std::string out, err;
    const int rc = runComparisonExe("/nonexistent/origin.npy /nonexistent/target.npy", out, err);
    EXPECT_NE(rc, 0);
    std::istringstream iss(out);
    std::string first;
    iss >> first;
    EXPECT_EQ(first, "-1");
    std::string rest;
    iss >> rest;
    EXPECT_TRUE(rest.empty()) << "extra text on stdout: '" << out << "'";
    EXPECT_FALSE(err.empty()) << "expected a description on stderr";
}
