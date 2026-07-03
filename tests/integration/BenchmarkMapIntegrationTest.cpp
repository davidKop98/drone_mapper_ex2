// Integration on the staff benchmark map (data_maps/benchmark_map.npy): a solid ground
// block (z=0..14) with a two-story house on top (z=15..27, footprint x[2,26] y[2,25]),
// a 4-wide doorway in the y=25 wall at x=21..24, a stairwell hole at z=20, and open air
// above the roof. BENCHMARK_MAP_PATH is the absolute source path baked in by CMake.
//
// This is the first, basic configuration (radius = resolution); the runBenchmark helper
// is parameterized so further variants (drone size, gps, start, steps) are one-liners.
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
namespace fs = std::filesystem;

Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
MappingBounds fullBounds(double res, std::size_t nx, std::size_t ny, std::size_t nz) {
    return MappingBounds{
        0.0 * x_extent[cm], static_cast<double>(nx) * res * x_extent[cm],
        0.0 * y_extent[cm], static_cast<double>(ny) * res * y_extent[cm],
        0.0 * z_extent[cm], static_cast<double>(nz) * res * z_extent[cm],
    };
}

// A world-coordinate box in cells (inclusive lo, exclusive hi), e.g. cellBox(2,26,...) is the
// house footprint x[20,260]cm. Lets a test concentrate the scored region on the structure.
MappingBounds cellBox(double x0, double x1, double y0, double y1, double z0, double z1,
                      double res) {
    return MappingBounds{x0 * res * x_extent[cm], x1 * res * x_extent[cm],
                         y0 * res * y_extent[cm], y1 * res * y_extent[cm],
                         z0 * res * z_extent[cm], z1 * res * z_extent[cm]};
}

// Run one benchmark-map mission. `bounds` sizes the output map AND the region the drone may
// explore AND the scored region (MapsComparison only counts samples in-bounds in BOTH maps),
// so shrinking it concentrates the score on whatever the box contains. `factor` is the
// output_mapping_resolution_factor (expected res = gps / factor).
SimulationManagerReport runBenchmarkBounded(const fs::path& out, double res_cm, double radius_cm,
                                            double gps_cm, Position3D start, double heading_deg,
                                            std::size_t max_steps, MappingBounds bounds,
                                            double factor = 1.0) {
    SimulationConfigData sim;
    sim.map_filename = BENCHMARK_MAP_PATH;
    sim.map_resolution = res_cm * cm;
    sim.map_offset = P(0, 0, 0);
    sim.initial_drone_position = start;
    sim.initial_angle = heading_deg * horizontal_angle[deg];

    MissionConfigData mission;
    mission.max_steps = max_steps;
    mission.gps_resolution = gps_cm * cm;
    mission.output_mapping_resolution_factor = factor;
    mission.mission_bounds = bounds;

    DroneConfigData drone;
    drone.radius = radius_cm * cm;
    drone.max_rotate = 45.0 * horizontal_angle[deg];
    drone.max_advance = 10.0 * cm;
    drone.max_elevate = 10.0 * cm;

    LidarConfigData lidar;
    lidar.z_min = 10.0 * cm;
    lidar.z_max = 100.0 * cm;
    lidar.d = 10.0 * cm;
    lidar.fov_circles = 1;

    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{sim, {mission}}};
    comp.drones = {drone};
    comp.lidars = {lidar};

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager mgr(std::move(factory));
    return mgr.run(comp, out);
}

// Full-map-extent convenience wrapper (the original variants score over the whole map).
SimulationManagerReport runBenchmark(const fs::path& out, double res_cm, double radius_cm,
                                     double gps_cm, Position3D start, double heading_deg,
                                     std::size_t max_steps, double factor = 1.0) {
    return runBenchmarkBounded(out, res_cm, radius_cm, gps_cm, start, heading_deg, max_steps,
                               fullBounds(res_cm, 29, 30, 31), factor);
}

// Translate a bounds box by a map offset (cm), so a cellBox built in the offset-0 frame lands
// correctly when the hidden map itself is shifted (negative-offset tests).
MappingBounds offsetBounds(MappingBounds b, double ox, double oy, double oz) {
    return MappingBounds{b.min_x + ox * x_extent[cm], b.max_x + ox * x_extent[cm],
                         b.min_y + oy * y_extent[cm], b.max_y + oy * y_extent[cm],
                         b.min_height + oz * z_extent[cm], b.max_height + oz * z_extent[cm]};
}

// Fully-parameterized run: every knob a "radical" test might twist (map offset, lidar geometry,
// drone kinematics, gps). Defaults match the normal benchmark config.
struct RunSpec {
    std::string map_path = BENCHMARK_MAP_PATH;
    double res_cm = 10.0;
    double off_x = 0.0, off_y = 0.0, off_z = 0.0; // hidden-map offset (cm)
    double radius_cm = 2.0;
    double gps_cm = 2.0;
    Position3D start{};
    double heading_deg = 0.0;
    std::size_t max_steps = 60000;
    MappingBounds bounds{};
    double lz_min = 10.0, lz_max = 100.0, ld = 10.0; // lidar geometry
    std::size_t fov_circles = 1;
    double max_rotate = 45.0, max_advance = 10.0, max_elevate = 10.0; // drone kinematics
};

SimulationManagerReport runSpec(const fs::path& out, const RunSpec& s) {
    SimulationConfigData sim;
    sim.map_filename = s.map_path;
    sim.map_resolution = s.res_cm * cm;
    sim.map_offset = P(s.off_x, s.off_y, s.off_z);
    sim.initial_drone_position = s.start;
    sim.initial_angle = s.heading_deg * horizontal_angle[deg];

    MissionConfigData mission;
    mission.max_steps = s.max_steps;
    mission.gps_resolution = s.gps_cm * cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = s.bounds;

    DroneConfigData drone;
    drone.radius = s.radius_cm * cm;
    drone.max_rotate = s.max_rotate * horizontal_angle[deg];
    drone.max_advance = s.max_advance * cm;
    drone.max_elevate = s.max_elevate * cm;

    LidarConfigData lidar;
    lidar.z_min = s.lz_min * cm;
    lidar.z_max = s.lz_max * cm;
    lidar.d = s.ld * cm;
    lidar.fov_circles = s.fov_circles;

    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{sim, {mission}}};
    comp.drones = {drone};
    comp.lidars = {lidar};

    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager mgr(std::move(factory));
    return mgr.run(comp, out);
}

// "Acted properly on a weird input" = no crash, a single run, a status from the valid enum, and
// a score that's either the no-overlap sentinel (-1) or a real percentage in [0,100].
void expectGraceful(const char* tag, const SimulationManagerReport& report) {
    ASSERT_EQ(report.runs.size(), 1u) << tag;
    const SimulationResult& r = report.runs.at(0);
    ASSERT_FALSE(r.mission_results.empty()) << tag;
    const MissionRunStatus st = r.mission_results.at(0).status;
    EXPECT_TRUE(st == MissionRunStatus::Completed || st == MissionRunStatus::MaxSteps ||
                st == MissionRunStatus::Error)
        << tag;
    EXPECT_GE(r.mission_score, -1.0) << tag;
    EXPECT_LE(r.mission_score, 100.0) << tag;
    std::cerr << "[radical] " << tag << " score=" << r.mission_score
              << " steps=" << r.mission_results.at(0).steps
              << " status=" << static_cast<int>(st) << '\n';
}
} // namespace

// Basic configuration: radius = resolution (10cm), small gps, start in the yard at z=17
// (clear of the solid ground), capped steps. Confirms the whole stack runs on the real map.
TEST(Integration, BenchmarkMapBasicRun) {
    ASSERT_TRUE(fs::exists(fs::path(BENCHMARK_MAP_PATH))) << "benchmark map missing";

    const fs::path out = fs::temp_directory_path() / "dm_benchmark_basic";
    std::error_code ec;
    fs::remove_all(out, ec);

    const auto report = runBenchmark(out, /*res*/ 10.0, /*radius*/ 10.0, /*gps*/ 2.0,
                                     P(225, 275, 175), /*heading*/ 270.0, /*max_steps*/ 20000);

    ASSERT_EQ(report.runs.size(), 1u);
    const SimulationResult& result = report.runs.at(0);
    ASSERT_FALSE(result.mission_results.empty());
    const MissionRunResult& mission = result.mission_results.at(0);

    EXPECT_NE(mission.status, MissionRunStatus::Error); // ran cleanly (Completed or MaxSteps)
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    EXPECT_TRUE(fs::exists(out / "simulation_output.yaml"));
    EXPECT_TRUE(fs::exists(out / "output_results" / "run_0" / "output_map.npy"));

    std::cerr << "[benchmark] basic score=" << result.mission_score << " steps=" << mission.steps
              << " status=" << static_cast<int>(mission.status) << '\n';

    fs::remove_all(out, ec);
}

// Small sub-cell drone (radius 2cm < resolution): a tighter footprint can hug surfaces the
// 10cm drone cannot, so it explores more and is given a larger step budget. Confirms the
// stack still runs cleanly and produces a positive score with a small drone.
TEST(Integration, BenchmarkMapSmallDrone) {
    const fs::path out = fs::temp_directory_path() / "dm_benchmark_small";
    std::error_code ec;
    fs::remove_all(out, ec);

    const auto report = runBenchmark(out, /*res*/ 10.0, /*radius*/ 2.0, /*gps*/ 2.0,
                                     P(225, 275, 175), /*heading*/ 270.0, /*max_steps*/ 40000);

    ASSERT_EQ(report.runs.size(), 1u);
    const SimulationResult& result = report.runs.at(0);
    ASSERT_FALSE(result.mission_results.empty());
    const MissionRunResult& mission = result.mission_results.at(0);

    EXPECT_NE(mission.status, MissionRunStatus::Error);
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    EXPECT_TRUE(fs::exists(out / "output_results" / "run_0" / "output_map.npy"));

    std::cerr << "[benchmark] small score=" << result.mission_score << " steps=" << mission.steps
              << " status=" << static_cast<int>(mission.status) << '\n';

    fs::remove_all(out, ec);
}

// Larger drone (radius 20cm = 2 cells) with a coarser 4cm gps. Clearance = radius + gps/2 =
// 22cm, so it must stay well above the solid ground (top at z=150cm); started at z=185cm in
// the yard. More constrained movement; the run must still complete cleanly and score > 0.
TEST(Integration, BenchmarkMapLargerDrone) {
    const fs::path out = fs::temp_directory_path() / "dm_benchmark_large";
    std::error_code ec;
    fs::remove_all(out, ec);

    const auto report = runBenchmark(out, /*res*/ 10.0, /*radius*/ 20.0, /*gps*/ 4.0,
                                     P(225, 275, 185), /*heading*/ 270.0, /*max_steps*/ 15000);

    ASSERT_EQ(report.runs.size(), 1u);
    const SimulationResult& result = report.runs.at(0);
    ASSERT_FALSE(result.mission_results.empty());
    const MissionRunResult& mission = result.mission_results.at(0);

    EXPECT_NE(mission.status, MissionRunStatus::Error);
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    EXPECT_TRUE(fs::exists(out / "output_results" / "run_0" / "output_map.npy"));

    std::cerr << "[benchmark] large score=" << result.mission_score << " steps=" << mission.steps
              << " status=" << static_cast<int>(mission.status) << '\n';

    fs::remove_all(out, ec);
}

// Coarse gps equal to the map resolution (10cm) with factor 1: expected output resolution =
// gps/factor = 10cm == the 10cm map resolution we use -> Accepted. Exercises the resolution
// decision on the benchmark map (vs. the basic run's IgnoredTooSmall at gps 2cm).
TEST(Integration, BenchmarkMapCoarseGpsAcceptedResolution) {
    const fs::path out = fs::temp_directory_path() / "dm_benchmark_coarse";
    std::error_code ec;
    fs::remove_all(out, ec);

    const auto report = runBenchmark(out, /*res*/ 10.0, /*radius*/ 10.0, /*gps*/ 10.0,
                                     P(225, 275, 175), /*heading*/ 270.0, /*max_steps*/ 12000,
                                     /*factor*/ 1.0);

    ASSERT_EQ(report.runs.size(), 1u);
    const SimulationResult& result = report.runs.at(0);
    ASSERT_FALSE(result.mission_results.empty());
    const MissionRunResult& mission = result.mission_results.at(0);

    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
    EXPECT_NE(mission.status, MissionRunStatus::Error);
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);

    std::cerr << "[benchmark] coarse score=" << result.mission_score << " steps=" << mission.steps
              << " status=" << static_cast<int>(mission.status) << '\n';

    fs::remove_all(out, ec);
}

// Start in the open air above the roof (roof at z=27, air z=28..30) looking down: the drone
// maps the house roof from above rather than approaching from the yard. A different vantage
// point over the same map; confirms a clean run and a positive score from above.
TEST(Integration, BenchmarkMapAboveRoof) {
    const fs::path out = fs::temp_directory_path() / "dm_benchmark_roof";
    std::error_code ec;
    fs::remove_all(out, ec);

    const auto report = runBenchmark(out, /*res*/ 10.0, /*radius*/ 10.0, /*gps*/ 2.0,
                                     P(135, 135, 295), /*heading*/ 0.0, /*max_steps*/ 12000);

    ASSERT_EQ(report.runs.size(), 1u);
    const SimulationResult& result = report.runs.at(0);
    ASSERT_FALSE(result.mission_results.empty());
    const MissionRunResult& mission = result.mission_results.at(0);

    EXPECT_NE(mission.status, MissionRunStatus::Error);
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    EXPECT_TRUE(fs::exists(out / "output_results" / "run_0" / "output_map.npy"));

    std::cerr << "[benchmark] roof score=" << result.mission_score << " steps=" << mission.steps
              << " status=" << static_cast<int>(mission.status) << '\n';

    fs::remove_all(out, ec);
}

// ---- House-concentrated scoring: shrink mission_bounds so the structure carries the weight ----
//
// With full-map bounds every config scores ~44 because ~13000 buried ground voxels (Occupied
// in the hidden map, forever unmappable) dominate the denominator. Restricting mission_bounds
// to the house (z>=15, above the ground) makes the score reflect actual structure mapping.
//
// FINDING (see commit discussion): even so, a small drone tops out near ~90%, NOT ~100% --
// it maps floor 1 and the z=20 slab (from below) perfectly, but never ascends the 4x4
// stairwell hole to floor 2 (floor 2 / roof stay ~95% Unmapped). The cause is exploration
// order: "up" is the lowest-priority DFS direction, so by the time the hole cell above is
// confirmed Empty the drone has already marked the cell below it visited and never returns.
// These tests pin the demonstrated behavior; they do NOT assert ~100 (that gap is a separate
// question about the exploration strategy, intentionally left for follow-up).

// House-tight bounds, small drone started inside floor 1: concentrating the score on the
// house lifts it far above the full-map ~44, demonstrating bounds concentration works.
TEST(Integration, BenchmarkMapHouseConcentratedInteriorStart) {
    const fs::path out = fs::temp_directory_path() / "dm_house_small";
    std::error_code ec; fs::remove_all(out, ec);

    const auto report = runBenchmarkBounded(out, /*res*/ 10.0, /*radius*/ 2.0, /*gps*/ 2.0,
        P(145, 105, 175), /*heading*/ 0.0, /*max_steps*/ 80000,
        cellBox(2, 26, 2, 25, 15, 27, 10.0)); // house footprint, z>=15 (above the ground)

    const SimulationResult& r = report.runs.at(0);
    ASSERT_FALSE(r.mission_results.empty());
    EXPECT_NE(r.mission_results.at(0).status, MissionRunStatus::Error);
    EXPECT_GT(r.mission_score, 80.0);  // vs ~44 over full bounds -- ground dilution removed
    EXPECT_LE(r.mission_score, 100.0);

    std::cerr << "[benchmark] house-small score=" << r.mission_score
              << " steps=" << r.mission_results.at(0).steps << '\n';
    fs::remove_all(out, ec);
}

// The discriminating test the worry was really about: with the ground dilution gone and a
// yard apron added so a yard-started drone can reach the doorway, the small drone (fits the
// 4-wide / 40cm door) outscores the large drone (radius 20cm -> ~44cm footprint, cannot enter
// and Completes quickly outside). Drone size now visibly matters -- it was masked before.
TEST(Integration, BenchmarkMapYardEntryFavorsSmallDrone) {
    const MappingBounds apron = cellBox(2, 26, 2, 29, 15, 27, 10.0); // house + yard rows to the door
    std::error_code ec;

    const fs::path os = fs::temp_directory_path() / "dm_yard_small"; fs::remove_all(os, ec);
    const auto rs = runBenchmarkBounded(os, 10.0, /*radius*/ 2.0, 2.0,
        P(225, 275, 175), /*heading*/ 90.0, /*max_steps*/ 80000, apron).runs.at(0);

    const fs::path ol = fs::temp_directory_path() / "dm_yard_large"; fs::remove_all(ol, ec);
    const auto rl = runBenchmarkBounded(ol, 10.0, /*radius*/ 20.0, 2.0,
        P(225, 275, 185), /*heading*/ 90.0, /*max_steps*/ 40000, apron).runs.at(0);

    ASSERT_FALSE(rs.mission_results.empty());
    ASSERT_FALSE(rl.mission_results.empty());
    EXPECT_NE(rs.mission_results.at(0).status, MissionRunStatus::Error);
    EXPECT_NE(rl.mission_results.at(0).status, MissionRunStatus::Error);
    EXPECT_GT(rs.mission_score, rl.mission_score); // the small drone gets inside; the large can't

    std::cerr << "[benchmark] yard-small score=" << rs.mission_score
              << "  yard-large score=" << rl.mission_score << '\n';
    fs::remove_all(os, ec); fs::remove_all(ol, ec);
}

// ---- Parameter-variation suite: varied starts, drone sizes, and sub-region boundaries. ----
// Shared shape so each variant is a few lines; asserts no Error + a measured score floor.
namespace {
struct VariantExpectation {
    double radius_cm, gps_cm;
    Position3D start;
    double heading_deg;
    std::size_t max_steps;
    MappingBounds bounds;
    double min_score; // conservative floor below the measured value
};

void runVariant(const char* tag, const VariantExpectation& v) {
    const fs::path out = fs::temp_directory_path() / "dm_variant";
    std::error_code ec;
    fs::remove_all(out, ec);

    const auto report = runBenchmarkBounded(out, 10.0, v.radius_cm, v.gps_cm, v.start,
                                            v.heading_deg, v.max_steps, v.bounds);
    ASSERT_EQ(report.runs.size(), 1u) << tag;
    const SimulationResult& r = report.runs.at(0);
    ASSERT_FALSE(r.mission_results.empty()) << tag;
    EXPECT_NE(r.mission_results.at(0).status, MissionRunStatus::Error) << tag;
    EXPECT_GT(r.mission_score, v.min_score) << tag;
    EXPECT_LE(r.mission_score, 100.0) << tag;
    EXPECT_TRUE(fs::exists(out / "output_results" / "run_0" / "output_map.npy")) << tag;

    std::cerr << "[variant] " << tag << " score=" << r.mission_score
              << " steps=" << r.mission_results.at(0).steps << '\n';
    fs::remove_all(out, ec);
}
} // namespace

// BOUNDARY: a single-floor sub-box (house footprint, z 15..19 only). With just one floor and a
// small drone, nearly every in-bounds voxel gets mapped -> score approaches 100 (measured 99.9).
TEST(Integration, BenchmarkMapFloor1OnlySubRegion) {
    runVariant("floor1-only r2 interior",
        {/*r*/ 2, /*gps*/ 2, P(145, 105, 175), /*h*/ 0, /*cap*/ 250000,
         cellBox(2, 26, 2, 25, 15, 20, 10.0), /*min*/ 92.0});
}

// SIZE: a tiny sub-cell drone (radius 1cm) from the yard over the full map. Mostly checks the
// stack stays sane at the small extreme; the ~46 score is the full-map ground dilution.
TEST(Integration, BenchmarkMapTinyDroneFullMap) {
    runVariant("tiny r1 yard full",
        {/*r*/ 1, /*gps*/ 2, P(225, 275, 175), /*h*/ 270, /*cap*/ 50000,
         fullBounds(10.0, 29, 30, 31), /*min*/ 0.0});
}

// SIZE: a medium drone (radius 5cm = half a cell) mapping the house interior. Fits the doorway
// and stairwell, so it still maps both floors well (measured ~91).
TEST(Integration, BenchmarkMapMediumDroneHouse) {
    runVariant("medium r5 house interior",
        {/*r*/ 5, /*gps*/ 2, P(145, 105, 175), /*h*/ 0, /*cap*/ 120000,
         cellBox(2, 26, 2, 25, 15, 27, 10.0), /*min*/ 75.0});
}

// BOUNDARY + START: a sub-box over the roof and the air above it, drone started above the roof
// looking down. Concentrates scoring on the roof slab, which maps cleanly from above (~97).
TEST(Integration, BenchmarkMapRoofRegionFromAbove) {
    runVariant("roof-region r10 above",
        {/*r*/ 10, /*gps*/ 2, P(135, 135, 295), /*h*/ 0, /*cap*/ 60000,
         cellBox(2, 26, 2, 25, 26, 31, 10.0), /*min*/ 85.0});
}

// BOUNDARY: a corner quadrant of the ORIGINAL map (x,y 0..15), full height incl. the buried
// ground. Score is capped (~49) by the unmappable ground block -- a deliberately hard region.
TEST(Integration, BenchmarkMapCornerQuadrant) {
    runVariant("map-quadrant r2 ground",
        {/*r*/ 2, /*gps*/ 2, P(75, 75, 175), /*h*/ 0, /*cap*/ 80000,
         cellBox(0, 15, 0, 15, 0, 31, 10.0), /*min*/ 0.0});
}

// START: a different yard launch point (west side) over the full map -- varies the entry path
// the DFS takes; confirms a clean run from an alternate start (~46, full-map dilution).
TEST(Integration, BenchmarkMapWestYardStart) {
    runVariant("west-yard r2 full",
        {/*r*/ 2, /*gps*/ 2, P(55, 275, 175), /*h*/ 0, /*cap*/ 50000,
         fullBounds(10.0, 29, 30, 31), /*min*/ 0.0});
}

// START: the drone begins INSIDE floor 2 (above the stairwell slab). It maps floor 2 + roof
// directly and works down through the hole, rather than climbing up to reach them (~89).
TEST(Integration, BenchmarkMapFloor2InteriorStart) {
    runVariant("floor2 interior r2",
        {/*r*/ 2, /*gps*/ 2, P(145, 105, 235), /*h*/ 0, /*cap*/ 150000,
         cellBox(2, 26, 2, 25, 15, 27, 10.0), /*min*/ 75.0});
}

// ============================ Radical / robustness tests ============================
// Each pokes the stack with an abnormal input and asserts it degrades gracefully (no crash,
// valid status, valid-or-sentinel score) rather than corrupting state or throwing out.

// WEIRD BOUNDS: inverted box (max < min). makeOutputMap clamps to a 0-cell map; nothing is
// in-bounds, so the drone finishes immediately and scoring finds no overlap (-1 sentinel).
TEST(Radical, InvertedMissionBounds) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_inv";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(145, 105, 175); s.max_steps = 20000;
    s.bounds = MappingBounds{260.0 * x_extent[cm], 20.0 * x_extent[cm],   // max_x < min_x
                             250.0 * y_extent[cm], 20.0 * y_extent[cm],
                             270.0 * z_extent[cm], 150.0 * z_extent[cm]};
    expectGraceful("inverted-bounds", runSpec(out, s));
    fs::remove_all(out, ec);
}

// WEIRD BOUNDS: a box entirely off the map (far in +x/+y). No overlap with the hidden map, so
// the score is the no-overlap sentinel; the run must still complete cleanly.
TEST(Radical, BoundsDisjointFromMap) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_disjoint";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(555, 555, 105); s.max_steps = 15000;
    s.bounds = cellBox(50, 60, 50, 60, 5, 15, 10.0); // world [500,600]x[500,600]x[50,150]
    const auto report = runSpec(out, s);
    expectGraceful("bounds-disjoint", report);
    EXPECT_LT(report.runs.at(0).mission_score, 0.0) << "disjoint -> -1 sentinel";
    fs::remove_all(out, ec);
}

// WEIRD BOUNDS: a box much larger than the map, with NEGATIVE mins. Output extends into space
// the hidden map doesn't cover (OutOfBounds there); only the overlap is scored -> still > 0.
TEST(Radical, OversizedNegativeBounds) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_oversize";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(145, 105, 175); s.max_steps = 50000;
    s.bounds = cellBox(-5, 34, -5, 34, -5, 34, 10.0); // world [-50,340]^3, encloses the map
    const auto report = runSpec(out, s);
    expectGraceful("oversized-negative-bounds", report);
    EXPECT_GT(report.runs.at(0).mission_score, 0.0);
    fs::remove_all(out, ec);
}

// WEIRD BOUNDS: a single-voxel-thick slab (one z layer). The drone is confined to that layer.
TEST(Radical, ThinSingleLayerBounds) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_thin";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(145, 105, 175); s.max_steps = 60000;
    s.bounds = cellBox(2, 26, 2, 25, 17, 18, 10.0); // z = [170,180), one cell thick
    const auto report = runSpec(out, s);
    expectGraceful("thin-single-layer", report);
    EXPECT_GT(report.runs.at(0).mission_score, 0.0);
    fs::remove_all(out, ec);
}

// LIDAR: a very short beam (z_max = 15cm, ~1.5 cells). The drone can barely see; it should
// still run and map a little rather than break.
TEST(Radical, ShortLidarBeam) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_short";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(145, 105, 175); s.max_steps = 60000;
    s.bounds = cellBox(2, 26, 2, 25, 15, 27, 10.0);
    s.lz_max = 15.0; // very short reach
    expectGraceful("short-lidar-beam", runSpec(out, s));
    fs::remove_all(out, ec);
}

// LIDAR: fov_circles = 0. MockLidar returns no hits, so nothing is mapped; the algorithm must
// still terminate cleanly (no confirmed-Empty neighbors -> finishes) instead of hanging/UB.
TEST(Radical, ZeroFovCircles) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_fov0";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(145, 105, 175); s.max_steps = 60000;
    s.bounds = cellBox(2, 26, 2, 25, 15, 27, 10.0);
    s.fov_circles = 0;
    expectGraceful("fov-circles-0", runSpec(out, s));
    fs::remove_all(out, ec);
}

// LIDAR: a dense, wide FOV (5 circles -> 1+4+16+64+256 = 341 beams PER scan call). Beam count
// grows 4^circle, so this is ~340x the per-step lidar cost of fov=1; the step cap is kept small
// so the suite stays fast. Mainly checks the wide-FOV beam math stays in range and maps.
TEST(Radical, DenseFovCircles) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_fov5";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.start = P(145, 105, 175); s.max_steps = 800; // small: each step does 341 raytraces
    s.bounds = cellBox(2, 26, 2, 25, 15, 27, 10.0);
    s.fov_circles = 5;
    const auto report = runSpec(out, s);
    expectGraceful("fov-circles-5", report);
    EXPECT_GT(report.runs.at(0).mission_score, 0.0);
    fs::remove_all(out, ec);
}

// GPS vs SIZE: a tiny drone (r=2) with a LARGE gps uncertainty (30cm). Clearance = r + gps/2 =
// 17cm, so the cautious footprint dominates the small body. Must stay graceful (likely a low
// score from heavy self-restriction), not wedge or crash.
TEST(Radical, SmallDroneLargeGps) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_smallbiggps";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.radius_cm = 2.0; s.gps_cm = 30.0; s.start = P(145, 105, 175); s.max_steps = 60000;
    s.bounds = cellBox(2, 26, 2, 25, 15, 27, 10.0);
    expectGraceful("small-drone-large-gps", runSpec(out, s));
    fs::remove_all(out, ec);
}

// GPS vs SIZE: the opposite extreme -- a big drone (r=20) with near-perfect gps (0.5cm). The
// body dominates clearance; it explores the open yard over the full map.
TEST(Radical, LargeDroneTinyGps) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_biglowgps";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.radius_cm = 20.0; s.gps_cm = 0.5; s.start = P(225, 275, 185); s.heading_deg = 270;
    s.max_steps = 40000; s.bounds = fullBounds(10.0, 29, 30, 31);
    const auto report = runSpec(out, s);
    expectGraceful("large-drone-tiny-gps", report);
    EXPECT_GT(report.runs.at(0).mission_score, 0.0);
    fs::remove_all(out, ec);
}

// GPS: exactly zero gps_resolution. The rounded GPS view collapses to the exact view (res<=0),
// so the drone observes truth. Must behave like a very-fine-gps run, not divide-by-zero.
TEST(Radical, ZeroGpsResolution) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_gps0";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s; s.gps_cm = 0.0; s.start = P(145, 105, 175); s.max_steps = 80000;
    s.bounds = cellBox(2, 26, 2, 25, 15, 27, 10.0);
    const auto report = runSpec(out, s);
    expectGraceful("zero-gps", report);
    EXPECT_GT(report.runs.at(0).mission_score, 0.0);
    fs::remove_all(out, ec);
}

// NEGATIVE MAP OFFSET: place the hidden map at world origin (-150,-150,-150). The start is
// RELATIVE to the offset (the factory adds it): (145,105,175) + offset = real (-5,-45,25) ->
// hidden cell (14,10,17), an interior floor-1 cell. Bounds are absolute, shifted to the map's
// world location. Correct coordinate handling should map the house as well as the offset-0 run.
TEST(Radical, NegativeMapOffset) {
    const fs::path out = fs::temp_directory_path() / "dm_rad_negoff";
    std::error_code ec; fs::remove_all(out, ec);
    RunSpec s;
    s.off_x = -150; s.off_y = -150; s.off_z = -150;
    s.start = P(145, 105, 175); // relative to the offset -> real (-5,-45,25) = hidden cell (14,10,17)
    s.max_steps = 150000;
    s.bounds = offsetBounds(cellBox(2, 26, 2, 25, 15, 27, 10.0), -150, -150, -150);
    const auto report = runSpec(out, s);
    expectGraceful("negative-map-offset", report);
    EXPECT_GT(report.runs.at(0).mission_score, 70.0) << "negatives must map as well as offset 0";
    fs::remove_all(out, ec);
}

// CONFIG GUARD: gps_resolution (30) > map_resolution (10) is out of spec. The manager must log
// GPS_RES_EXCEEDS_MAP_RES to the error log and RECOVER by clamping gps to the map resolution --
// so instead of freezing after one scan (the raw gps=30 behavior), the drone explores like
// gps=10 and reaches a normal score.
TEST(Radical, GpsExceedsMapResIsGuardedAndClamped) {
    const fs::path out = fs::temp_directory_path() / "dm_gps_guard";
    std::error_code ec; fs::remove_all(out, ec);

    RunSpec s; s.gps_cm = 30.0; s.res_cm = 10.0; s.start = P(145, 105, 175); s.max_steps = 40000;
    s.bounds = cellBox(2, 26, 2, 25, 15, 27, 10.0);
    const auto r = runSpec(out, s).runs.at(0);

    // Recovered: clamped run maps like gps=10 (~89 over 40k steps), not the frozen 80.8 @ 181.
    ASSERT_FALSE(r.mission_results.empty());
    EXPECT_NE(r.mission_results.at(0).status, MissionRunStatus::Error);
    EXPECT_GT(r.mission_score, 85.0);
    EXPECT_GT(r.mission_results.at(0).steps, 1000u); // it actually moved, didn't freeze

    // Guarded: the descriptive error was written immediately to the error log.
    std::ifstream log(out / "output_results" / "errors.txt");
    std::stringstream buf; buf << log.rdbuf();
    EXPECT_NE(buf.str().find("GPS_RES_EXCEEDS_MAP_RES"), std::string::npos) << buf.str();

    fs::remove_all(out, ec);
}
