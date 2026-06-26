// Integration suite (a): the FULL real stack with the real MappingAlgorithmImpl, driven
// through SimulationManager::run() on a parsed composition. Assertions pin cross-component
// invariants (a scanned wall lands Occupied; a bad run scores -1 yet the batch survives;
// the cartesian count and per-run outputs match), not merely "didn't crash".
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <system_error>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
namespace fs = std::filesystem;

Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
MappingBounds box(double lo, double hi) {
    return MappingBounds{lo * x_extent[cm], hi * x_extent[cm], lo * y_extent[cm],
                         hi * y_extent[cm], lo * z_extent[cm], hi * z_extent[cm]};
}

// A 5x5x5 "room": outer shell Occupied, interior Empty, written to a temp .npy.
fs::path writeRoomMap(const char* name) {
    constexpr std::size_t n = 5;
    auto arr = std::make_shared<NpyArray>(NpyArray::shape_t{n, n, n}, sizeof(std::uint8_t), 'u', false);
    arr->Allocate();
    std::uint8_t* data = arr->Data<std::uint8_t>();
    for (std::size_t x = 0; x < n; ++x) {
        for (std::size_t y = 0; y < n; ++y) {
            for (std::size_t z = 0; z < n; ++z) {
                const bool shell = (x == 0 || x == n - 1 || y == 0 || y == n - 1 || z == 0 || z == n - 1);
                data[x * n * n + y * n + z] = shell ? 1 : 0;
            }
        }
    }
    const fs::path path = fs::temp_directory_path() / name;
    EXPECT_EQ(arr->SaveNPY(path.string()), nullptr);
    return path;
}

// Load a saved output_map.npy with the box-aligned config (offset 0, the given resolution).
std::unique_ptr<Map3DImpl> loadOutputMap(const fs::path& path, double res, std::size_t n) {
    auto arr = std::make_shared<NpyArray>();
    EXPECT_EQ(arr->LoadNPY(path.string()), nullptr);
    MapConfig cfg;
    cfg.offset = P(0, 0, 0);
    cfg.resolution = res * cm;
    cfg.boundaries = box(0, static_cast<double>(n) * res);
    return std::make_unique<Map3DImpl>(arr, cfg);
}

SimulationConfigData roomSim(const fs::path& map) {
    SimulationConfigData s;
    s.map_filename = map;
    s.map_resolution = 10.0 * cm;
    s.map_offset = P(0, 0, 0);
    s.initial_drone_position = P(25, 25, 25);
    s.initial_angle = 0.0 * horizontal_angle[deg];
    return s;
}
MissionConfigData roomMission() {
    MissionConfigData m;
    m.max_steps = 100000;
    m.gps_resolution = 2.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    m.mission_bounds = box(0, 50);
    return m;
}
DroneConfigData smallDrone(double max_advance = 10.0) {
    DroneConfigData d;
    d.radius = 2.0 * cm;
    d.max_rotate = 45.0 * horizontal_angle[deg];
    d.max_advance = max_advance * cm;
    d.max_elevate = 10.0 * cm;
    return d;
}
LidarConfigData roomLidar(double z_max = 60.0) {
    LidarConfigData l;
    l.z_min = 5.0 * cm;
    l.z_max = z_max * cm;
    l.d = 5.0 * cm;
    l.fov_circles = 1;
    return l;
}
fs::path freshDir(const char* name) {
    const fs::path dir = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(dir, ec);
    return dir;
}
} // namespace

// Single real run: sane score, outputs on disk, and the cross-component invariant that a
// scanned wall lands Occupied (and the drone's own cell Empty) in the saved output map.
TEST(Integration, RealStackScoresAndScannedWallLandsOccupied) {
    const fs::path map = writeRoomMap("dm_int_room.npy");
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{roomSim(map), {roomMission()}}};
    comp.drones = {smallDrone()};
    comp.lidars = {roomLidar()};

    const fs::path out = freshDir("dm_int_single");
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager mgr(std::move(factory));
    const auto report = mgr.run(comp, out);

    ASSERT_EQ(report.runs.size(), 1u);
    const SimulationResult& result = report.runs.at(0);
    EXPECT_GT(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
    ASSERT_FALSE(result.mission_results.empty());
    std::cerr << "[int] real score=" << result.mission_score
              << " steps=" << result.mission_results.at(0).steps << '\n';

    const fs::path output_map = out / "output_results" / "run_0" / "output_map.npy";
    ASSERT_TRUE(fs::exists(out / "simulation_output.yaml"));
    ASSERT_TRUE(fs::exists(output_map));

    auto saved = loadOutputMap(output_map, 10.0, 5);
    EXPECT_EQ(saved->atVoxel(P(45, 25, 25)), VoxelOccupancy::Occupied); // +x shell wall, scanned
    EXPECT_EQ(saved->atVoxel(P(25, 25, 25)), VoxelOccupancy::Empty);    // the drone's own cell

    std::error_code ec;
    fs::remove(map, ec);
    fs::remove_all(out, ec);
}

// Multi-run cartesian product (2 drones x 2 lidars = 4 runs): the count matches, every run
// scores in (0,100], and each run's output map is written under run_0..run_3.
TEST(Integration, RealStackMultiRunAggregationAndPerRunOutputs) {
    const fs::path map = writeRoomMap("dm_int_room_multi.npy");
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{roomSim(map), {roomMission()}}};
    comp.drones = {smallDrone(10.0), smallDrone(8.0)};
    comp.lidars = {roomLidar(60.0), roomLidar(50.0)};

    const fs::path out = freshDir("dm_int_multi");
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager mgr(std::move(factory));
    const auto report = mgr.run(comp, out);

    ASSERT_EQ(report.runs.size(), 4u); // 1 mission * 2 drones * 2 lidars
    for (const SimulationResult& r : report.runs) {
        EXPECT_GT(r.mission_score, 0.0);
        EXPECT_LE(r.mission_score, 100.0);
    }
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(fs::exists(out / "output_results" / ("run_" + std::to_string(i)) / "output_map.npy"))
            << "missing output map for run " << i;
    }

    const YAML::Node root = YAML::LoadFile((out / "simulation_output.yaml").string());
    ASSERT_TRUE(root["runs"]);
    EXPECT_EQ(root["runs"].size(), 4u);
    EXPECT_TRUE(root["runs"][0]["resolution_status"]);

    std::error_code ec;
    fs::remove(map, ec);
    fs::remove_all(out, ec);
}

// Error path: a run whose hidden map won't load scores -1, the batch continues to the good
// run, and the error is logged immediately. (The isolation invariant, integration-level.)
TEST(Integration, RealStackBadMapScoresMinus1AndBatchContinues) {
    const fs::path good_map = writeRoomMap("dm_int_room_good.npy");
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {
        {roomSim("does_not_exist.npy"), {roomMission()}}, // run 0: bad map
        {roomSim(good_map), {roomMission()}},             // run 1: good map
    };
    comp.drones = {smallDrone()};
    comp.lidars = {roomLidar()};

    const fs::path out = freshDir("dm_int_badmap");
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager mgr(std::move(factory));
    const auto report = mgr.run(comp, out);

    ASSERT_EQ(report.runs.size(), 2u);              // batch did not abort on the bad run
    EXPECT_LT(report.runs.at(0).mission_score, 0.0); // bad run scored -1
    EXPECT_GT(report.runs.at(1).mission_score, 0.0); // good run completed
    EXPECT_LE(report.runs.at(1).mission_score, 100.0);

    const fs::path log = out / "output_results" / "errors.txt";
    ASSERT_TRUE(fs::exists(log));
    std::ifstream log_in(log);
    std::stringstream buffer;
    buffer << log_in.rdbuf();
    EXPECT_NE(buffer.str().find("RUN_CREATE_FAILED"), std::string::npos); // error was logged

    std::error_code ec;
    fs::remove(good_map, ec);
    fs::remove_all(out, ec);
}
