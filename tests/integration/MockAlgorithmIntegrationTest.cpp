// Integration suite (b): the full real harness, but with a GMock IMappingAlgorithm that
// scripts a short deterministic sequence. This isolates the pipeline (movement -> scan ->
// applyToMap -> score -> output) from the real "brain", so a bug in one cannot mask a bug
// in the other. A custom factory injects the scripted algorithm while wiring everything
// else for real (reusing SimulationRunFactoryImpl's construction helpers).
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <system_error>
#include <utility>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using ::testing::_;
using ::testing::Return;
namespace fs = std::filesystem;

Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
Orientation O(double h, double a) {
    return Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
}
MappingBounds box(double lo, double hi) {
    return MappingBounds{lo * x_extent[cm], hi * x_extent[cm], lo * y_extent[cm],
                         hi * y_extent[cm], lo * z_extent[cm], hi * z_extent[cm]};
}
MovementCommand advanceCmd(double cm_) {
    MovementCommand c{};
    c.type = MovementCommandType::Advance;
    c.distance = cm_ * cm;
    return c;
}
MappingStepCommand moveScan(MovementCommand move, Orientation scan) {
    return MappingStepCommand{move, scan, AlgorithmStatus::Working};
}
MappingStepCommand moveOnly(MovementCommand move) {
    return MappingStepCommand{move, std::nullopt, AlgorithmStatus::Working};
}
MappingStepCommand finishedCmd() {
    return MappingStepCommand{std::nullopt, std::nullopt, AlgorithmStatus::Finished};
}

class ScriptedAlgo : public IMappingAlgorithm {
public:
    ScriptedAlgo(const MissionConfigData& mission, const LidarConfigData& lidar,
                 const DroneConfigData& drone, const IMap3D& output_map)
        : IMappingAlgorithm(mission, lidar, drone, output_map) {}
    MOCK_METHOD(MappingStepCommand, nextStep, (const DroneState&, const LidarScanResult*), (override));
};

// Builds the real run graph but injects a scripted ScriptedAlgo (set up by `script`).
class ScriptedAlgoFactory : public ISimulationRunFactory {
public:
    explicit ScriptedAlgoFactory(std::function<void(ScriptedAlgo&)> script) : script_(std::move(script)) {}

    std::unique_ptr<ISimulationRun> create(const SimulationConfigData& sim,
                                           const MissionConfigData& mission,
                                           const DroneConfigData& drone,
                                           const LidarConfigData& lidar,
                                           const std::filesystem::path& output_path) override {
        auto hidden = SimulationRunFactoryImpl::loadHiddenMap(sim);
        const auto resolution = SimulationRunFactoryImpl::resolveOutputResolution(mission, sim.map_resolution);
        auto output = SimulationRunFactoryImpl::makeOutputMap(mission.mission_bounds, resolution.resolution);

        auto exact_gps = std::make_unique<MockGPS>(
            sim.initial_drone_position, Orientation{sim.initial_angle, 0.0 * altitude_angle[deg]}, 0.0 * cm);
        auto rounded_gps = std::make_unique<MockGPS>(exact_gps->truth(), mission.gps_resolution);
        auto movement = std::make_unique<MockMovement>(*exact_gps, *hidden, drone.radius);
        auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden, *exact_gps);

        auto algo = std::make_unique<ScriptedAlgo>(mission, lidar, drone, *output);
        script_(*algo); // install the scripted nextStep sequence

        auto drone_control = std::make_unique<DroneControlImpl>(
            lidar, *lidar_impl, *rounded_gps, *exact_gps, *movement, *output, *algo);
        const fs::path output_map_file = output_path / "output_map.npy";
        auto mission_control = std::make_unique<MissionControlImpl>(
            mission, drone, *hidden, *output, *drone_control, output_map_file);

        return std::make_unique<SimulationRunImpl>(
            std::move(hidden), std::move(output), std::move(exact_gps), std::move(rounded_gps),
            std::move(movement), std::move(lidar_impl), std::move(algo), std::move(drone_control),
            std::move(mission_control), sim, mission, output_map_file, resolution.status);
    }

private:
    std::function<void(ScriptedAlgo&)> script_;
};

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
std::unique_ptr<Map3DImpl> loadOutputMap(const fs::path& path) {
    auto arr = std::make_shared<NpyArray>();
    EXPECT_EQ(arr->LoadNPY(path.string()), nullptr);
    MapConfig cfg;
    cfg.offset = P(0, 0, 0);
    cfg.resolution = 10.0 * cm;
    cfg.boundaries = box(0, 50);
    return std::make_unique<Map3DImpl>(arr, cfg);
}
SimulationCompositionData roomComposition(const fs::path& map) {
    SimulationConfigData sim;
    sim.map_filename = map;
    sim.map_resolution = 10.0 * cm;
    sim.map_offset = P(0, 0, 0);
    sim.initial_drone_position = P(25, 25, 25);
    sim.initial_angle = 0.0 * horizontal_angle[deg];
    MissionConfigData mission;
    mission.max_steps = 100;
    mission.gps_resolution = 2.0 * cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = box(0, 50);
    DroneConfigData drone;
    drone.radius = 2.0 * cm;
    drone.max_rotate = 45.0 * horizontal_angle[deg];
    drone.max_advance = 30.0 * cm;
    drone.max_elevate = 10.0 * cm;
    LidarConfigData lidar;
    lidar.z_min = 5.0 * cm;
    lidar.z_max = 60.0 * cm;
    lidar.d = 5.0 * cm;
    lidar.fov_circles = 1;
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{sim, {mission}}};
    comp.drones = {drone};
    comp.lidars = {lidar};
    return comp;
}
fs::path freshDir(const char* name) {
    const fs::path dir = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(dir, ec);
    return dir;
}
} // namespace

// Scripted move + scan: the drone advances +x then scans +x; the harness must place the
// scanned +x wall Occupied (and the post-move origin cell Empty) in the saved output map,
// and a report is written.
TEST(Integration, MockAlgorithmScriptedRunMapsScannedWall) {
    const fs::path map = writeRoomMap("dm_mockalgo_room.npy");
    auto script = [](ScriptedAlgo& algo) {
        EXPECT_CALL(algo, nextStep(_, _))
            .WillOnce(Return(moveScan(advanceCmd(10), O(0, 0)))) // move +x 10, then scan +x
            .WillOnce(Return(finishedCmd()));
    };

    const fs::path out = freshDir("dm_mockalgo_good");
    SimulationManager mgr(std::make_unique<ScriptedAlgoFactory>(script));
    const auto report = mgr.run(roomComposition(map), out);

    ASSERT_EQ(report.runs.size(), 1u);
    ASSERT_FALSE(report.runs.at(0).mission_results.empty());
    EXPECT_EQ(report.runs.at(0).mission_results.at(0).status, MissionRunStatus::Completed);

    const fs::path output_map = out / "output_results" / "run_0" / "output_map.npy";
    ASSERT_TRUE(fs::exists(out / "simulation_output.yaml"));
    ASSERT_TRUE(fs::exists(output_map));

    // Drone moved 25 -> 35; the +x beam from x=35 hits the shell at cell 4.
    auto saved = loadOutputMap(output_map);
    EXPECT_EQ(saved->atVoxel(P(45, 25, 25)), VoxelOccupancy::Occupied); // scanned wall
    EXPECT_EQ(saved->atVoxel(P(35, 25, 25)), VoxelOccupancy::Empty);    // post-move origin cell

    std::error_code ec;
    fs::remove(map, ec);
    fs::remove_all(out, ec);
}

// Scripted crash: the drone is told to advance straight into the +x wall -> MockMovement
// reports DRONE_HITS_OBSTACLE -> the run scores -1 and the error is logged immediately.
TEST(Integration, MockAlgorithmCrashScoresMinus1AndLogs) {
    const fs::path map = writeRoomMap("dm_mockalgo_crash.npy");
    auto script = [](ScriptedAlgo& algo) {
        EXPECT_CALL(algo, nextStep(_, _))
            .WillOnce(Return(moveOnly(advanceCmd(30)))); // +x 30 from x=25 -> through the wall
    };

    const fs::path out = freshDir("dm_mockalgo_crash");
    SimulationManager mgr(std::make_unique<ScriptedAlgoFactory>(script));
    const auto report = mgr.run(roomComposition(map), out);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_LT(report.runs.at(0).mission_score, 0.0); // -1
    ASSERT_FALSE(report.runs.at(0).mission_results.empty());
    EXPECT_EQ(report.runs.at(0).mission_results.at(0).status, MissionRunStatus::Error);

    const fs::path log = out / "output_results" / "errors.txt";
    ASSERT_TRUE(fs::exists(log));
    std::ifstream log_in(log);
    std::stringstream buffer;
    buffer << log_in.rdbuf();
    EXPECT_NE(buffer.str().find("DRONE_HITS_OBSTACLE"), std::string::npos);

    std::error_code ec;
    fs::remove(map, ec);
    fs::remove_all(out, ec);
}
