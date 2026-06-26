// SimulationManager orchestration, isolated with a GMock'd factory.
#include <drone_mapper/CompositionParser.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MissionTypes.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <yaml-cpp/yaml.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using ::testing::_;
using ::testing::Eq;
using ::testing::Field;
using ::testing::Return;
using ::testing::Throw;
namespace fs = std::filesystem;

class MockRun : public ISimulationRun {
public:
    MOCK_METHOD(SimulationResult, run, (), (override));
};
class MockFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>, create,
                (const SimulationConfigData& simulation, const MissionConfigData& mission,
                 const DroneConfigData& drone, const LidarConfigData& lidar,
                 const std::filesystem::path& output_path),
                (override));
};

SimulationConfigData simCfg(const char* map) {
    SimulationConfigData s;
    s.map_filename = map;
    s.map_resolution = 10.0 * cm;
    return s;
}
MissionConfigData missionCfg() {
    MissionConfigData m;
    m.max_steps = 10;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}
DroneConfigData droneCfg() {
    DroneConfigData d;
    d.radius = 5.0 * cm;
    return d;
}
LidarConfigData lidarCfg() {
    LidarConfigData l;
    l.z_min = 10.0 * cm;
    l.z_max = 100.0 * cm;
    l.d = 10.0 * cm;
    l.fov_circles = 1;
    return l;
}
SimulationResult okResult(double score = 90.0) {
    SimulationResult r;
    r.mission_score = score;
    r.resolution_request_status = ResolutionRequestStatus::Accepted;
    r.mission_results.push_back(MissionRunResult{MissionRunStatus::Completed, 5, {}});
    return r;
}
SimulationResult errorResult() {
    SimulationResult r;
    r.mission_score = -1.0;
    r.resolution_request_status = ResolutionRequestStatus::Accepted;
    r.mission_results.push_back(
        MissionRunResult{MissionRunStatus::Error, 1, {ErrorRef{"DRONE_HITS_OBSTACLE", "crash"}}});
    return r;
}
// A mock run programmed to return `result` once.
std::unique_ptr<ISimulationRun> runReturning(const SimulationResult& result) {
    auto r = std::make_unique<MockRun>();
    EXPECT_CALL(*r, run()).WillOnce(Return(result));
    return r;
}
fs::path freshDir(const char* name) {
    const fs::path dir = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(dir, ec);
    return dir;
}
} // namespace

// PDF's grouped composition expands to 20 runs: (2 + 3) missions * 2 drones * 2 lidars.
TEST(SimulationManager, ExpandsGroupedCompositionTo20Runs) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {
        {simCfg("a.npy"), {missionCfg(), missionCfg()}},                  // 2 missions
        {simCfg("b.npy"), {missionCfg(), missionCfg(), missionCfg()}},    // 3 missions
    };
    comp.drones = {droneCfg(), droneCfg()};
    comp.lidars = {lidarCfg(), lidarCfg()};

    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(20)
        .WillRepeatedly([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                           const LidarConfigData&, const fs::path&) -> std::unique_ptr<ISimulationRun> {
            return runReturning(okResult());
        });

    SimulationManager mgr(std::move(factory));
    const fs::path out = freshDir("dm_mgr_20");
    const auto report = mgr.run(comp, out);
    EXPECT_EQ(report.runs.size(), 20u);
    std::error_code ec;
    fs::remove_all(out, ec);
}

// One run errors -> that run scores -1 and the loop continues (all 4 produced).
TEST(SimulationManager, OneRunErrorScoresMinus1AndLoopContinues) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{simCfg("a.npy"), {missionCfg()}}};
    comp.drones = {droneCfg(), droneCfg()};
    comp.lidars = {lidarCfg(), lidarCfg()}; // 1 * 2 * 2 = 4 runs

    auto factory = std::make_unique<MockFactory>();
    int call = 0;
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(4)
        .WillRepeatedly([&call](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                                const LidarConfigData&, const fs::path&) -> std::unique_ptr<ISimulationRun> {
            const bool is_error = (call == 1); // second run errors
            ++call;
            return runReturning(is_error ? errorResult() : okResult());
        });

    SimulationManager mgr(std::move(factory));
    const fs::path out = freshDir("dm_mgr_err");
    const auto report = mgr.run(comp, out);
    ASSERT_EQ(report.runs.size(), 4u);
    int minus1 = 0;
    for (const auto& r : report.runs) {
        if (r.mission_score < 0.0) ++minus1;
    }
    EXPECT_EQ(minus1, 1);
    std::error_code ec;
    fs::remove_all(out, ec);
}

// A whole group fails (shared map won't load -> create() throws) -> all that group's runs
// score -1 and the batch continues with the good group.
TEST(SimulationManager, WholeGroupFailureScoresMinus1ForAllRunsInGroup) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {
        {simCfg("bad.npy"), {missionCfg()}},
        {simCfg("good.npy"), {missionCfg()}},
    };
    comp.drones = {droneCfg(), droneCfg()};
    comp.lidars = {lidarCfg(), lidarCfg()}; // each group: 1 * 2 * 2 = 4 runs -> 8 total

    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory,
                create(Field(&SimulationConfigData::map_filename, Eq(fs::path("bad.npy"))), _, _, _, _))
        .Times(4)
        .WillRepeatedly(Throw(std::runtime_error("cannot load bad.npy")));
    EXPECT_CALL(*factory,
                create(Field(&SimulationConfigData::map_filename, Eq(fs::path("good.npy"))), _, _, _, _))
        .Times(4)
        .WillRepeatedly([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                           const LidarConfigData&, const fs::path&) -> std::unique_ptr<ISimulationRun> {
            return runReturning(okResult());
        });

    SimulationManager mgr(std::move(factory));
    const fs::path out = freshDir("dm_mgr_group");
    const auto report = mgr.run(comp, out);
    ASSERT_EQ(report.runs.size(), 8u);
    int minus1 = 0;
    for (const auto& r : report.runs) {
        if (r.mission_score < 0.0) ++minus1;
    }
    EXPECT_EQ(minus1, 4); // exactly the bad group's runs
    std::error_code ec;
    fs::remove_all(out, ec);
}

// simulation_output.yaml carries per-run scores and resolution status.
TEST(SimulationManager, WritesSimulationOutputYamlWithScoresAndStatus) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{simCfg("a.npy"), {missionCfg()}}};
    comp.drones = {droneCfg()};
    comp.lidars = {lidarCfg()}; // 1 run

    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(1)
        .WillOnce([](const SimulationConfigData&, const MissionConfigData&, const DroneConfigData&,
                     const LidarConfigData&, const fs::path&) -> std::unique_ptr<ISimulationRun> {
            return runReturning(okResult(87.5));
        });

    SimulationManager mgr(std::move(factory));
    const fs::path out = freshDir("dm_mgr_yaml");
    (void)mgr.run(comp, out);

    const fs::path yaml = out / "simulation_output.yaml";
    ASSERT_TRUE(fs::exists(yaml));
    const YAML::Node root = YAML::LoadFile(yaml.string());
    ASSERT_TRUE(root["runs"]);
    ASSERT_EQ(root["runs"].size(), 1u);
    EXPECT_DOUBLE_EQ(root["runs"][0]["score"].as<double>(), 87.5);
    EXPECT_EQ(root["runs"][0]["resolution_status"].as<std::string>(), "Accepted");
    std::error_code ec;
    fs::remove_all(out, ec);
}

// CLI path resolution: missing -> simulation.yaml; relative stays relative; absolute as-is.
TEST(SimulationManager, CliPathResolution) {
    EXPECT_EQ(resolveCompositionPath(std::nullopt), fs::path("simulation.yaml"));
    EXPECT_EQ(resolveCompositionPath(std::optional<std::string>("runs/foo.yaml")), fs::path("runs/foo.yaml"));
    EXPECT_FALSE(resolveCompositionPath(std::optional<std::string>("runs/foo.yaml")).is_absolute());
    EXPECT_EQ(resolveCompositionPath(std::optional<std::string>("/abs/foo.yaml")), fs::path("/abs/foo.yaml"));
    EXPECT_TRUE(resolveCompositionPath(std::optional<std::string>("/abs/foo.yaml")).is_absolute());
}
