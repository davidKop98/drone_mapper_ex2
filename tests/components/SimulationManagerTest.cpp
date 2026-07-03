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
#include <fstream>
#include <iterator>
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

// ---- Survivor-killing additions (mutation campaign) --------------------------------

// Exact per-run values, summary convention, and failure detail, all re-read from the
// report AND the written yaml: runs {90 (5 steps), 80 (7 steps), create-throws -> -1}.
// score_range EXCLUDES failed (-1) runs by convention -> exactly (80, 90). The failed
// run still carries a mission result with the RUN_CREATE_FAILED error detail.
TEST(SimulationManager, ReportCarriesExactValuesSummaryAndFailureDetail) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{simCfg("a.npy"), {missionCfg()}}};
    comp.drones = {droneCfg(), droneCfg(), droneCfg()}; // 3 runs
    comp.lidars = {lidarCfg()};

    auto factory = std::make_unique<MockFactory>();
    int call = 0;
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(3)
        .WillRepeatedly([&call](const SimulationConfigData&, const MissionConfigData&,
                                const DroneConfigData&, const LidarConfigData&,
                                const fs::path&) -> std::unique_ptr<ISimulationRun> {
            const int i = call++;
            if (i == 2) throw std::runtime_error("cannot load a.npy");
            SimulationResult r;
            r.mission_score = (i == 0) ? 90.0 : 80.0;
            r.resolution_request_status = ResolutionRequestStatus::Accepted;
            r.mission_results.push_back(MissionRunResult{
                MissionRunStatus::Completed, static_cast<std::size_t>(i == 0 ? 5 : 7), {}});
            return runReturning(r);
        });

    SimulationManager mgr(std::move(factory));
    const fs::path out = freshDir("dm_mgr_exact");
    const auto report = mgr.run(comp, out);

    ASSERT_EQ(report.runs.size(), 3u);
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 80.0); // -1 excluded from range
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 90.0);
    ASSERT_FALSE(report.runs[2].mission_results.empty()); // failed run keeps its detail
    EXPECT_EQ(report.runs[2].mission_results[0].status, MissionRunStatus::Error);
    ASSERT_FALSE(report.runs[2].mission_results[0].errors.empty());
    EXPECT_EQ(report.runs[2].mission_results[0].errors[0].code, "RUN_CREATE_FAILED");

    const YAML::Node root = YAML::LoadFile((out / "simulation_output.yaml").string());
    ASSERT_EQ(root["runs"].size(), 3u);
    EXPECT_EQ(root["runs"][0]["missions"][0]["steps"].as<std::size_t>(), 5u);
    EXPECT_EQ(root["runs"][1]["missions"][0]["steps"].as<std::size_t>(), 7u);
    EXPECT_DOUBLE_EQ(root["score_range"]["min"].as<double>(), 80.0);
    EXPECT_DOUBLE_EQ(root["score_range"]["max"].as<double>(), 90.0);
    EXPECT_EQ(root["runs"][2]["missions"][0]["errors"][0]["code"].as<std::string>(),
              "RUN_CREATE_FAILED");
    std::error_code ec;
    fs::remove_all(out, ec);
}

// errors.txt is written IMMEDIATELY (flushed) with code AND message: the second run's
// create() reads the log while the batch is still running and must already see the
// first run's failure -- a deferred/buffered log or a detail-less line fails here.
TEST(SimulationManager, ErrorsTxtImmediateWithCodeAndMessage) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{simCfg("a.npy"), {missionCfg()}}};
    comp.drones = {droneCfg(), droneCfg()}; // 2 runs
    comp.lidars = {lidarCfg()};

    const fs::path out = freshDir("dm_mgr_log");
    const fs::path log = out / "output_results" / "errors.txt";
    std::string seen_mid_batch;
    auto factory = std::make_unique<MockFactory>();
    int call = 0;
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly([&](const SimulationConfigData&, const MissionConfigData&,
                            const DroneConfigData&, const LidarConfigData&,
                            const fs::path&) -> std::unique_ptr<ISimulationRun> {
            if (call++ == 0) throw std::runtime_error("distinctive-load-failure");
            std::ifstream f(log); // mid-batch: run 0 already failed, batch still running
            seen_mid_batch.assign(std::istreambuf_iterator<char>(f),
                                  std::istreambuf_iterator<char>());
            return runReturning(okResult());
        });

    SimulationManager mgr(std::move(factory));
    (void)mgr.run(comp, out);

    EXPECT_NE(seen_mid_batch.find("RUN_CREATE_FAILED"), std::string::npos) << seen_mid_batch;
    EXPECT_NE(seen_mid_batch.find("distinctive-load-failure"), std::string::npos) << seen_mid_batch;
    std::error_code ec;
    fs::remove_all(out, ec);
}

// Every run gets its OWN numbered output directory: capture the paths handed to
// create() and require run_0 / run_1, distinct, and existing on disk.
TEST(SimulationManager, PerRunOutputDirsAreDistinct) {
    SimulationCompositionData comp;
    comp.simulation_mission_groups = {{simCfg("a.npy"), {missionCfg()}}};
    comp.drones = {droneCfg(), droneCfg()};
    comp.lidars = {lidarCfg()}; // 2 runs

    std::vector<fs::path> run_dirs;
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly([&](const SimulationConfigData&, const MissionConfigData&,
                            const DroneConfigData&, const LidarConfigData&,
                            const fs::path& p) -> std::unique_ptr<ISimulationRun> {
            run_dirs.push_back(p);
            return runReturning(okResult());
        });

    SimulationManager mgr(std::move(factory));
    const fs::path out = freshDir("dm_mgr_dirs");
    (void)mgr.run(comp, out);

    ASSERT_EQ(run_dirs.size(), 2u);
    EXPECT_NE(run_dirs[0], run_dirs[1]); // a collision would overwrite one run's map
    EXPECT_EQ(run_dirs[0].filename(), "run_0");
    EXPECT_EQ(run_dirs[1].filename(), "run_1");
    EXPECT_TRUE(fs::exists(run_dirs[0]));
    EXPECT_TRUE(fs::exists(run_dirs[1]));
    std::error_code ec;
    fs::remove_all(out, ec);
}

// The parser reads every field from ITS OWN key -- all values distinct so a crossed
// key cannot coincide -- and dimensions_cm is a DIAMETER (radius = half).
TEST(SimulationManager, ParserReadsFieldsFromCorrectKeys) {
    const fs::path dir = freshDir("dm_parse_fields");
    std::error_code ec;
    fs::create_directories(dir, ec);
    const auto write = [&](const char* name, const std::string& body) {
        std::ofstream f(dir / name);
        f << body;
    };
    write("compose.yaml",
          "simulation_compositions:\n"
          "  simulations:\n"
          "    - simulation_config: \"sim.yaml\"\n"
          "      mission_configs: [\"mission.yaml\"]\n"
          "  drone_configs: [\"drone.yaml\"]\n"
          "  lidar_configs: [\"lidar.yaml\"]\n");
    write("sim.yaml", "simulation_config:\n  map_filename: \"m.npy\"\n  map_resolution_cm: 7\n");
    write("mission.yaml", "mission_config:\n  max_steps: 123\n  gps_resolution_cm: 4\n");
    write("drone.yaml",
          "drone_config:\n  dimensions_cm: 10\n  max_rotate_deg: 11\n"
          "  max_advance_cm: 22\n  max_elevate_cm: 33\n");
    write("lidar.yaml",
          "lidar_config:\n  z_min_cm: 6\n  z_max_cm: 300\n  d_cm: 2\n  fov_circles: 3\n");

    const auto comp = parseCompositionYaml(dir / "compose.yaml");
    ASSERT_EQ(comp.simulation_mission_groups.size(), 1u);
    ASSERT_EQ(comp.drones.size(), 1u);
    ASSERT_EQ(comp.lidars.size(), 1u);
    EXPECT_DOUBLE_EQ(
        std::get<0>(comp.simulation_mission_groups[0]).map_resolution.force_numerical_value_in(cm), 7.0);
    EXPECT_EQ(std::get<1>(comp.simulation_mission_groups[0])[0].max_steps, 123u);
    EXPECT_DOUBLE_EQ(
        std::get<1>(comp.simulation_mission_groups[0])[0].gps_resolution.force_numerical_value_in(cm), 4.0);
    EXPECT_DOUBLE_EQ(comp.drones[0].radius.force_numerical_value_in(cm), 5.0); // diameter 10
    EXPECT_DOUBLE_EQ(comp.drones[0].max_rotate.force_numerical_value_in(deg), 11.0);
    EXPECT_DOUBLE_EQ(comp.drones[0].max_advance.force_numerical_value_in(cm), 22.0);
    EXPECT_DOUBLE_EQ(comp.drones[0].max_elevate.force_numerical_value_in(cm), 33.0);
    EXPECT_DOUBLE_EQ(comp.lidars[0].z_min.force_numerical_value_in(cm), 6.0);
    EXPECT_DOUBLE_EQ(comp.lidars[0].d.force_numerical_value_in(cm), 2.0);
    EXPECT_EQ(comp.lidars[0].fov_circles, 3u);
    fs::remove_all(dir, ec);
}
