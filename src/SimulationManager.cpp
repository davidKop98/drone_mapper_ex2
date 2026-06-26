#include <drone_mapper/SimulationManager.h>

#include <drone_mapper/ISimulationRun.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <ctime>
#include <exception>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

namespace fs = std::filesystem;

std::string nowUtc() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
#if defined(_WIN32)
    gmtime_s(&tm_buf, &now);
#else
    gmtime_r(&now, &tm_buf);
#endif
    char buffer[32] = {};
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_buf);
    return buffer;
}

std::string resolutionStatusName(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "Accepted";
    case types::ResolutionRequestStatus::Ignored:
        return "Ignored";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IgnoredTooSmall";
    }
    return "Unknown";
}

std::string missionStatusName(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed:
        return "Completed";
    case types::MissionRunStatus::MaxSteps:
        return "MaxSteps";
    case types::MissionRunStatus::Error:
        return "Error";
    }
    return "Unknown";
}

// Write a single error to the log immediately (flushed), not batched at the end.
void logError(std::ofstream& log, std::size_t run_index, const types::ErrorRef& error) {
    if (log) {
        log << "run " << run_index << ": [" << error.code << "] " << error.message << '\n';
        log.flush();
    }
}

std::tuple<double, double> scoreRange(const std::vector<types::SimulationResult>& runs) {
    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    bool any = false;
    for (const types::SimulationResult& run : runs) {
        if (run.mission_score < 0.0) continue; // skip failed (-1) runs
        any = true;
        lo = std::min(lo, run.mission_score);
        hi = std::max(hi, run.mission_score);
    }
    return any ? std::tuple<double, double>{lo, hi} : std::tuple<double, double>{-1.0, -1.0};
}

void writeReportYaml(const fs::path& path, const types::SimulationManagerReport& report) {
    YAML::Node root;
    root["metric"] = report.metric;
    root["generated_at_utc"] = report.generated_at_utc;
    root["error_score"] = report.error_score;
    root["score_range"]["min"] = std::get<0>(report.score_range);
    root["score_range"]["max"] = std::get<1>(report.score_range);

    for (const types::SimulationResult& run : report.runs) {
        YAML::Node run_node;
        run_node["map_filename"] = run.simulation_config.map_filename.string();
        run_node["score"] = run.mission_score;
        run_node["resolution_status"] = resolutionStatusName(run.resolution_request_status);
        run_node["output_map_file"] = run.output_map_file.string();
        for (const types::MissionRunResult& mission : run.mission_results) {
            YAML::Node mission_node;
            mission_node["status"] = missionStatusName(mission.status);
            mission_node["steps"] = mission.steps;
            for (const types::ErrorRef& error : mission.errors) {
                YAML::Node error_node;
                error_node["code"] = error.code;
                error_node["message"] = error.message;
                mission_node["errors"].push_back(error_node);
            }
            run_node["missions"].push_back(mission_node);
        }
        root["runs"].push_back(run_node);
    }

    std::ofstream out(path, std::ios::trunc);
    if (out) {
        out << root;
    }
}

// Run one (sim, mission, drone, lidar) combination. Any failure to even construct the run
// (e.g. a shared map file that won't load) scores -1 for this combination and is logged;
// the batch continues.
types::SimulationResult runOne(ISimulationRunFactory& factory,
                               const types::SimulationConfigData& sim,
                               const types::MissionConfigData& mission,
                               const types::DroneConfigData& drone,
                               const types::LidarConfigData& lidar,
                               const fs::path& run_output_path,
                               std::ofstream& error_log,
                               std::size_t run_index) {
    try {
        std::unique_ptr<ISimulationRun> run = factory.create(sim, mission, drone, lidar, run_output_path);
        types::SimulationResult result = run->run(); // run() catches its own errors -> -1 result
        for (const types::MissionRunResult& mission_result : result.mission_results) {
            for (const types::ErrorRef& error : mission_result.errors) {
                logError(error_log, run_index, error); // immediate
            }
        }
        return result;
    } catch (const std::exception& e) {
        const types::ErrorRef error{"RUN_CREATE_FAILED", e.what()};
        logError(error_log, run_index, error);
        types::SimulationResult failed;
        failed.simulation_config = sim;
        failed.mission_config = mission;
        failed.resolution_request_status = types::ResolutionRequestStatus::Ignored;
        failed.mission_results.push_back(types::MissionRunResult{types::MissionRunStatus::Error, 0, {error}});
        failed.mission_score = -1.0;
        return failed;
    }
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    const fs::path results_dir = output_path / "output_results";
    std::error_code ec;
    fs::create_directories(results_dir, ec);
    std::ofstream error_log(results_dir / "errors.txt", std::ios::trunc); // errors logged as they occur

    std::vector<types::SimulationResult> runs;
    std::size_t index = 0;
    // Expand the cartesian product: per group, per its missions, per drone, per lidar.
    for (const auto& group : composition.simulation_mission_groups) {
        const types::SimulationConfigData& sim = std::get<0>(group);
        const std::vector<types::MissionConfigData>& missions = std::get<1>(group);
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    const fs::path run_dir = results_dir / ("run_" + std::to_string(index));
                    fs::create_directories(run_dir, ec);
                    runs.push_back(runOne(*run_factory_, sim, mission, drone, lidar, run_dir, error_log, index));
                    ++index;
                }
            }
        }
    }

    types::SimulationManagerReport report;
    report.generated_at_utc = nowUtc();
    report.metric = "cell_accuracy_percent";
    report.error_score = -1;
    report.score_range = scoreRange(runs);
    report.runs = std::move(runs);

    writeReportYaml(output_path / "simulation_output.yaml", report);
    return report;
}

} // namespace drone_mapper
