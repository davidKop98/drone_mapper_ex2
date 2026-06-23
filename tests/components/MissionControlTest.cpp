// MissionControl is isolated with a GMock'd IDroneControl, driving a real Map3DImpl
// output map so the end-of-mission save can be observed.
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using ::testing::Return;

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(DroneStepResult, step, (), (override));
    MOCK_METHOD(DroneState, state, (), (const, override));
};

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
MissionConfigData missionCfgSteps(std::size_t max_steps) {
    MissionConfigData m;
    m.max_steps = max_steps;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1;
    return m;
}
std::filesystem::path tempFile(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}
DroneStepResult cont() { return DroneStepResult{DroneStepStatus::Continue, {}}; }
DroneStepResult done() { return DroneStepResult{DroneStepStatus::Completed, {}}; }
DroneStepResult err(const char* m) { return DroneStepResult{DroneStepStatus::Error, m}; }
} // namespace

// Loops while the drone reports Continue, finishes Completed when it does, saves the map.
TEST(MissionControl, CompletedWhenDroneFinishes) {
    Map3DImpl hidden(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    Map3DImpl output(makeArray(5, 5, 5, 255), mapCfg(10, 5, 5, 5));
    MockDroneControl dc;
    const auto file = tempFile("dm_mission_completed.npy");
    std::filesystem::remove(file);
    MissionControlImpl mission(missionCfgSteps(100), DroneConfigData{}, hidden, output, dc, file);

    EXPECT_CALL(dc, step())
        .WillOnce(Return(cont()))
        .WillOnce(Return(cont()))
        .WillOnce(Return(done()));

    const MissionRunResult r = mission.runMission();
    EXPECT_EQ(r.status, MissionRunStatus::Completed);
    EXPECT_EQ(r.steps, 3u);
    EXPECT_TRUE(r.errors.empty());
    EXPECT_TRUE(std::filesystem::exists(file)); // output map saved at the end
    std::filesystem::remove(file);
}

// A drone that never finishes hits the step cap -> MaxSteps.
TEST(MissionControl, MaxStepsWhenNeverFinishing) {
    Map3DImpl hidden(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    Map3DImpl output(makeArray(5, 5, 5, 255), mapCfg(10, 5, 5, 5));
    MockDroneControl dc;
    const auto file = tempFile("dm_mission_maxsteps.npy");
    std::filesystem::remove(file);
    MissionControlImpl mission(missionCfgSteps(5), DroneConfigData{}, hidden, output, dc, file);

    EXPECT_CALL(dc, step()).WillRepeatedly(Return(cont()));

    const MissionRunResult r = mission.runMission();
    EXPECT_EQ(r.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(r.steps, 5u);
    std::filesystem::remove(file);
}

// A step Error short-circuits the loop -> Error (run scores -1 upstream).
TEST(MissionControl, ErrorShortCircuits) {
    Map3DImpl hidden(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    Map3DImpl output(makeArray(5, 5, 5, 255), mapCfg(10, 5, 5, 5));
    MockDroneControl dc;
    const auto file = tempFile("dm_mission_error.npy");
    std::filesystem::remove(file);
    MissionControlImpl mission(missionCfgSteps(100), DroneConfigData{}, hidden, output, dc, file);

    EXPECT_CALL(dc, step())
        .WillOnce(Return(cont()))
        .WillOnce(Return(err("DRONE_HITS_OBSTACLE")));

    const MissionRunResult r = mission.runMission();
    EXPECT_EQ(r.status, MissionRunStatus::Error);
    EXPECT_EQ(r.steps, 2u);
    EXPECT_FALSE(r.errors.empty());
    std::filesystem::remove(file);
}

// The output map is saved even when the loop terminates by error.
TEST(MissionControl, SavesOutputMapOnError) {
    Map3DImpl hidden(makeArray(5, 5, 5, 0), mapCfg(10, 5, 5, 5));
    Map3DImpl output(makeArray(5, 5, 5, 255), mapCfg(10, 5, 5, 5));
    MockDroneControl dc;
    const auto file = tempFile("dm_mission_save_on_error.npy");
    std::filesystem::remove(file);
    MissionControlImpl mission(missionCfgSteps(100), DroneConfigData{}, hidden, output, dc, file);

    EXPECT_CALL(dc, step()).WillOnce(Return(err("DRONE_HITS_OBSTACLE")));

    (void)mission.runMission();
    EXPECT_TRUE(std::filesystem::exists(file));
    std::filesystem::remove(file);
}
