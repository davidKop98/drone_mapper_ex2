// DroneControl is isolated with GMock'd IMappingAlgorithm / ILidar / IDroneMovement,
// driving against real MockGPS views and a real Map3DImpl output map.
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/Units.h>
#include <drone_mapper/types/DroneTypes.h>
#include <drone_mapper/types/LidarTypes.h>
#include <drone_mapper/types/MapTypes.h>
#include <drone_mapper/types/MissionTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using ::testing::_;
using ::testing::InSequence;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Return;

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    explicit MockMappingAlgorithm(const IMap3D& map)
        : IMappingAlgorithm(MissionConfigData{}, LidarConfigData{}, DroneConfigData{}, map) {}
    MOCK_METHOD(MappingStepCommand, nextStep,
                (const DroneState& state, const LidarScanResult* latest_scan), (override));
};
class MockLidarSensor : public ILidar {
public:
    MOCK_METHOD(LidarScanResult, scan, (Orientation scan_orientation), (const, override));
    MOCK_METHOD(LidarConfigData, config, (), (const, override)); // 20.6: ILidar gained config()
};
class MockDroneMovement : public IDroneMovement {
public:
    MOCK_METHOD(MovementResult, rotate, (RotationDirection direction, HorizontalAngle angle), (override));
    MOCK_METHOD(MovementResult, advance, (PhysicalLength distance), (override));
    MOCK_METHOD(MovementResult, elevate, (PhysicalLength distance), (override));
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
LidarConfigData lidarCfg() {
    LidarConfigData l;
    l.z_min = 10.0 * cm;
    l.z_max = 300.0 * cm;
    l.d = 10.0 * cm;
    l.fov_circles = 1;
    return l;
}
Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
Orientation O(double h, double a) {
    return Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
}
std::shared_ptr<GpsTruth> makeTruth(double x, double y, double z, double h) {
    auto t = std::make_shared<GpsTruth>();
    t->position = P(x, y, z);
    t->heading = O(h, 0);
    return t;
}
MovementCommand advanceCmd(double cm_) {
    return MovementCommand{MovementCommandType::Advance, RotationDirection::Left,
                           0.0 * horizontal_angle[deg], cm_ * cm};
}
} // namespace

// Movement executes before the scan within a single step.
TEST(DroneControl, MovementExecutedBeforeScan) {
    Map3DImpl output(makeArray(10, 5, 5, 255), mapCfg(10, 10, 5, 5));
    Map3DImpl algo_map(makeArray(1, 1, 1, 255), mapCfg(10, 1, 1, 1));
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    auto truth = makeTruth(55, 25, 25, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 10.0 * cm);
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    MappingStepCommand cmd{advanceCmd(10), O(0, 0), AlgorithmStatus::Working};
    EXPECT_CALL(algo, nextStep(_, _)).WillOnce(Return(cmd));
    {
        InSequence seq;
        EXPECT_CALL(movement, advance(_)).WillOnce(Return(MovementResult{true, {}}));
        EXPECT_CALL(lidar, scan(_)).WillOnce(Return(LidarScanResult{}));
    }

    EXPECT_EQ(control.step().status, DroneStepStatus::Continue);
}

// applyToMap uses the EXACT origin, not the rounded GPS position.
TEST(DroneControl, AppliesScanFromExactOrigin) {
    Map3DImpl output(makeArray(40, 5, 5, 255), mapCfg(10, 40, 5, 5));
    Map3DImpl algo_map(makeArray(1, 1, 1, 255), mapCfg(10, 1, 1, 1));
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    auto truth = makeTruth(252, 25, 25, 0); // exact 252, rounds to 250 @ res 5
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 5.0 * cm);
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    MappingStepCommand cmd{std::nullopt, O(0, 0), AlgorithmStatus::Working};
    EXPECT_CALL(algo, nextStep(_, _)).WillOnce(Return(cmd));
    const LidarScanResult scan{LidarHit{48.0 * cm, O(0, 0)}}; // hit 48cm ahead
    EXPECT_CALL(lidar, scan(_)).WillOnce(Return(scan));

    (void)control.step();
    // exact 252 + 48 = 300 -> cell 30 Occupied; rounded 250 would land at cell 29.
    EXPECT_EQ(output.atVoxel(P(305, 25, 25)), VoxelOccupancy::Occupied);
    EXPECT_NE(output.atVoxel(P(295, 25, 25)), VoxelOccupancy::Occupied);
}

// A failed movement ends the step in Error and the scan is skipped.
TEST(DroneControl, FailedMovementReturnsErrorAndSkipsScan) {
    Map3DImpl output(makeArray(10, 5, 5, 255), mapCfg(10, 10, 5, 5));
    Map3DImpl algo_map(makeArray(1, 1, 1, 255), mapCfg(10, 1, 1, 1));
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    auto truth = makeTruth(55, 25, 25, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 10.0 * cm);
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    MappingStepCommand cmd{advanceCmd(10), O(0, 0), AlgorithmStatus::Working};
    EXPECT_CALL(algo, nextStep(_, _)).WillOnce(Return(cmd));
    EXPECT_CALL(movement, advance(_)).WillOnce(Return(MovementResult{false, "DRONE_HITS_OBSTACLE"}));
    EXPECT_CALL(lidar, scan(_)).Times(0);

    const DroneStepResult r = control.step();
    EXPECT_EQ(r.status, DroneStepStatus::Error);
    EXPECT_EQ(r.message, "DRONE_HITS_OBSTACLE");
}

// AlgorithmStatus maps to DroneStepStatus: Working->Continue, both Finished*->Completed.
TEST(DroneControl, StatusMapping) {
    auto stepStatusFor = [](AlgorithmStatus s) {
        Map3DImpl output(makeArray(5, 5, 5, 255), mapCfg(10, 5, 5, 5));
        Map3DImpl algo_map(makeArray(1, 1, 1, 255), mapCfg(10, 1, 1, 1));
        MockMappingAlgorithm algo(algo_map);
        MockLidarSensor lidar;
        MockDroneMovement movement;
        auto truth = makeTruth(25, 25, 25, 0);
        MockGPS exact(truth, 0.0 * cm);
        MockGPS rounded(truth, 5.0 * cm);
        DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

        EXPECT_CALL(algo, nextStep(_, _))
            .WillOnce(Return(MappingStepCommand{std::nullopt, std::nullopt, s}));
        return control.step().status;
    };
    EXPECT_EQ(stepStatusFor(AlgorithmStatus::Working), DroneStepStatus::Continue);
    EXPECT_EQ(stepStatusFor(AlgorithmStatus::Finished), DroneStepStatus::Completed);
    EXPECT_EQ(stepStatusFor(AlgorithmStatus::FinishedWithUnmappableVoxels), DroneStepStatus::Completed);
}

// The scan taken this step is threaded into the next nextStep (null on the first).
TEST(DroneControl, LatestScanThreadedToNextStep) {
    Map3DImpl output(makeArray(40, 5, 5, 255), mapCfg(10, 40, 5, 5));
    Map3DImpl algo_map(makeArray(1, 1, 1, 255), mapCfg(10, 1, 1, 1));
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    auto truth = makeTruth(55, 25, 25, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 10.0 * cm);
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    const LidarScanResult scan_a{LidarHit{30.0 * cm, O(0, 0)}};
    LidarScanResult captured;
    {
        InSequence seq;
        EXPECT_CALL(algo, nextStep(_, IsNull())) // first step: no prior scan
            .WillOnce(Return(MappingStepCommand{std::nullopt, O(0, 0), AlgorithmStatus::Working}));
        EXPECT_CALL(algo, nextStep(_, NotNull())) // second step: threaded scan
            .WillOnce([&](const DroneState&, const LidarScanResult* s) {
                captured = *s;
                return MappingStepCommand{std::nullopt, std::nullopt, AlgorithmStatus::Working};
            });
    }
    EXPECT_CALL(lidar, scan(_)).WillOnce(Return(scan_a));

    (void)control.step(); // takes scan_a, stores it
    (void)control.step(); // passes &scan_a

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_DOUBLE_EQ(captured[0].distance.force_numerical_value_in(cm), 30.0);
}
