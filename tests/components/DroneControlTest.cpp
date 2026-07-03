// DroneControl is isolated with GMock'd IMappingAlgorithm / ILidar / IDroneMovement.
// The output map and both GPS views are scaffolding (DroneControl's subject is step
// orchestration), so they are FakeMap3D / FakeGps: Map3DImpl and MockGPS mutations
// cannot fail this suite.
#include "FakeGps.h"
#include "FakeMap3D.h"

#include <drone_mapper/DroneControlImpl.h>
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
#include <vector>

namespace {
using namespace drone_mapper;
using namespace drone_mapper::types;
using ::testing::_;
using ::testing::InSequence;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Return;
using drone_mapper::test_support::FakeGps;
using drone_mapper::test_support::FakeMap3D;

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
MovementCommand advanceCmd(double cm_) {
    return MovementCommand{MovementCommandType::Advance, RotationDirection::Left,
                           0.0 * horizontal_angle[deg], cm_ * cm};
}
} // namespace

// Movement executes before the scan within a single step.
TEST(DroneControl, MovementExecutedBeforeScan) {
    FakeMap3D output(mapCfg(10, 10, 5, 5), 10, 5, 5);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(55, 25, 25), O(0, 0));
    FakeGps rounded(P(60, 30, 30), O(0, 0)); // what res-10 rounding would report
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
    FakeMap3D output(mapCfg(10, 40, 5, 5), 40, 5, 5);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(252, 25, 25), O(0, 0)); // exact 252; res-5 rounding would say 250
    FakeGps rounded(P(250, 25, 25), O(0, 0));
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
    FakeMap3D output(mapCfg(10, 10, 5, 5), 10, 5, 5);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(55, 25, 25), O(0, 0));
    FakeGps rounded(P(60, 30, 30), O(0, 0)); // what res-10 rounding would report
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
        FakeMap3D output(mapCfg(10, 5, 5, 5), 5, 5, 5);
        FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
        MockMappingAlgorithm algo(algo_map);
        MockLidarSensor lidar;
        MockDroneMovement movement;
        FakeGps exact(P(25, 25, 25), O(0, 0));
        FakeGps rounded(P(25, 25, 25), O(0, 0)); // already on the res-5 grid
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
    FakeMap3D output(mapCfg(10, 40, 5, 5), 40, 5, 5);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(55, 25, 25), O(0, 0));
    FakeGps rounded(P(60, 30, 30), O(0, 0)); // what res-10 rounding would report
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

// The algorithm must see BELIEF, not truth: the DroneState fed to nextStep is built
// from the ROUNDED GPS view. Exact truth leaking here silently breaks the whole
// gps_resolution design (the drone would plan with information it cannot have).
// Also pins the step_index contract: consecutive steps feed 0, then 1.
TEST(DroneControl, StateFedToAlgorithmIsRoundedView) {
    FakeMap3D output(mapCfg(10, 10, 10, 10), 10, 10, 10);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(252, 173, 91), O(0, 0));   // truth
    FakeGps rounded(P(250, 175, 90), O(0, 0)); // res-5 rounding: distinct on EVERY axis
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    std::vector<DroneState> seen;
    EXPECT_CALL(algo, nextStep(_, _))
        .Times(2)
        .WillRepeatedly([&](const DroneState& s, const LidarScanResult*) {
            seen.push_back(s);
            return MappingStepCommand{std::nullopt, std::nullopt, AlgorithmStatus::Working};
        });
    (void)control.step();
    (void)control.step();

    ASSERT_EQ(seen.size(), 2u);
    EXPECT_DOUBLE_EQ(seen[0].position.x.force_numerical_value_in(cm), 250.0);
    EXPECT_DOUBLE_EQ(seen[0].position.y.force_numerical_value_in(cm), 175.0);
    EXPECT_DOUBLE_EQ(seen[0].position.z.force_numerical_value_in(cm), 90.0);
    EXPECT_EQ(seen[0].step_index, 0u);
    EXPECT_EQ(seen[1].step_index, 1u);
}

// Heading-asymmetric twin of AppliesScanFromExactOrigin: at heading 90 the same 48cm
// hit must land in +y (exact y 252 + 48 = 300 -> cell y 30). A zeroed/constant heading
// in applyToMap would place it in +x instead and leave the +y cell Unmapped.
TEST(DroneControl, AppliesScanFromExactOriginAtHeading90) {
    FakeMap3D output(mapCfg(10, 5, 40, 5), 5, 40, 5);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(25, 252, 25), O(90, 0));   // facing +y
    FakeGps rounded(P(25, 250, 25), O(90, 0));
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    MappingStepCommand cmd{std::nullopt, O(0, 0), AlgorithmStatus::Working};
    EXPECT_CALL(algo, nextStep(_, _)).WillOnce(Return(cmd));
    const LidarScanResult scan{LidarHit{48.0 * cm, O(0, 0)}}; // hit 48cm ahead
    EXPECT_CALL(lidar, scan(_)).WillOnce(Return(scan));

    (void)control.step();
    EXPECT_EQ(output.atVoxel(P(25, 305, 25)), VoxelOccupancy::Occupied); // +y cell 30
    EXPECT_NE(output.atVoxel(P(45, 255, 25)), VoxelOccupancy::Occupied); // +x stays clear
}

// The lidar is scanned at the COMMANDED orientation, not a default one: the algorithm
// asks for (37, -12) -- non-zero and distinct on both axes so a dropped-orientation
// mutation cannot coincide -- and lidar.scan must receive exactly that.
TEST(DroneControl, ScanCommandOrientationPassedToLidar) {
    FakeMap3D output(mapCfg(10, 10, 5, 5), 10, 5, 5);
    FakeMap3D algo_map(mapCfg(10, 1, 1, 1), 1, 1, 1);
    MockMappingAlgorithm algo(algo_map);
    MockLidarSensor lidar;
    MockDroneMovement movement;
    FakeGps exact(P(55, 25, 25), O(0, 0));
    FakeGps rounded(P(60, 30, 30), O(0, 0));
    DroneControlImpl control(lidarCfg(), lidar, rounded, exact, movement, output, algo);

    MappingStepCommand cmd{std::nullopt, O(37, -12), AlgorithmStatus::Working};
    EXPECT_CALL(algo, nextStep(_, _)).WillOnce(Return(cmd));
    Orientation seen = O(0, 0);
    EXPECT_CALL(lidar, scan(_)).WillOnce([&](Orientation o) {
        seen = o;
        return LidarScanResult{};
    });

    (void)control.step();
    EXPECT_DOUBLE_EQ(seen.horizontal.force_numerical_value_in(deg), 37.0);
    EXPECT_DOUBLE_EQ(seen.altitude.force_numerical_value_in(deg), -12.0);
}
