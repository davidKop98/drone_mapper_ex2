#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

// Applies moves to the EXACT shared GpsTruth (through an exact MockGPS view, so both the
// exact and rounded views follow) and enforces ground-truth collision: the swept sphere
// of the drone radius along the path must not overlap an Occupied voxel of the hidden
// map. Collision is checked with the hidden map's own snapping (Map3DImpl::atVoxel), the
// same routine the algorithm's clearance uses, so the two agree. A sphere is heading-
// independent, so rotation never collides.
class MockMovement final : public IDroneMovement {
public:
    MockMovement(MockGPS& exact_gps, const IMap3D& hidden_map, PhysicalLength radius);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    // True if the drone-radius sphere swept from start to end overlaps any Occupied
    // voxel of the hidden map.
    [[nodiscard]] bool sweptSphereHitsOccupied(const Position3D& start, const Position3D& end) const;

    MockGPS& exact_gps_;
    const IMap3D& hidden_map_;
    PhysicalLength radius_;
};

} // namespace drone_mapper
