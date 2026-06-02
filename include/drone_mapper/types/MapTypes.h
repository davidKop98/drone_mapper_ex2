#pragma once

#include <drone_mapper/Units.h>

namespace drone_mapper::types {

enum class VoxelOccupancy {
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

} // namespace drone_mapper::types
