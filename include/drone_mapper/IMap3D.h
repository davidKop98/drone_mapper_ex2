#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// Read-only 3D occupancy map interface used by LiDAR implementations.
// **Do not change this interface.**
class IMap3D {
public:
    virtual ~IMap3D() = default;

    [[nodiscard]] virtual types::VoxelOccupancy get(const Position3D& pos) const = 0;
    [[nodiscard]] virtual PhysicalLength resolution() const = 0;
};

} // namespace drone_mapper
