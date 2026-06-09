#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution);
    // Changed: added offset-aware construction for hidden maps loaded from NPY files.
    Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution, Position3D offset);
    // Changed: output maps are configured from map bounds/resolution/offset instead of mission boundaries.
    Map3DImpl(const types::MappingBounds& bounds, PhysicalLength resolution, Position3D offset = {});

    // Changed: implements the renamed IMap3D voxel query.
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Changed: exposes boundaries, offset, and resolution as one map-owned configuration.
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    // Changed: shared ownership supports the new pointer-based storage member.
    std::shared_ptr<NpyArray> map_;
    // Changed: replaces standalone resolution_ so all map geometry stays together.
    types::MapConfig config_;
};

} // namespace drone_mapper
