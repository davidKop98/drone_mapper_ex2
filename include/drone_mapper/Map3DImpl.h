#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <filesystem>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution);
    Map3DImpl(const types::MappingBounds& bounds, PhysicalLength resolution);

    [[nodiscard]] types::VoxelOccupancy get(const Position3D& pos) const override;
    [[nodiscard]] PhysicalLength resolution() const override;
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& path) const override;

private:
    NpyArray map_;
    PhysicalLength resolution_;
};

} // namespace drone_mapper
