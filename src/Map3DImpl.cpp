#include <drone_mapper/Map3DImpl.h>

#include <fstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    (void)pos;
    return types::VoxelOccupancy::Unmapped;
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    (void)pos;
    (void)value;
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    std::ofstream output{path};
    if (!output) {
        throw std::runtime_error("Failed to create output map file.");
    }
}

} // namespace drone_mapper
