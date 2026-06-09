#include <drone_mapper/Map3DImpl.h>

#include <fstream>
#include <stdexcept>
#include <string>

namespace drone_mapper {

Map3DImpl::Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution)
    : Map3DImpl(path, resolution, Position3D{}) {}

Map3DImpl::Map3DImpl(const std::filesystem::path& path, PhysicalLength resolution, Position3D offset)
    : map_(std::make_shared<NpyArray>()),
      config_{types::MappingBounds{}, offset, resolution} {
    const std::string path_string = path.string();
    const char* error = map_->LoadNPY(path_string.c_str());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file: ") + error);
    }
}

Map3DImpl::Map3DImpl(const types::MappingBounds& bounds, PhysicalLength resolution, Position3D offset)
    : map_(std::make_shared<NpyArray>()),
      config_{bounds, offset, resolution} {
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
