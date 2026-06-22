#include <drone_mapper/Map3DImpl.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_mapper {
namespace {

// uint8 storage encoding. OutOfBounds is NEVER stored; it is derived from the grid
// extent. Decode checks 0/255/253 explicitly, then falls back to Occupied so a
// hidden-map wall stored as any other nonzero value (e.g. 9) still reads Occupied.
constexpr std::uint8_t kEmpty = 0;
constexpr std::uint8_t kOccupied = 1;
constexpr std::uint8_t kPotentiallyOccupied = 253;
constexpr std::uint8_t kUnmapped = 255;

// A tiny fraction of a cell, added to the normalized (cell-count) value so a
// coordinate exactly on a cell boundary lands in the upper cell despite FP drift.
constexpr double kSnapEpsilon = 1e-9;

[[nodiscard]] types::VoxelOccupancy decodeByte(std::uint8_t byte) {
    switch (byte) {
    case kEmpty:
        return types::VoxelOccupancy::Empty;
    case kUnmapped:
        return types::VoxelOccupancy::Unmapped;
    case kPotentiallyOccupied:
        return types::VoxelOccupancy::PotentiallyOccupied;
    default:
        return types::VoxelOccupancy::Occupied;
    }
}

[[nodiscard]] std::uint8_t encodeValue(types::VoxelOccupancy value) {
    switch (value) {
    case types::VoxelOccupancy::Empty:
        return kEmpty;
    case types::VoxelOccupancy::Occupied:
        return kOccupied;
    case types::VoxelOccupancy::PotentiallyOccupied:
        return kPotentiallyOccupied;
    case types::VoxelOccupancy::Unmapped:
    case types::VoxelOccupancy::OutOfBounds:
        return kUnmapped;
    }
    return kUnmapped;
}

struct VoxelIndex {
    std::size_t x = 0;
    std::size_t y = 0;
    std::size_t z = 0;
    bool valid = false;
};

// THE single world -> voxel snapping site. index = floor((world - offset)/res + eps)
// per axis, all values extracted to cm. Returns valid=false when the config is
// unusable (non-3D shape or non-positive resolution) or the position is outside the
// grid extent (which is exactly how atVoxel decides OutOfBounds).
[[nodiscard]] VoxelIndex snapToVoxel(const Position3D& pos,
                                     const types::MapConfig& config,
                                     const NpyArray::shape_t& shape) {
    if (shape.size() != 3) {
        return {};
    }
    const double res = config.resolution.force_numerical_value_in(cm);
    if (res <= 0.0) {
        return {};
    }
    const double fx = std::floor(
        (pos.x.force_numerical_value_in(cm) - config.offset.x.force_numerical_value_in(cm)) / res + kSnapEpsilon);
    const double fy = std::floor(
        (pos.y.force_numerical_value_in(cm) - config.offset.y.force_numerical_value_in(cm)) / res + kSnapEpsilon);
    const double fz = std::floor(
        (pos.z.force_numerical_value_in(cm) - config.offset.z.force_numerical_value_in(cm)) / res + kSnapEpsilon);

    if (fx < 0.0 || fy < 0.0 || fz < 0.0) {
        return {};
    }
    if (fx >= static_cast<double>(shape[0]) ||
        fy >= static_cast<double>(shape[1]) ||
        fz >= static_cast<double>(shape[2])) {
        return {};
    }
    return VoxelIndex{static_cast<std::size_t>(fx),
                      static_cast<std::size_t>(fy),
                      static_cast<std::size_t>(fz),
                      true};
}

// C-order [x][y][z]: linear = x*ny*nz + y*nz + z.
[[nodiscard]] std::size_t linearIndex(const VoxelIndex& v, const NpyArray::shape_t& shape) {
    return v.x * shape[1] * shape[2] + v.y * shape[2] + v.z;
}

} // namespace

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
    const VoxelIndex v = snapToVoxel(pos, config_, map_->Shape());
    if (!v.valid) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    return decodeByte(map_->Data<std::uint8_t>()[linearIndex(v, map_->Shape())]);
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    return snapToVoxel(pos, config_, map_->Shape()).valid;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    if (value == types::VoxelOccupancy::OutOfBounds) {
        return; // OutOfBounds is never stored.
    }
    const VoxelIndex v = snapToVoxel(pos, config_, map_->Shape());
    if (!v.valid) {
        return;
    }
    map_->Data<std::uint8_t>()[linearIndex(v, map_->Shape())] = encodeValue(value);
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    const char* error = map_->SaveNPY(path.string());
    if (error != nullptr) {
        throw std::runtime_error(std::string("Failed to save map to '") + path.string() + "': " + error);
    }
}

} // namespace drone_mapper
