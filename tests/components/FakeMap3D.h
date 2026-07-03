#pragma once
// Hand-rolled IMutableMap3D test fixture, deliberately independent of Map3DImpl:
// sparse cell storage, its own trivial floor snap, extents given at construction.
// Suites whose subject merely NEEDS a map (MockLidar, DroneControl, MappingAlgorithm)
// use this so that no Map3DImpl mutation can propagate into them; suites whose
// subject IS map interaction (ScanResultToVoxels, MapsComparison, Map3D itself)
// stay on the real Map3DImpl.
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/Units.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <map>
#include <tuple>

namespace drone_mapper::test_support {

class FakeMap3D final : public IMutableMap3D {
public:
    // `unset` is what un-written cells read as (255-filled arrays -> Unmapped,
    // 1-filled "solid" arrays -> Occupied).
    FakeMap3D(types::MapConfig config, std::size_t nx, std::size_t ny, std::size_t nz,
              types::VoxelOccupancy unset = types::VoxelOccupancy::Unmapped)
        : config_(config), nx_(nx), ny_(ny), nz_(nz), unset_(unset) {}

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        long long x = 0, y = 0, z = 0;
        if (!cellOf(pos, x, y, z)) return types::VoxelOccupancy::OutOfBounds;
        const auto it = cells_.find(std::tuple{x, y, z});
        return (it == cells_.end()) ? unset_ : it->second;
    }

    [[nodiscard]] types::MapConfig getMapConfig() const override { return config_; }

    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        long long x = 0, y = 0, z = 0;
        return cellOf(pos, x, y, z);
    }

    void set(const Position3D& pos, types::VoxelOccupancy value) override {
        if (value == types::VoxelOccupancy::OutOfBounds) return;
        long long x = 0, y = 0, z = 0;
        if (!cellOf(pos, x, y, z)) return;
        cells_[std::tuple{x, y, z}] = value;
    }

    void save(const std::filesystem::path&) const override {} // fixture: nothing to persist

private:
    bool cellOf(const Position3D& pos, long long& x, long long& y, long long& z) const {
        const double res = config_.resolution.force_numerical_value_in(cm);
        if (res <= 0.0) return false;
        x = cell(pos.x.force_numerical_value_in(cm) - config_.offset.x.force_numerical_value_in(cm), res);
        y = cell(pos.y.force_numerical_value_in(cm) - config_.offset.y.force_numerical_value_in(cm), res);
        z = cell(pos.z.force_numerical_value_in(cm) - config_.offset.z.force_numerical_value_in(cm), res);
        return x >= 0 && y >= 0 && z >= 0 && x < static_cast<long long>(nx_) &&
               y < static_cast<long long>(ny_) && z < static_cast<long long>(nz_);
    }
    static long long cell(double offset_cm, double res) {
        return static_cast<long long>(std::floor(offset_cm / res + 1e-9));
    }

    types::MapConfig config_;
    std::size_t nx_, ny_, nz_;
    types::VoxelOccupancy unset_;
    std::map<std::tuple<long long, long long, long long>, types::VoxelOccupancy> cells_;
};

} // namespace drone_mapper::test_support
