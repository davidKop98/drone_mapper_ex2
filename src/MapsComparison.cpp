#include <drone_mapper/MapsComparison.h>

#include <drone_mapper/Units.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace drone_mapper {
namespace {

Position3D makePos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

[[nodiscard]] bool isOccupied(const IMap3D& map, const Position3D& center) {
    return map.atVoxel(center) == types::VoxelOccupancy::Occupied;
}

} // namespace

// Cell-by-cell accuracy in [0, 100], one score per target. Ported from EX1's
// Simulator::computeScore, adapted for EX2's voxel states:
//   * Iterate the origin's in-bounds cells (its MappingBounds box at its resolution),
//     sampling each cell center, and score only cells that are ALSO in-bounds in the
//     target (the intersection).
//   * Binary "Occupied vs not": a cell is correct when origin and target AGREE on
//     whether it is Occupied. Unmapped and PotentiallyOccupied count as "not Occupied",
//     so an unmapped wall (origin Occupied, target not) costs score.
//   * Divergence from EX1: EX1 required an exact Empty<->Empty / Occupied<->Occupied
//     match, so a target-Unmapped cell over an Empty origin cell was WRONG there. Under
//     this EX2 rule the two agree (both "not Occupied") -> correct. EX1 is silent on the
//     new PotentiallyOccupied state; we fold it into "not Occupied" the same way.
//   * Same-resolution only; comparing maps of different resolutions is the skipped bonus.
std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());

    const types::MapConfig cfg = origin.getMapConfig();
    const double res = cfg.resolution.force_numerical_value_in(cm);
    if (res <= 0.0) { // no cell size -> nothing we can score
        scores.assign(targets.size(), 0.0);
        return scores;
    }

    const auto& bounds = cfg.boundaries;
    const double min_x = bounds.min_x.force_numerical_value_in(cm);
    const double min_y = bounds.min_y.force_numerical_value_in(cm);
    const double min_z = bounds.min_height.force_numerical_value_in(cm);
    const int nx = std::max(0, static_cast<int>(std::lround((bounds.max_x.force_numerical_value_in(cm) - min_x) / res)));
    const int ny = std::max(0, static_cast<int>(std::lround((bounds.max_y.force_numerical_value_in(cm) - min_y) / res)));
    const int nz = std::max(0, static_cast<int>(std::lround((bounds.max_height.force_numerical_value_in(cm) - min_z) / res)));

    for (IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(0.0);
            continue;
        }
        long long total = 0;
        long long correct = 0;
        for (int iz = 0; iz < nz; ++iz) {
            const double z = min_z + (static_cast<double>(iz) + 0.5) * res;
            for (int iy = 0; iy < ny; ++iy) {
                const double y = min_y + (static_cast<double>(iy) + 0.5) * res;
                for (int ix = 0; ix < nx; ++ix) {
                    const double x = min_x + (static_cast<double>(ix) + 0.5) * res;
                    const Position3D center = makePos(x, y, z);
                    if (!origin.isInBounds(center) || !target->isInBounds(center)) {
                        continue; // only score cells in-bounds in BOTH maps
                    }
                    ++total;
                    if (isOccupied(origin, center) == isOccupied(*target, center)) {
                        ++correct;
                    }
                }
            }
        }
        // No overlapping in-bounds cells (e.g. disjoint boxes or an empty grid) is
        // undefined rather than 0%; report a negative sentinel so callers can detect it.
        scores.push_back(total > 0 ? static_cast<double>(correct) / static_cast<double>(total) * 100.0 : -1.0);
    }
    return scores;
}

} // namespace drone_mapper
