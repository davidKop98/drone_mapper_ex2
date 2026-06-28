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
//   * Sweep the origin's MappingBounds box in fine sub-voxel steps (resolution / 4),
//     like the collision sweep, and score only points that are ALSO in-bounds in the
//     target (the intersection). atVoxel snaps each sample to its voxel, so on aligned
//     same-resolution grids the score equals the per-voxel ratio; the finer sampling
//     adds precision when the origin and target grids are offset/misaligned.
//   * Binary "Occupied vs not": a sampled point is correct when origin and target AGREE
//     on whether it is Occupied. Unmapped and PotentiallyOccupied count as "not Occupied",
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
    const double max_x = bounds.max_x.force_numerical_value_in(cm);
    const double max_y = bounds.max_y.force_numerical_value_in(cm);
    const double max_z = bounds.max_height.force_numerical_value_in(cm);

    // Fine sub-voxel sweep: step a quarter of a cell, sampling sub-cell centers so no
    // sample lands exactly on a voxel boundary. Counts are integers so every voxel of an
    // aligned grid gets exactly the same number of samples.
    const double step = res / 4.0;
    const long nx = std::max(0L, std::lround((max_x - min_x) / step));
    const long ny = std::max(0L, std::lround((max_y - min_y) / step));
    const long nz = std::max(0L, std::lround((max_z - min_z) / step));

    for (IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
            continue;
        }
        long long total = 0;
        long long correct = 0;
        for (long iz = 0; iz < nz; ++iz) {
            const double z = min_z + (static_cast<double>(iz) + 0.5) * step;
            for (long iy = 0; iy < ny; ++iy) {
                const double y = min_y + (static_cast<double>(iy) + 0.5) * step;
                for (long ix = 0; ix < nx; ++ix) {
                    const double x = min_x + (static_cast<double>(ix) + 0.5) * step;
                    const Position3D sample = makePos(x, y, z);
                    if (!origin.isInBounds(sample) || !target->isInBounds(sample)) {
                        continue; // only score points in-bounds in BOTH maps
                    }
                    ++total;
                    if (isOccupied(origin, sample) == isOccupied(*target, sample)) {
                        ++correct;
                    }
                }
            }
        }
        // No overlapping in-bounds region (e.g. disjoint boxes or an empty grid) is
        // undefined rather than 0%; report a negative sentinel so callers can detect it.
        scores.push_back(total > 0 ? static_cast<double>(correct) / static_cast<double>(total) * 100.0 : -1.0);
    }
    return scores;
}

} // namespace drone_mapper
