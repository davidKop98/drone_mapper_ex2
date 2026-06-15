#include <drone_mapper/MapsComparison.h>

namespace drone_mapper {

std::vector<double> MapsComparison::compare(const IMap3D& original,
                               const std::vector<IMap3D*> targets) {
    // Discard inputs
    (void)original;
    (void)targets;
    // Push 100 - always identical!
    std::vector<double> vec;
    vec.push_back(100);
    return vec;
}

} // namespace drone_mapper
