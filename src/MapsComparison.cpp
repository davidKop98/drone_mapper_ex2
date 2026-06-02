#include <drone_mapper/MapsComparison.h>

namespace drone_mapper {

double MapsComparison::compare(const IMap3D& map1,
                               const IMap3D& map2,
                               ResolutionRatio resolution_ratio) {
    (void)map1;
    (void)map2;
    (void)resolution_ratio;
    return -1.0;
}

} // namespace drone_mapper
