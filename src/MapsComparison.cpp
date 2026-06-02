#include <drone_mapper/MapsComparison.h>

#include <drone_mapper/Map3DImpl.h>

namespace drone_mapper {

double MapsComparison::compare(const IMap3D& expected,
                               const IMap3D& actual,
                               PhysicalLength resolution) {
    (void)expected;
    (void)actual;
    (void)resolution;
    return -1.0;
}

double MapsComparison::compare(const std::filesystem::path& expected,
                               const std::filesystem::path& actual,
                               PhysicalLength resolution) {
    const Map3DImpl expected_map{expected, resolution};
    const Map3DImpl actual_map{actual, resolution};
    return compare(expected_map, actual_map, resolution);
}

} // namespace drone_mapper
