#pragma once

#include <cpp_course/IMap3D.h>
#include <cpp_course/Types.h>

#include <filesystem>

namespace drone_mapper {

class MapsComparison {
public:
    [[nodiscard]] static double compare(const IMap3D& expected,
                                        const IMap3D& actual,
                                        PhysicalLength resolution = 1.0 * cm);
    [[nodiscard]] static double compare(const std::filesystem::path& expected,
                                        const std::filesystem::path& actual,
                                        PhysicalLength resolution = 1.0 * cm);
};

} // namespace drone_mapper
