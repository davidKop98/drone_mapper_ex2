#pragma once

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

namespace drone_mapper {

struct ResolutionRatio {
    PhysicalLength numerator;
    PhysicalLength denominator;
};

class MapsComparison {
public:
    [[nodiscard]] static double compare(const IMap3D& map1,
                                        const IMap3D& map2,
                                        ResolutionRatio resolution_ratio);
};

} // namespace drone_mapper
