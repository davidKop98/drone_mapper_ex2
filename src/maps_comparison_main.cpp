#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Units.h>

#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace drone_mapper;

namespace {

// A MapConfig matching a loaded array's shape: offset 0, resolution 1cm, boundaries the
// full [0, shape) grid. Both maps get a config built this way, so on the mandatory
// same-grid path cell i maps 1:1 to voxel i in either map.
types::MapConfig configForShape(const NpyArray& arr) {
    const NpyArray::shape_t& shape = arr.Shape();
    const double nx = (shape.size() > 0) ? static_cast<double>(shape[0]) : 0.0;
    const double ny = (shape.size() > 1) ? static_cast<double>(shape[1]) : 0.0;
    const double nz = (shape.size() > 2) ? static_cast<double>(shape[2]) : 0.0;

    types::MapConfig cfg;
    cfg.offset = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    cfg.resolution = 1.0 * cm;
    cfg.boundaries = types::MappingBounds{
        0.0 * x_extent[cm], nx * x_extent[cm],
        0.0 * y_extent[cm], ny * y_extent[cm],
        0.0 * z_extent[cm], nz * z_extent[cm],
    };
    return cfg;
}

// Load a .npy into a shared NpyArray; returns nullptr (with `error` set) on failure.
std::shared_ptr<NpyArray> loadNpy(const std::string& path, std::string& error) {
    auto arr = std::make_shared<NpyArray>();
    const char* err = arr->LoadNPY(path);
    if (err != nullptr) {
        error = "failed to load '" + path + "': " + err;
        return nullptr;
    }
    if (arr->Shape().size() != 3) {
        error = "map '" + path + "' is not a 3D array";
        return nullptr;
    }
    return arr;
}

// Print -1 to stdout and a description to stderr; return the program's error code.
int fail(const std::string& message) {
    std::cout << "-1\n";
    std::cerr << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char** argv) {
    // ./maps_comparison <origin_map> <target_map> [comparison_config=<path>]
    if (argc < 3 || argc > 4) {
        return fail("Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]");
    }
    try {
        const std::string origin_path = argv[1];
        const std::string target_path = argv[2];
        // argv[3] (comparison_config) only matters for the different-resolution bonus; on
        // the mandatory path both maps share offset/boundaries/resolution, so it is ignored.

        std::string error;
        const std::shared_ptr<NpyArray> origin_arr = loadNpy(origin_path, error);
        if (!origin_arr) {
            return fail(error);
        }
        const std::shared_ptr<NpyArray> target_arr = loadNpy(target_path, error);
        if (!target_arr) {
            return fail(error);
        }

        Map3DImpl origin_map(origin_arr, configForShape(*origin_arr));
        Map3DImpl target_map(target_arr, configForShape(*target_arr));

        const std::vector<IMap3D*> targets{&target_map};
        const std::vector<double> scores = MapsComparison::compare(origin_map, targets);

        const double score = scores.at(0);
        if (score < 0.0) { // no overlapping in-bounds region between the two maps
            return fail("maps have no overlapping in-bounds region to compare");
        }
        std::cout << score << '\n'; // ONLY the score number on stdout
        return 0;
    } catch (const std::exception& e) {
        return fail(std::string("error: ") + e.what());
    } catch (...) {
        return fail("error: unknown failure");
    }
}
