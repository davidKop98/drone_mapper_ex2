#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <map1_path> <map2_path> [resolution_ratio=<res1>/<res2>]\n";
        return 1;
    }
    return 0;
}
