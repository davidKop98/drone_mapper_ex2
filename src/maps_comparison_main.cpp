#include <drone_mapper/MapsComparison.h>

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <map1> <map2> [resolution_ratio=<res1>/<res2>]\n";
        return 1;
    }

    try {
        std::cout << drone_mapper::MapsComparison::compare(argv[1], argv[2]) << "\n";
    } catch (const std::exception& error) {
        std::cout << "-1\n";
        std::cerr << error.what() << "\n";
        return 1;
    }

    return 0;
}
