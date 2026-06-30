cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(TinyNPY CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/CompositionParser.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Compile the yaml example file
add_executable(example_yml cpp_yaml_example/main.cpp)
target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)

# --- Tests ---
enable_testing()
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(drone_mapper_simulation_test
    tests/smoke_test.cpp
    tests/components/Map3DTest.cpp
    tests/components/SimulationRunTest.cpp
    tests/components/MockLidarTest.cpp
    tests/components/ScanResultToVoxelsTest.cpp
    tests/components/MappingAlgorithmTest.cpp
    tests/components/DroneControlTest.cpp
    tests/components/MissionControlTest.cpp
    tests/components/MapsComparisonTest.cpp
    tests/components/SimulationManagerTest.cpp
    tests/integration/SimulationEndToEndTest.cpp
    tests/integration/MockAlgorithmIntegrationTest.cpp
    tests/integration/BenchmarkMapIntegrationTest.cpp
)
target_compile_definitions(drone_mapper_simulation_test
    PRIVATE
        BENCHMARK_MAP_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data_maps/benchmark_map.npy")
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
)
gtest_discover_tests(drone_mapper_simulation_test)
