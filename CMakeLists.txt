cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_VISUALIZATION "Enable trajectory visualization with Matplot++" OFF)

# Core library with all source files
set(LIB_SOURCES
    src/polynom.cpp
    src/polynomial_trajectory.cpp
    src/geometry.cpp
    src/linalg.cpp
    src/behaviour.cpp
    src/frenet_grid_planner.cpp
    src/frenet_lattice_planner.cpp
    src/path2d.cpp
    src/planner.cpp
)

add_library(motion_planning_lib STATIC ${LIB_SOURCES})
target_include_directories(motion_planning_lib PUBLIC include)

# Main executable
add_executable(MotionPlanning main.cpp)
target_link_libraries(MotionPlanning motion_planning_lib)

if(ENABLE_VISUALIZATION)
    find_package(Matplot++ REQUIRED)
    target_link_libraries(MotionPlanning Matplot++::matplot)
    target_compile_definitions(MotionPlanning PRIVATE HAS_MATPLOTPP)
endif()

# Testing
option(BUILD_TESTS "Build the tests" ON)
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # Test sources - add new test files here
    set(TEST_SOURCES
        test/test_polynom.cpp
        test/test_polynomial_trajectory.cpp
    )

    add_executable(unit_tests ${TEST_SOURCES})
    target_link_libraries(unit_tests motion_planning_lib GTest::gtest GTest::gtest_main)
    add_test(NAME unit_tests COMMAND unit_tests)
endif()