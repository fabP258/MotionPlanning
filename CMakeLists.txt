cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_VISUALIZATION "Enable trajectory visualization with Matplot++" OFF)

include_directories(include)

set(SOURCES
    src/polynom.cpp
    src/polynomial_trajectory.cpp
    src/geometry.cpp
    src/linalg.cpp
    src/behaviour.cpp
    src/frenet_grid_planner.cpp
    src/frenet_lattice_planner.cpp
    src/path2d.cpp
    main.cpp
)

add_executable(MotionPlanning ${SOURCES})

if(ENABLE_VISUALIZATION)
    find_package(Matplot++ REQUIRED)
    target_link_libraries(MotionPlanning Matplot++::matplot)
    target_compile_definitions(MotionPlanning PRIVATE HAS_MATPLOTPP)
endif()