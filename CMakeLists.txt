cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

set(SOURCES
    src/polynom.cpp
    src/polynomial_trajectory.cpp
    src/geometry.cpp
    src/linalg.cpp
    src/behaviour.cpp
    src/planner.cpp
    main.cpp
)

add_executable(MotionPlanning ${SOURCES})