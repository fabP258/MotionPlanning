# Motion Planning

A C++20 motion planning library using Frenet coordinates and polynomial trajectory optimization.

## Features

### Implemented

- **Polynomial Trajectory Generation**: Up to 5th degree polynomial trajectories with smooth jerk minimization
- **Frenet Grid Search Planner**: Cost-based trajectory sampling and selection in Frenet coordinates
- **Longitudinal Behaviors**:
  - Following Behavior: Car-following with configurable time gap and minimum distance
  - Stopping Behavior: Deceleration to a target stop point
  - Merging Behavior: Gap-based merging between lead and lag vehicles
  - Velocity Keeping Behavior: Maintain desired velocity with offset sampling
- **Cost Functions**: Configurable weights for jerk, maneuver time, and target deviation
- **Trajectory Validation**: Acceleration and jerk limit checking
- **2D Path Representation**: Cubic polynomial path fitting with arc length parameterization
- **Compile-time Grid Generation**: Template-based linspace for efficient sampling grids

### Planned

- Collision detection and avoidance
- Road boundary checking and constraints
- Static obstacle avoidance
- Dynamic replanning
- Cartesian trajectory conversion

## Build Instructions

### Prerequisites

- CMake 3.10 or higher
- C++20 compatible compiler (GCC 10+, Clang 10+, MSVC 2019+)

### Building

```bash
mkdir build
cd build
cmake ..
make
```

### Running

```bash
./MotionPlanning
```

## Project Structure

```
include/          # Header files
  ├── behaviour.h          # Longitudinal behavior definitions
  ├── planner.h            # Frenet grid search planner
  ├── polynomial_trajectory.h  # Trajectory representation
  ├── polynom.h            # Polynomial math utilities
  ├── geometry.h           # Frenet states and 2D paths
  └── linalg.h             # Linear algebra utilities
src/              # Implementation files
main.cpp          # Example usage
```
