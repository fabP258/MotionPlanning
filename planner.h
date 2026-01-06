#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "behaviour.h"
#include "geometry.h"
#include <array>
#include <iostream>
#include <optional>

namespace Planner {

class FrenetGridSearchPlanner {

  public:
    void run(const Common::FrenetState &latState,
             const Common::FrenetState &longState,
             const LongitudinalBehaviour &longBehaviour) {
        for (int i = 0; i < TIME_GRID.size(); i++) {

            // sample lateral trajectories
            std::array<std::optional<Common::TPolynom<float, 5>>,
                       LATERAL_DISTANCE_GRID.size()>
                lateralTrajectories;
            for (int j = 0; j < LATERAL_DISTANCE_GRID.size(); j++) {
                std::cout << "Sampling lateral trajectory d(t) with d1=";
                std::cout << LATERAL_DISTANCE_GRID[j];
                std::cout << " and t1=" << TIME_GRID[i] << ", d1_test=";

                Common::FrenetState latEndState = {LATERAL_DISTANCE_GRID[j],
                                                   0.0f, 0.0f};
                lateralTrajectories[j] = Common::solveBoundaryValueProblem(
                    latState, latEndState, TIME_GRID[i]);
                if (lateralTrajectories[j]) {
                    std::cout
                        << (*lateralTrajectories[j]).evaluate(TIME_GRID[i])
                        << "\n";
                } else {
                    std::cout << "INVALID\n";
                }
                // TODO: calc lat only cost
            }

            // sample longitudinal trajectories
            // TODO: grid should be dependent on the velocity
            std::array<std::optional<Common::TPolynom<float, 5>>,
                       ARC_LENGTH_GRID.size()>
                longitudinalTrajectories;
            // Common::FrenetState longTargetState =
            // longBehaviour.calcTargetState(TIME_GRID[i]);
            for (int j = 0; j < ARC_LENGTH_GRID.size(); j++) {
                std::cout << "Sampling longitudinal trajectory s(t) with s1=";
                std::cout << ARC_LENGTH_GRID[j];
                std::cout << " and t1=" << TIME_GRID[i] << "\n";
                longitudinalTrajectories[j] = longBehaviour.calcTrajectory(
                    longState, TIME_GRID[i], ARC_LENGTH_GRID[j]);
                // TODO: calc long only cost
            }

            // TODO: evaluate cross-wise set superposition of lat. and long.
            // trajectories
            // - feasibility checks
            // - cost
            // - collision check (static and dynamic)
            // - road boundary check
            // -> update current best trajectories and proceed with next time
            // end time
        }
    }

  private:
    // TODO: calculate equidistant grids at compile-time
    static constexpr std::array<float, 11> LATERAL_DISTANCE_GRID = {
        -0.25f, 0.2f, 0.15f, 0.1f, 0.05f, 0.0f,
        0.05f,  0.1f, 0.15f, 0.2f, 0.25f};
    static constexpr std::array<float, 11> ARC_LENGTH_GRID = {
        0.0f,  5.0f,  10.0f, 15.0f, 20.0f, 25.0f,
        30.0f, 40.0f, 45.0f, 50.0f, 55.0f};
    static constexpr std::array<float, 5> TIME_GRID = {1.0f, 1.5f, 2.0f, 2.5f,
                                                       3.0f};
};

} // namespace Planner

#endif // PLANNER_H_INCLUDED