#include "planner.h"
#include <iostream>
#include <limits>

namespace Planner {

void FrenetGridSearchPlanner::run(const Common::FrenetState &latState,
                                  const Common::FrenetState &longState,
                                  const LongitudinalBehaviour &longBehaviour) {

    Common::FrenetState latStartState = latState;
    Common::FrenetState longStartState = longState;
    if (previousTrajectory_) {
        if (std::optional<Common::FrenetState> latStateFromPrevious =
                previousTrajectory_->latTrajectory.evaluateState(CYCLE_TIME);
            latStateFromPrevious) {
            latStartState = latStateFromPrevious.value();
        }
        if (std::optional<Common::FrenetState> longStateFromPrevious =
                previousTrajectory_->longTrajectory.evaluateState(CYCLE_TIME);
            longStateFromPrevious) {
            longStartState = longStateFromPrevious.value();
        }
    }

    // Track best trajectory across all time horizons
    float minCost = std::numeric_limits<float>::max();
    std::optional<FrenetTrajectory> bestTrajectory;

    for (int i = 0; i < TIME_GRID.size(); i++) {

        // sample lateral trajectories
        std::array<std::optional<Common::PolynomialTrajectory>,
                   LATERAL_DISTANCE_GRID.size()>
            lateralTrajectories;
        for (int j = 0; j < LATERAL_DISTANCE_GRID.size(); j++) {
            std::cout << "Sampling lateral trajectory d(t) with d1=";
            std::cout << LATERAL_DISTANCE_GRID[j];
            std::cout << " and t1=" << TIME_GRID[i] << ", d1_test=";

            Common::FrenetState latEndState = {LATERAL_DISTANCE_GRID[j], 0.0f,
                                               0.0f};
            lateralTrajectories[j] =
                Common::PolynomialTrajectory::fromBoundaryStates(
                    latStartState, latEndState, TIME_GRID[i]);
            if (lateralTrajectories[j]) {
                std::cout << lateralTrajectories[j]->evaluate(TIME_GRID[i])
                          << ", squared jerk cost: "
                          << lateralTrajectories[j]->jerkCost() << "\n";
            } else {
                std::cout << "INVALID\n";
            }
            // TODO: calc lat only cost
        }

        // sample longitudinal trajectories
        std::vector<std::optional<Common::PolynomialTrajectory>>
            longitudinalTrajectories =
                longBehaviour.sampleTrajectories(longStartState, TIME_GRID[i]);

        // Evaluate cross-wise combinations of lateral and longitudinal
        // trajectories
        for (int j = 0; j < lateralTrajectories.size(); j++) {
            if (!lateralTrajectories[j])
                continue;

            float latCost = calculateLateralCost(lateralTrajectories[j].value(),
                                                  TIME_GRID[i]);

            for (int k = 0; k < longitudinalTrajectories.size(); k++) {
                if (!longitudinalTrajectories[k])
                    continue;

                // TODO: Add longitudinal cost calculation
                // TODO: Add feasibility checks (acceleration/jerk limits)
                // TODO: Add collision checking (static and dynamic)
                // TODO: Add road boundary checks

                float totalCost = latCost; // + longCost when implemented

                // Update best trajectory if this combination has lower cost
                if (totalCost < minCost) {
                    minCost = totalCost;
                    bestTrajectory = FrenetTrajectory{
                        lateralTrajectories[j].value(),
                        longitudinalTrajectories[k].value()};
                }
            }
        }
    }

    // CRITICAL: Always update previousTrajectory_, even if no valid trajectory found
    // If bestTrajectory is nullopt, this resets previousTrajectory_ to nullopt,
    // forcing next cycle to use provided states instead of stale trajectory
    previousTrajectory_ = bestTrajectory;
}

void FrenetGridSearchPlanner::reset() {
    previousTrajectory_.reset();
}

float FrenetGridSearchPlanner::calculateLateralCost(
    const Common::PolynomialTrajectory &latTraj, const float endTime) {

    float jerkCost = latTraj.jerkCost();
    float timeCost = endTime;

    // Target deviation: difference between end position and target (0.0 for lane keeping)
    float targetDeviation = latTraj.endState().distance - 0.0f;
    float deviationCost = targetDeviation * targetDeviation;

    return latCostWeights_.squaredJerkIntegral * jerkCost +
           latCostWeights_.maneuverTime * timeCost +
           latCostWeights_.squaredTargetdeviation * deviationCost;
}

} // namespace Planner