#include "planner.h"
#include "behaviour.h"
#include "geometry.h"
#include "polynomial_trajectory.h"
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
            lateralTrajectories =
                sampleLateralTrajectories(latStartState, TIME_GRID[i]);

        // sample longitudinal trajectories
        std::vector<std::optional<Common::PolynomialTrajectory>>
            longitudinalTrajectories = sampleLongitudinalTrajectories(
                longStartState, longBehaviour, TIME_GRID[i]);

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

                // TODO: Add longitudinal cost calculation -> stupid to do here
                // TODO: Add feasibility checks (acceleration/jerk limits)
                // TODO: Add collision checking (static and dynamic)
                // TODO: Add road boundary checks

                float totalCost = latCost; // + longCost when implemented

                // Update best trajectory if this combination has lower cost
                if (totalCost < minCost) {
                    minCost = totalCost;
                    bestTrajectory =
                        FrenetTrajectory{lateralTrajectories[j].value(),
                                         longitudinalTrajectories[k].value()};
                }
            }
        }
    }

    // CRITICAL: Always update previousTrajectory_, even if no valid trajectory
    // found If bestTrajectory is nullopt, this resets previousTrajectory_ to
    // nullopt, forcing next cycle to use provided states instead of stale
    // trajectory
    previousTrajectory_ = bestTrajectory;
}

void FrenetGridSearchPlanner::reset() {
    previousTrajectory_.reset();
}

std::array<std::optional<Common::PolynomialTrajectory>,
           FrenetGridSearchPlanner::LATERAL_DISTANCE_GRID.size()>
FrenetGridSearchPlanner::sampleLateralTrajectories(
    const Common::FrenetState &startState, const float endTime) const {
    std::array<std::optional<Common::PolynomialTrajectory>,
               FrenetGridSearchPlanner::LATERAL_DISTANCE_GRID.size()>
        trajectories;
    int trajIdx = 0;
    Common::FrenetState endState = {0.0f, 0.0f, 0.0f};
    for (auto &t : trajectories) {
        endState.distance = LATERAL_DISTANCE_GRID[trajIdx++];
        t = Common::PolynomialTrajectory::fromBoundaryStates(startState,
                                                             endState, endTime);
        if (t && !t->isMaxAccelerationBelowLimit(latLimits_.acceleration))
            t.reset();
    }

    return trajectories;
}

std::vector<std::optional<Common::PolynomialTrajectory>>
FrenetGridSearchPlanner::sampleLongitudinalTrajectories(
    const Common::FrenetState &startState,
    const LongitudinalBehaviour &behaviour, const float endTime) const {
    std::vector<std::optional<Common::PolynomialTrajectory>> trajectories;
    for (const auto &offset : behaviour.offsetGrid()) {
        Common::FrenetState endState =
            behaviour.calcTargetState(endTime, offset);
        std::optional<Common::PolynomialTrajectory> traj;
        switch (behaviour.planningStrategy()) {
        case LongitudinalBehaviour::PlanningStrategy::FULL:
            traj = Common::PolynomialTrajectory::fromBoundaryStates(
                startState, endState, endTime);
            break;
        case LongitudinalBehaviour::PlanningStrategy::VELOCITY:
            traj = Common::PolynomialTrajectory::fromStartStateAndEndVelocity(
                startState, endState.velocity, endState.accel, endTime);
        }
        if (traj &&
            traj->isMaxAccelerationBelowLimit(longLimits_.acceleration)) {
            trajectories.push_back(traj);
        }
    }
    return trajectories;
}

float FrenetGridSearchPlanner::calculateLateralCost(
    const Common::PolynomialTrajectory &latTraj, const float endTime) {

    float jerkCost = latTraj.jerkCost();
    float timeCost = endTime;

    // Target deviation: difference between end position and target (0.0 for
    // lane keeping)
    float targetDeviation = latTraj.endState().distance - 0.0f;
    float deviationCost = targetDeviation * targetDeviation;

    return latCostWeights_.squaredJerkIntegral * jerkCost +
           latCostWeights_.maneuverTime * timeCost +
           latCostWeights_.squaredTargetdeviation * deviationCost;
}

} // namespace Planner