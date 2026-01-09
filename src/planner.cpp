#include "planner.h"
#include "behaviour.h"
#include "geometry.h"
#include "polynomial_trajectory.h"
#include <iostream>
#include <limits>

namespace Planner {

std::optional<FrenetTrajectory> FrenetGridSearchPlanner::run(
    const Common::Path2D &referencePath, const RoadBoundary &leftRoadBoundary,
    const RoadBoundary &rightRoadBoundary, const Common::FrenetState &latState,
    const Common::FrenetState &longState,
    const LongitudinalBehaviour &longBehaviour) {
    // calculate initial state
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

    // transform road boundary points into frenet frame
    FrenetRoadBoundary transformedLeftRoadBoundary;
    for (const Common::Point2D &p : leftRoadBoundary) {
        transformedLeftRoadBoundary.push_back(
            referencePath.projectPointIntoFrenet(p));
    }
    FrenetRoadBoundary transformedRightRoadBoundary;
    for (const Common::Point2D &p : rightRoadBoundary) {
        transformedRightRoadBoundary.push_back(
            referencePath.projectPointIntoFrenet(p));
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
        Common::FixedCapacityBuffer<Common::PolynomialTrajectory,
                                    MAX_LONGITUDINAL_OFFSET_SAMPLES>
            longitudinalTrajectories = sampleLongitudinalTrajectories(
                longStartState, longBehaviour, TIME_GRID[i]);

        // Evaluate cross-wise combinations of lateral and longitudinal
        // trajectories
        for (int j = 0; j < lateralTrajectories.size(); j++) {
            if (!lateralTrajectories[j])
                continue;

            for (int k = 0; k < longitudinalTrajectories.size(); k++) {
                // TODO: Add collision checking (static and dynamic)

                FrenetTrajectory currentTrajectory{
                    lateralTrajectories[j].value(),
                    longitudinalTrajectories[k]};

                if (!isTrajectoryWithinRoadBoundaries(
                        currentTrajectory, transformedLeftRoadBoundary,
                        transformedRightRoadBoundary)) {
                    continue;
                }

                float totalCost = lateralTrajectories[j]->cost() +
                                  longitudinalTrajectories[k].cost();

                // Update best trajectory if this combination has lower cost
                if (totalCost < minCost) {
                    minCost = totalCost;
                    bestTrajectory =
                        FrenetTrajectory{lateralTrajectories[j].value(),
                                         longitudinalTrajectories[k]};
                }
            }
        }
    }

    // CRITICAL: Always update previousTrajectory_, even if no valid trajectory
    // found If bestTrajectory is nullopt, this resets previousTrajectory_ to
    // nullopt, forcing next cycle to use provided states instead of stale
    // trajectory
    previousTrajectory_ = bestTrajectory;

    return bestTrajectory;
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
        if (!t)
            continue;
        t->setCost(calculateLateralCost(t.value(), endTime));
        // invalidate trajectory if it exceeds dynamic limits
        if (!isTrajectoryValid(t.value(), latLimits_))
            t.reset();
    }

    return trajectories;
}

Common::FixedCapacityBuffer<Common::PolynomialTrajectory,
                            MAX_LONGITUDINAL_OFFSET_SAMPLES>
FrenetGridSearchPlanner::sampleLongitudinalTrajectories(
    const Common::FrenetState &startState,
    const LongitudinalBehaviour &behaviour, const float endTime) const {
    Common::FixedCapacityBuffer<Common::PolynomialTrajectory,
                                MAX_LONGITUDINAL_OFFSET_SAMPLES>
        trajectories{};
    const Common::FrenetState targetState = behaviour.calcTargetState(endTime);
    for (const auto &offset : behaviour.offsetGrid()) {
        Common::FrenetState endState = targetState;
        std::optional<Common::PolynomialTrajectory> traj;
        switch (behaviour.planningStrategy()) {
        case LongitudinalBehaviour::PlanningStrategy::FULL:
            endState.distance += offset;
            traj = Common::PolynomialTrajectory::fromBoundaryStates(
                startState, endState, endTime);
            break;
        case LongitudinalBehaviour::PlanningStrategy::VELOCITY:
            endState.velocity += offset;
            traj = Common::PolynomialTrajectory::fromStartStateAndEndVelocity(
                startState, endState.velocity, endState.accel, endTime);
            break;
        }
        if (!traj)
            continue;
        traj->setCost(calculateLongitudinalCost(traj.value(), behaviour,
                                                targetState, endTime));
        // only add trajectory if it is within dynamic limits
        if (isTrajectoryValid(traj.value(), longLimits_)) {
            trajectories.push_back(traj.value());
        }
    }
    return trajectories;
}

float FrenetGridSearchPlanner::calculateLateralCost(
    const Common::PolynomialTrajectory &latTraj, const float endTime) const {

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

float FrenetGridSearchPlanner::calculateLongitudinalCost(
    const Common::PolynomialTrajectory &longTraj,
    const LongitudinalBehaviour &behaviour,
    const Common::FrenetState &targetState, const float endTime) const {
    float jerkCost = longTraj.jerkCost();
    float targetDeviation;
    switch (behaviour.planningStrategy()) {
    case LongitudinalBehaviour::PlanningStrategy::FULL:
        targetDeviation = longTraj.endState().distance - targetState.distance;
        break;
    case LongitudinalBehaviour::PlanningStrategy::VELOCITY:
        targetDeviation = longTraj.endState().velocity - targetState.velocity;
        break;
    }
    float squaredTargetDeviation = targetDeviation * targetDeviation;
    CostWeights weights = behaviour.costWeights();
    return weights.squaredJerkIntegral * jerkCost +
           weights.maneuverTime * endTime +
           weights.squaredTargetdeviation * squaredTargetDeviation;
}

bool FrenetGridSearchPlanner::isTrajectoryValid(
    const Common::PolynomialTrajectory &trajectory,
    const FrenetTrajectoryLimits &limits) {
    return trajectory.isMaxAccelerationBelowLimit(limits.acceleration) &&
           trajectory.isMaxJerkBelowLimit(limits.jerk);
}

bool FrenetGridSearchPlanner::isTrajectoryWithinRoadBoundaries(
    const FrenetTrajectory &trajectory,
    const FrenetRoadBoundary &leftRoadBoundary,
    const FrenetRoadBoundary &rightRoadBoundary) const {
    std::array<float, 100> timeGrid =
        linspace<100>(0, trajectory.latTrajectory.endTime());
    // sample trajectory on grid
    std::array<float, 100> sGrid;
    std::array<float, 100> dGrid;
    for (int i = 0; i < 100; ++i) {
        sGrid[i] = trajectory.longTrajectory.evaluate(timeGrid[i]);
        dGrid[i] = trajectory.latTrajectory.evaluate(timeGrid[i]);
    }

    // evaluate left road boundary
    for (const Common::FrenetPoint &p : leftRoadBoundary) {
        // search for closest s-match
        int idx = 0;
        float minDiff = std::numeric_limits<float>::max();
        for (int i = 0; i < sGrid.size(); ++i) {
            float diff = std::abs(p.s - sGrid[i]);
            if (diff < minDiff) {
                minDiff = diff;
                idx = i;
            }
        }
        // TODO: use linear interpolation instead of neirest neighbor

        // check for lateral distance - ignore rotation of the vehicle
        if (dGrid[idx] >= (p.d - vehicleHalfWidth_)) {
            return false;
        }
    }

    // evaluate right road boundary
    for (const Common::FrenetPoint &p : rightRoadBoundary) {
        // search for closest s-match
        int idx = 0;
        float minDiff = std::numeric_limits<float>::max();
        for (int i = 0; i < sGrid.size(); ++i) {
            float diff = std::abs(p.s - sGrid[i]);
            if (diff < minDiff) {
                minDiff = diff;
                idx = i;
            }
        }

        // check for lateral distance - ignore rotation of the vehicle
        if (dGrid[idx] <= (p.d + vehicleHalfWidth_)) {
            return false;
        }
    }

    return true;
}

} // namespace Planner