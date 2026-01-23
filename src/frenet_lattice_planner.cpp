#include "frenet_lattice_planner.h"
#include "geometry.h"
#include "planner.h"
#include "polynom.h"
#include <limits>

namespace Planner {

std::optional<FrenetSplineTrajectory<FrenetStateLatticePlanner::T_SZ>>
FrenetStateLatticePlanner::run(const Common::FrenetState &latState,
                               const Common::FrenetState &longState,
                               float referenceVelocity) {
    // Calculate initial state from previous trajectory or use input directly
    Common::FrenetState initialLat = latState;
    Common::FrenetState initialLong = longState;

    if (previousTrajectory_) {
        auto evalLat = previousTrajectory_->evaluateLatState(CYCLE_TIME);
        auto evalLong = previousTrajectory_->evaluateLongState(CYCLE_TIME);
        if (evalLat && evalLong) {
            initialLat = *evalLat;
            initialLong = *evalLong;
        }
    }

    initializeCostTable();

    // Expand from continuous root to first lattice layer
    expandFromRoot(initialLat, initialLong, referenceVelocity);

    // Forward Pass (Dynamic Programming)
    for (int t = 0; t < T_SZ - 1; ++t) {
        float dt = TIME_GRID[t + 1] - TIME_GRID[t];

        for (int d = 0; d < D_SZ; ++d) {
            for (int ds = 0; ds < DS_SZ; ++ds) {
                for (int v = 0; v < V_SZ; ++v) {

                    if (costTable[t][d][ds][v] ==
                        std::numeric_limits<float>::max())
                        continue;

                    // Current node state
                    Common::FrenetState currLat = {LATERAL_DISTANCE_GRID[d],
                                                   0.0f, 0.0f};
                    Common::FrenetState currLon = {LONG_OFFSET_GRID[ds],
                                                   LONG_SPEED_GRID[v], 0};

                    // Expand to next layer
                    for (int next_d = 0; next_d < D_SZ; ++next_d) {
                        // Note: Only consider forward movement or standstill
                        for (int next_ds = ds; next_ds < DS_SZ; ++next_ds) {
                            for (int next_v = 0; next_v < V_SZ; ++next_v) {

                                Common::FrenetState targetLat = {
                                    LATERAL_DISTANCE_GRID[next_d], 0, 0};
                                Common::FrenetState targetLon = {
                                    LONG_OFFSET_GRID[next_ds],
                                    LONG_SPEED_GRID[next_v], 0};

                                // Solve BVP
                                auto latTraj = Common::PolynomialTrajectory::
                                    fromBoundaryStates(currLat, targetLat, dt);
                                auto lonTraj = Common::PolynomialTrajectory::
                                    fromBoundaryStates(currLon, targetLon, dt);

                                if (!latTraj || !lonTraj)
                                    continue;

                                // check dynamic limits
                                if (!isTrajectoryWithinDynamicLimits(
                                        latTraj.value(), latLimits_) ||
                                    !isTrajectoryWithinDynamicLimits(
                                        lonTraj.value(), longLimits_)) {
                                    continue;
                                }

                                // calculate cost
                                float moveCost = calculateCombinedCost(
                                    *latTraj, *lonTraj, referenceVelocity);
                                float totalCost =
                                    costTable[t][d][ds][v] + moveCost;

                                if (totalCost < costTable[t + 1][next_d]
                                                         [next_ds][next_v] &&
                                    isCollisionFree(*latTraj, *lonTraj)) {
                                    costTable[t + 1][next_d][next_ds][next_v] =
                                        totalCost;
                                    parentTable[t + 1][next_d][next_ds]
                                               [next_v] = {t, d, ds, v};
                                    edgeTable[t + 1][next_d][next_ds][next_v] =
                                        FrenetTrajectory{latTraj.value(),
                                                         lonTraj.value()};
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Backtrack to find optimal sequence
    auto result = reconstructPath();
    previousTrajectory_ = result;
    return result;
}

void FrenetStateLatticePlanner::initializeCostTable() {
    for (int t = 0; t < T_SZ; ++t) {
        for (int d = 0; d < D_SZ; ++d) {
            for (int ds = 0; ds < DS_SZ; ++ds) {
                for (int v = 0; v < V_SZ; ++v) {
                    costTable[t][d][ds][v] = std::numeric_limits<float>::max();
                }
            }
        }
    }
}

void FrenetStateLatticePlanner::expandFromRoot(
    const Common::FrenetState &latState, const Common::FrenetState &longState,
    const float referenceVelocity) {
    // Time from root (t=0) to first lattice layer
    float dt = TIME_GRID[0];

    for (int d = 0; d < D_SZ; ++d) {
        for (int ds = 0; ds < DS_SZ; ++ds) {
            for (int v = 0; v < V_SZ; ++v) {
                Common::FrenetState targetLat = {LATERAL_DISTANCE_GRID[d], 0,
                                                 0};
                Common::FrenetState targetLon = {LONG_OFFSET_GRID[ds],
                                                 LONG_SPEED_GRID[v], 0};

                // Solve BVP from continuous root state to lattice node
                auto latTraj = Common::PolynomialTrajectory::fromBoundaryStates(
                    latState, targetLat, dt);
                auto lonTraj = Common::PolynomialTrajectory::fromBoundaryStates(
                    longState, targetLon, dt);

                if (latTraj && lonTraj && isCollisionFree(*latTraj, *lonTraj)) {
                    float cost = calculateCombinedCost(*latTraj, *lonTraj,
                                                       referenceVelocity);
                    costTable[0][d][ds][v] = cost;
                    initialEdgeTable[d][ds][v] =
                        FrenetTrajectory{latTraj.value(), lonTraj.value()};
                }
            }
        }
    }
}

std::optional<FrenetSplineTrajectory<FrenetStateLatticePlanner::T_SZ>>
FrenetStateLatticePlanner::reconstructPath() const {
    // Find best node in final layer
    auto bestNode = findBestTerminalNode();
    if (!bestNode) {
        return std::nullopt;
    }

    std::array<FrenetTrajectory, T_SZ> spline;
    Node current = *bestNode;

    // Backtrack through lattice layers (t_idx > 0)
    while (current.t_idx > 0) {
        spline[current.t_idx] = *edgeTable[current.t_idx][current.d_idx]
                                          [current.ds_idx][current.v_idx];
        current = parentTable[current.t_idx][current.d_idx][current.ds_idx]
                             [current.v_idx];
    }

    // Add initial edge from continuous root to first lattice layer
    spline[0] = *initialEdgeTable[current.d_idx][current.ds_idx][current.v_idx];

    return spline;
}

std::optional<FrenetStateLatticePlanner::Node>
FrenetStateLatticePlanner::findBestTerminalNode() const {
    float minTotal = std::numeric_limits<float>::max();
    std::optional<Node> best = std::nullopt;

    for (int d = 0; d < D_SZ; ++d) {
        for (int ds = 0; ds < DS_SZ; ++ds) {
            for (int v = 0; v < V_SZ; ++v) {
                if (costTable[T_SZ - 1][d][ds][v] < minTotal) {
                    minTotal = costTable[T_SZ - 1][d][ds][v];
                    best = Node{T_SZ - 1, d, ds, v};
                }
            }
        }
    }

    return best;
}

float FrenetStateLatticePlanner::calculateCombinedCost(
    const Common::PolynomialTrajectory &lat,
    const Common::PolynomialTrajectory &lon, float referenceVelocity) const {
    return calculateLateralCost(lat) +
           calculateLongitudinalCost(lon, referenceVelocity);
}

float FrenetStateLatticePlanner::calculateLateralCost(
    const Common::PolynomialTrajectory &trajectory) const {
    return latCostWeights_.squaredJerkIntegral * trajectory.jerkCost() +
           latCostWeights_.squaredTargetdeviation *
               trajectory.distanceCost(0.0f);
}

float FrenetStateLatticePlanner::calculateLongitudinalCost(
    const Common::PolynomialTrajectory &trajectory,
    float referenceVelocity) const {
    return lonCostWeights_.squaredJerkIntegral * trajectory.jerkCost() +
           lonCostWeights_.squaredTargetdeviation *
               trajectory.velocityCost(referenceVelocity);
}

} // namespace Planner