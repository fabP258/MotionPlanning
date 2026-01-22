#ifndef FRENET_LATTICE_PLANNER_H_INCLUDED
#define FRENET_LATTICE_PLANNER_H_INCLUDED

#include "planner.h"
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace Planner {

class FrenetStateLatticePlanner {
  private:
    struct Node {
        int t_idx, d_idx, ds_idx, v_idx;

        // Helper to check if node is initialized
        bool operator==(const Node &other) const {
            return t_idx == other.t_idx && d_idx == other.d_idx &&
                   ds_idx == other.ds_idx && v_idx == other.v_idx;
        }
    };

    FrenetTrajectoryLimits latLimits_;
    FrenetTrajectoryLimits longLimits_;

    // Planning cycle time for trajectory evaluation
    static constexpr float CYCLE_TIME = 0.1f;

    // Grid sizes
    static constexpr int T_SZ = 5;
    static constexpr int D_SZ = 5;
    static constexpr int DS_SZ = 5;
    static constexpr int V_SZ = 5;

    // Time grid starts at first planning horizon (root is at t=0)
    static constexpr auto TIME_GRID = linspace<T_SZ>(1.0f, 5.0f);
    static constexpr auto LATERAL_DISTANCE_GRID = linspace<D_SZ>(2.5f, 2.5f);
    static constexpr auto LONG_OFFSET_GRID = linspace<DS_SZ>(0.0f, 30.0f);
    static constexpr auto LONG_SPEED_GRID = linspace<V_SZ>(0.0f, 25.0f);

    // DP Tables
    float costTable[T_SZ][D_SZ][DS_SZ][V_SZ];

    // Stores the trajectory that resulted in the best cost for this node
    // TODO: might be beneficial to not store all edges and recompute optimal
    // path edges in backtracking
    std::optional<FrenetTrajectory> edgeTable[T_SZ][D_SZ][DS_SZ][V_SZ];

    // Stores the index of the parent node for backtracking
    Node parentTable[T_SZ][D_SZ][DS_SZ][V_SZ];

    // Initial edges from continuous root to first lattice layer (t_idx=0)
    std::optional<FrenetTrajectory> initialEdgeTable[D_SZ][DS_SZ][V_SZ];

    // Storage for previous trajectory (for state continuity across cycles)
    std::optional<FrenetSplineTrajectory<T_SZ>> previousTrajectory_;

    void initializeCostTable();

    void expandFromRoot(const Common::FrenetState &latState,
                        const Common::FrenetState &longState);

    // Dummy helper methods to be implemented
    bool isCollisionFree(const Common::PolynomialTrajectory &lat,
                         const Common::PolynomialTrajectory &lon) {
        // TODO: Implement spatiotemporal collision checking
        return true;
    }

    float calculateCombinedCost(const Common::PolynomialTrajectory &lat,
                                const Common::PolynomialTrajectory &lon) {
        // TODO: rework this function
        // Weights for the cost function
        const float w_jerk = 1.0f;
        const float w_v = 10.0f;
        const float target_v = 20.0f;

        float cost = w_jerk * (lat.jerkCost() + lon.jerkCost());
        cost += w_v * std::pow(lon.endState().velocity - target_v, 2);
        return cost;
    }

  public:
    FrenetStateLatticePlanner(const FrenetTrajectoryLimits &latLimits,
                              const FrenetTrajectoryLimits &longLimits)
        : latLimits_(latLimits), longLimits_(longLimits) {
    }

    std::optional<FrenetSplineTrajectory<T_SZ>>
    run(const Common::FrenetState &startLat,
        const Common::FrenetState &startLong);

    std::optional<FrenetSplineTrajectory<T_SZ>> reconstructPath() const;

    std::optional<Node> findBestTerminalNode() const;
};

} // namespace Planner

#endif // FRENET_LATTICE_PLANNER_H_INCLUDED