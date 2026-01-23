#ifndef FRENET_GRID_PLANNER_H_INCLUDED
#define FRENET_GRID_PLANNER_H_INCLUDED

#include "behaviour.h"
#include "planner.h"
#include <vector>

namespace Planner {

class FrenetGridSearchPlanner {

  public:
    FrenetGridSearchPlanner(const CostWeights &latCostWeights,
                            const FrenetTrajectoryLimits &latLimits,
                            const FrenetTrajectoryLimits &longLimits)
        : latCostWeights_(latCostWeights), latLimits_(latLimits),
          longLimits_(longLimits) {
    }

    std::optional<FrenetTrajectory>
    run(const Common::Path2D &referencePath,
        const RoadBoundary &leftRoadBoundary,
        const RoadBoundary &rightRoadBoundary,
        const Common::FrenetState &latState,
        const Common::FrenetState &longState,
        const LongitudinalBehaviour &longBehaviour,
        std::vector<FrenetTrajectory> &debugTrajectories);

    void reset();

  private:
    std::optional<FrenetTrajectory> previousTrajectory_;
    CostWeights latCostWeights_;
    FrenetTrajectoryLimits latLimits_;
    FrenetTrajectoryLimits longLimits_;
    float vehicleHalfWidth_ = 1.25f;

    static constexpr float CYCLE_TIME = 0.1f;

    // Compile-time generated equidistant grids
    static constexpr std::array<float, 11> LATERAL_DISTANCE_GRID =
        linspace<11>(-0.5f, 0.5f);

    static constexpr std::array<float, 5> TIME_GRID = linspace<5>(1.0f, 30.0f);

    Common::FixedCapacityBuffer<Common::PolynomialTrajectory,
                                LATERAL_DISTANCE_GRID.size()>
    sampleLateralTrajectories(const Common::FrenetState &startState,
                              const float endTime) const;

    Common::FixedCapacityBuffer<Common::PolynomialTrajectory,
                                MAX_LONGITUDINAL_OFFSET_SAMPLES>
    sampleLongitudinalTrajectories(const Common::FrenetState &startState,
                                   const LongitudinalBehaviour &behaviour,
                                   const float endTime) const;

    float calculateLateralCost(const Common::PolynomialTrajectory &latTraj,
                               const float endTime) const;

    float
    calculateLongitudinalCost(const Common::PolynomialTrajectory &longTraj,
                              const LongitudinalBehaviour &behaviour,
                              const Common::FrenetState &targetState,
                              const float endTime) const;

    bool isTrajectoryWithinRoadBoundaries(
        const FrenetTrajectory &trajectory,
        const FrenetRoadBoundary &leftRoadBoundary,
        const FrenetRoadBoundary &rightRoadBoundary) const;
};

} // namespace Planner

#endif // FRENET_GRID_SEARCH_PLANNER_H_INCLUDED