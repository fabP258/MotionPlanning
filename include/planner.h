#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "behaviour.h"
#include "geometry.h"
#include "polynomial_trajectory.h"
#include <array>
#include <optional>

namespace Planner {

struct FrenetTrajectory {
    Common::PolynomialTrajectory latTrajectory;
    Common::PolynomialTrajectory longTrajectory;
};

class FrenetGridSearchPlanner {

  public:
    struct CostWeights {
        float squaredJerkIntegral = 1.0f;
        float maneuverTime = 1.0f;
        float squaredTargetdeviation = 1.0f;
    };

    FrenetGridSearchPlanner(const CostWeights &latCostWeights)
        : latCostWeights_(latCostWeights) {
    }

    void run(const Common::FrenetState &latState,
             const Common::FrenetState &longState,
             const LongitudinalBehaviour &longBehaviour);

    void reset();

  private:
    std::optional<FrenetTrajectory> previousTrajectory_;
    CostWeights latCostWeights_;

    float calculateLateralCost(const Common::PolynomialTrajectory &latTraj,
                               const float endTime);

    static constexpr float CYCLE_TIME = 0.1f;
    // TODO: calculate equidistant grids at compile-time
    static constexpr std::array<float, 11> LATERAL_DISTANCE_GRID = {
        -0.25f, -0.2f, -0.15f, -0.1f, -0.05f, 0.0f,
        0.05f,  0.1f,  0.15f,  0.2f,  0.25f};
    static constexpr std::array<float, 11> ARC_LENGTH_GRID = {
        0.0f,  5.0f,  10.0f, 15.0f, 20.0f, 25.0f,
        30.0f, 40.0f, 45.0f, 50.0f, 55.0f};
    static constexpr std::array<float, 5> TIME_GRID = {1.0f, 1.5f, 2.0f, 2.5f,
                                                       3.0f};
};

} // namespace Planner

#endif // PLANNER_H_INCLUDED
