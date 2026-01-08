#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "behaviour.h"
#include "geometry.h"
#include "polynomial_trajectory.h"
#include <array>
#include <optional>

namespace Planner {

// Compile-time generation of equidistant grid points
// Generates N points from start to stop (inclusive)
template <std::size_t N>
constexpr std::array<float, N> linspace(float start, float stop) {
    static_assert(N > 0, "Grid must have at least one point");

    std::array<float, N> result{};

    if constexpr (N == 1) {
        result[0] = start;
    } else {
        float step = (stop - start) / (N - 1);
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = start + i * step;
        }
    }

    return result;
}

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

    struct FrenetTrajectoryLimits {
        float acceleration = 2.0f;
    };

    FrenetGridSearchPlanner(const CostWeights &latCostWeights,
                            const FrenetTrajectoryLimits &latLimits,
                            const FrenetTrajectoryLimits &longLimits)
        : latCostWeights_(latCostWeights), latLimits_(latLimits),
          longLimits_(longLimits) {
    }

    void run(const Common::FrenetState &latState,
             const Common::FrenetState &longState,
             const LongitudinalBehaviour &longBehaviour);

    void reset();

  private:
    std::optional<FrenetTrajectory> previousTrajectory_;
    CostWeights latCostWeights_;
    FrenetTrajectoryLimits latLimits_;
    FrenetTrajectoryLimits longLimits_;

    static constexpr float CYCLE_TIME = 0.1f;

    // Compile-time generated equidistant grids
    static constexpr std::array<float, 11> LATERAL_DISTANCE_GRID =
        linspace<11>(-0.25f, 0.25f);

    static constexpr std::array<float, 5> TIME_GRID = linspace<5>(1.0f, 3.0f);

    std::array<std::optional<Common::PolynomialTrajectory>,
               LATERAL_DISTANCE_GRID.size()>
    sampleLateralTrajectories(const Common::FrenetState &startState,
                              const float endTime) const;

    // TODO: replace the vector with a fixed size buffer
    std::vector<std::optional<Common::PolynomialTrajectory>>
    sampleLongitudinalTrajectories(const Common::FrenetState &startState,
                                   const LongitudinalBehaviour &behaviour,
                                   const float endTime) const;

    float calculateLateralCost(const Common::PolynomialTrajectory &latTraj,
                               const float endTime);
};

} // namespace Planner

#endif // PLANNER_H_INCLUDED
