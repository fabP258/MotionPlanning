#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "behaviour.h"
#include "fixed_capacity_buffer.h"
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
    struct FrenetTrajectoryLimits {
        float acceleration = 2.0f;
        float jerk = 5.0f;
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

    Common::FixedCapacityBuffer<Common::PolynomialTrajectory,
                                MAX_LONGITUDINAL_OFFSET_SAMPLES>
    sampleLongitudinalTrajectories(const Common::FrenetState &startState,
                                   const LongitudinalBehaviour &behaviour,
                                   const float endTime) const;

    static bool
    isTrajectoryValid(const Common::PolynomialTrajectory &trajectory,
                      const FrenetTrajectoryLimits &limits);

    float calculateLateralCost(const Common::PolynomialTrajectory &latTraj,
                               const float endTime) const;

    float
    calculateLongitudinalCost(const Common::PolynomialTrajectory &longTraj,
                              const LongitudinalBehaviour &behaviour,
                              const Common::FrenetState &targetState,
                              const float endTime) const;
};

} // namespace Planner

#endif // PLANNER_H_INCLUDED
