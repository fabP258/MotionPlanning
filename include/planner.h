#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "fixed_capacity_buffer.h"
#include "geometry.h"
#include "path2d.h"
#include "polynomial_trajectory.h"
#include <array>

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

using RoadBoundary = Common::FixedCapacityBuffer<Common::Point2D, 100>;
using FrenetRoadBoundary =
    Common::FixedCapacityBuffer<Common::FrenetPoint, 100>;

struct FrenetTrajectory {
    Common::PolynomialTrajectory latTrajectory;
    Common::PolynomialTrajectory longTrajectory;
};

template <int N> class FrenetSplineTrajectory {
  private:
    std::array<FrenetTrajectory, N> trajectorySpline;

  public:
    FrenetSplineTrajectory(const std::array<FrenetTrajectory, N> &spline)
        : trajectorySpline(spline) {
    }

    std::optional<Common::FrenetState> evaluateLatState(const float t) const {
        if (t < 0.0f)
            return std::nullopt;

        float prevEndTime = 0.0f;
        for (const FrenetTrajectory &trajectory : trajectorySpline) {
            float endTime = prevEndTime + trajectory.latTrajectory.endTime();
            if (t <= endTime) {
                return trajectory.latTrajectory.evaluateState(t - prevEndTime);
            }
            prevEndTime = endTime;
        }

        return std::nullopt;
    }

    std::optional<Common::FrenetState> evaluateLongState(const float t) const {
        if (t < 0.0f)
            return std::nullopt;

        float prevEndTime = 0.0f;
        for (const FrenetTrajectory &trajectory : trajectorySpline) {
            float endTime = prevEndTime + trajectory.longTrajectory.endTime();
            if (t <= endTime) {
                return trajectory.longTrajectory.evaluateState(t - prevEndTime);
            }
            prevEndTime = endTime;
        }

        return std::nullopt;
    }
};

} // namespace Planner

#endif // PLANNER_H_INCLUDED
