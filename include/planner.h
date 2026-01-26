#ifndef PLANNER_H_INCLUDED
#define PLANNER_H_INCLUDED

#include "fixed_capacity_buffer.h"
#include "geometry.h"
#include "path2d.h"
#include "polynomial_spline_trajectory.h"
#include "polynomial_trajectory.h"

namespace Planner {

using RoadBoundary = Common::FixedCapacityBuffer<Common::Point2D, 100>;
using FrenetRoadBoundary =
    Common::FixedCapacityBuffer<Common::FrenetPoint, 100>;

struct FrenetTrajectoryLimits {
    float acceleration = 2.0f;
    float jerk = 5.0f;
};

bool isTrajectoryWithinDynamicLimits(
    const Common::PolynomialTrajectory &trajectory,
    const FrenetTrajectoryLimits &limits);

struct FrenetTrajectory {
    Common::PolynomialTrajectory latTrajectory;
    Common::PolynomialTrajectory longTrajectory;
};

template <size_t N> struct FrenetSplineTrajectory {
    Common::PolynomialSplineTrajectory<N> latTrajectorySpline;
    Common::PolynomialSplineTrajectory<N> longTrajectorySpline;
};

} // namespace Planner

#endif // PLANNER_H_INCLUDED
