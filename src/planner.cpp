#include "planner.h"

namespace Planner {

bool isTrajectoryWithinDynamicLimits(
    const Common::PolynomialTrajectory &trajectory,
    const FrenetTrajectoryLimits &limits) {
    return trajectory.isMaxAccelerationBelowLimit(limits.acceleration) &&
           trajectory.isMaxJerkBelowLimit(limits.jerk);
}

} // namespace Planner