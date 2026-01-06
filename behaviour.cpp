#include "behaviour.h"
#include "geometry.h"
#include <cmath>

namespace Planner {

std::vector<std::optional<Common::TPolynom<float, 5>>>
LongitudinalBehaviour::sampleTrajectories(const Common::FrenetState &startState,
                                          const float endTime) const {
    std::vector<std::optional<Common::TPolynom<float, 5>>> trajectories;
    for (float offset : offsetGrid()) {
        trajectories.push_back(calcTrajectory(startState, endTime, offset));
    }
    return trajectories;
}

std::optional<Common::TPolynom<float, 5>>
LongitudinalBehaviour::calcTrajectory(const Common::FrenetState &startState,
                                      const float endTime, float offset) const {
    return dispatchCalculation(startState, calcTargetState(endTime, offset),
                               endTime);
}

std::span<const float> FollowingBehaviour::offsetGrid() const {
    return POSITION_OFFSET_GRID;
}

Common::FrenetState
FollowingBehaviour::calcTargetState(const float endTime,
                                    const float offset) const {
    Common::FrenetState targetState;

    targetState.distance =
        calcLeadVehiclePosition(endTime) -
        (minGap + timeGap * calcLeadVehicleVelocity(endTime)) + offset;
    targetState.velocity = calcLeadVehicleVelocity(endTime) -
                           timeGap * calcLeadVehicleAcceleration(endTime);
    targetState.accel = calcLeadVehicleAcceleration(endTime);

    return targetState;
}

std::optional<Common::TPolynom<float, 5>>
FollowingBehaviour::dispatchCalculation(const Common::FrenetState &startState,
                                        const Common::FrenetState &endState,
                                        const float endTime) const {
    return Common::solveBoundaryValueProblem(startState, endState, endTime);
}

float FollowingBehaviour::calcLeadVehiclePosition(const float time) const {
    return leadVehicleState.distance + leadVehicleState.velocity * time +
           0.5f * leadVehicleState.accel * std::pow(time, 2.0f);
}

float FollowingBehaviour::calcLeadVehicleVelocity(const float time) const {
    return leadVehicleState.velocity + leadVehicleState.accel * time;
}

float FollowingBehaviour::calcLeadVehicleAcceleration(const float time) const {
    return leadVehicleState.accel;
}

std::span<const float> StoppingBehaviour::offsetGrid() const {
    return POSITION_OFFSET_GRID;
}

Common::FrenetState
StoppingBehaviour::calcTargetState(const float endTime,
                                   const float offset) const {
    Common::FrenetState targetState;

    targetState.distance = stopDistance + offset;
    targetState.velocity = 0.0f;
    targetState.accel = 0.0f;

    return targetState;
}

std::optional<Common::TPolynom<float, 5>>
StoppingBehaviour::dispatchCalculation(const Common::FrenetState &startState,
                                       const Common::FrenetState &endState,
                                       const float endTime) const {
    return Common::solveBoundaryValueProblem(startState, endState, endTime);
}

std::span<const float> MergingBehaviour::offsetGrid() const {
    return POSITION_OFFSET_GRID;
}

Common::FrenetState
MergingBehaviour::calcTargetState(const float endTime,
                                  const float offset) const {
    Common::FrenetState targetState;

    float leadTargetDistance =
        predictVehiclePosition(leadVehicleState, endTime) -
        (minGap + timeGap * predictVehicleVelocity(leadVehicleState, endTime));
    float lagTargetDistance =
        predictVehiclePosition(lagVehicleState, endTime) + minGap +
        timeGap * predictVehicleVelocity(lagVehicleState, endTime);
    targetState.distance =
        0.5f * (leadTargetDistance + lagTargetDistance) + offset;

    float leadTargetVelocity =
        predictVehicleVelocity(leadVehicleState, endTime) -
        timeGap * predictVehicleAcceleration(leadVehicleState, endTime);
    float lagTargetVelocity =
        predictVehicleVelocity(lagVehicleState, endTime) +
        timeGap * predictVehicleAcceleration(lagVehicleState, endTime);
    targetState.velocity = 0.5f * (leadTargetVelocity + lagTargetVelocity);

    float leadTargetAccel =
        predictVehicleAcceleration(leadVehicleState, endTime);
    float lagTargetAccel = predictVehicleAcceleration(lagVehicleState, endTime);
    targetState.accel = 0.5f * (leadTargetAccel + lagTargetAccel);

    return targetState;
}

std::optional<Common::TPolynom<float, 5>>
MergingBehaviour::dispatchCalculation(const Common::FrenetState &startState,
                                      const Common::FrenetState &endState,
                                      const float endTime) const {
    return Common::solveBoundaryValueProblem(startState, endState, endTime);
}

float MergingBehaviour::predictVehiclePosition(const Common::FrenetState &state,
                                               const float time) const {
    return state.distance + state.velocity * time +
           0.5f * state.accel * std::pow(time, 2.0f);
}

float MergingBehaviour::predictVehicleVelocity(const Common::FrenetState &state,
                                               const float time) const {
    return state.velocity + state.accel * time;
}

float MergingBehaviour::predictVehicleAcceleration(
    const Common::FrenetState &state, const float time) const {
    return state.accel;
}

std::span<const float> VelocityKeepingBehaviour::offsetGrid() const {
    return VELOCITY_OFFSET_GRID;
}

Common::FrenetState
VelocityKeepingBehaviour::calcTargetState(const float endTime,
                                          const float offset) const {
    Common::FrenetState targetState;
    targetState.distance = 0.0f; // will be ignored
    targetState.velocity = desiredVelocity + offset;
    targetState.accel = 0.0f;

    return targetState;
}

std::optional<Common::TPolynom<float, 5>>
VelocityKeepingBehaviour::dispatchCalculation(
    const Common::FrenetState &startState, const Common::FrenetState &endState,
    const float endTime) const {
    return Common::solveBoundaryValueProblem(startState, endState.velocity,
                                             endState.accel, endTime);
}

} // namespace Planner