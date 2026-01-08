#include "behaviour.h"
#include "geometry.h"
#include <cmath>

namespace Planner {

Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
FollowingBehaviour::offsetGrid() const {
    return POSITION_OFFSET_GRID;
}

Common::FrenetState
FollowingBehaviour::calcTargetState(const float endTime) const {
    Common::FrenetState targetState;

    targetState.distance =
        calcLeadVehiclePosition(endTime) -
        (minGap_ + timeGap_ * calcLeadVehicleVelocity(endTime));
    targetState.velocity = calcLeadVehicleVelocity(endTime) -
                           timeGap_ * calcLeadVehicleAcceleration(endTime);
    targetState.accel = calcLeadVehicleAcceleration(endTime);

    return targetState;
}

LongitudinalBehaviour::PlanningStrategy
FollowingBehaviour::planningStrategy() const {
    return PlanningStrategy::FULL;
}

CostWeights FollowingBehaviour::costWeights() const {
    return costWeights_;
}

float FollowingBehaviour::calcLeadVehiclePosition(const float time) const {
    return leadVehicleState_.distance + leadVehicleState_.velocity * time +
           0.5f * leadVehicleState_.accel * std::pow(time, 2.0f);
}

float FollowingBehaviour::calcLeadVehicleVelocity(const float time) const {
    return leadVehicleState_.velocity + leadVehicleState_.accel * time;
}

float FollowingBehaviour::calcLeadVehicleAcceleration(const float time) const {
    return leadVehicleState_.accel;
}

Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
StoppingBehaviour::offsetGrid() const {
    return POSITION_OFFSET_GRID;
}

Common::FrenetState
StoppingBehaviour::calcTargetState(const float endTime) const {
    Common::FrenetState targetState;

    targetState.distance = stopDistance_;
    targetState.velocity = 0.0f;
    targetState.accel = 0.0f;

    return targetState;
}

LongitudinalBehaviour::PlanningStrategy
StoppingBehaviour::planningStrategy() const {
    return PlanningStrategy::FULL;
}

CostWeights StoppingBehaviour::costWeights() const {
    return costWeights_;
}

Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
MergingBehaviour::offsetGrid() const {
    return POSITION_OFFSET_GRID;
}

Common::FrenetState
MergingBehaviour::calcTargetState(const float endTime) const {
    Common::FrenetState targetState;

    float leadTargetDistance =
        predictVehiclePosition(leadVehicleState, endTime) -
        (minGap + timeGap * predictVehicleVelocity(leadVehicleState, endTime));
    float lagTargetDistance =
        predictVehiclePosition(lagVehicleState, endTime) + minGap +
        timeGap * predictVehicleVelocity(lagVehicleState, endTime);
    targetState.distance = 0.5f * (leadTargetDistance + lagTargetDistance);

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

LongitudinalBehaviour::PlanningStrategy
MergingBehaviour::planningStrategy() const {
    return PlanningStrategy::FULL;
}

CostWeights MergingBehaviour::costWeights() const {
    return costWeights_;
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

Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
VelocityKeepingBehaviour::offsetGrid() const {
    return VELOCITY_OFFSET_GRID;
}

Common::FrenetState
VelocityKeepingBehaviour::calcTargetState(const float endTime) const {
    Common::FrenetState targetState;
    targetState.distance = 0.0f; // will be ignored
    targetState.velocity = desiredVelocity;
    targetState.accel = 0.0f;

    return targetState;
}

LongitudinalBehaviour::PlanningStrategy
VelocityKeepingBehaviour::planningStrategy() const {
    return PlanningStrategy::VELOCITY;
}

CostWeights VelocityKeepingBehaviour::costWeights() const {
    return costWeights_;
}

} // namespace Planner
