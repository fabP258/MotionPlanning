#include "behaviour.h"
#include "planner.h"

int main() {
    Planner::FrenetGridSearchPlanner::CostWeights latCostWeights;
    latCostWeights.squaredJerkIntegral = 1.0f;
    latCostWeights.squaredTargetdeviation = 1.0f;
    latCostWeights.maneuverTime = 1.0f;

    Planner::FrenetGridSearchPlanner planner(latCostWeights);

    Common::FrenetState latState;
    latState.distance = -0.12f;
    latState.velocity = 0.04f;
    latState.accel = 0.01f;

    Common::FrenetState longState;
    longState.distance = 0.0f;
    longState.velocity = 18.0f;
    longState.accel = 0.0f;

    Common::FrenetState longLeadState;
    longLeadState.distance = 50.0f;
    longLeadState.velocity = 28.0f;
    longLeadState.accel = 0.5f;
    float minGap = 5.0f;
    float timeGap = 1.5f;
    Planner::FollowingBehaviour longBehaviour(longLeadState, minGap, timeGap);

    planner.run(latState, longState, longBehaviour);

    return 0;
}