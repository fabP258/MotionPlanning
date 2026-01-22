#include "behaviour.h"
#include "frenet_grid_planner.h"
#include "path2d.h"
#include "planner.h"
#include <iostream>

#ifdef HAS_MATPLOTPP
#include <matplot/matplot.h>
#endif

int main() {
    Planner::CostWeights latCostWeights;
    latCostWeights.squaredJerkIntegral = 1.0f;
    latCostWeights.squaredTargetdeviation = 1.0f;
    latCostWeights.maneuverTime = 1.0f;

    Planner::FrenetTrajectoryLimits latLimits;
    latLimits.acceleration = 5.0f;
    latLimits.jerk = 15.0f;

    Planner::FrenetTrajectoryLimits longLimits;
    longLimits.acceleration = 2.0f;

    Planner::FrenetGridSearchPlanner planner(latCostWeights, latLimits,
                                             longLimits);

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
    longLeadState.accel = 0.2f;
    float minGap = 5.0f;
    float timeGap = 1.5f;
    Planner::FollowingBehaviour longBehaviour(longLeadState, minGap, timeGap);

    // TODO: create proper reference path
    std::array<float, 4> xPoly = {0.0f, 1.0f, 0.0f, 0.0f};
    std::array<float, 4> yPoly = {0.2f, 0.0f, 0.0f, 0.0f};
    Common::Path2D referencePath{Common::Polynom(xPoly),
                                 Common::Polynom(yPoly)};

    // TODO: create proper road boundaries
    Planner::RoadBoundary leftRoadBoundary;
    Planner::RoadBoundary rightRoadBoundary;

    // buffer for all valid (dynamically) trajectories
    std::vector<Planner::FrenetTrajectory> validTrajectories;

    std::optional<Planner::FrenetTrajectory> optimalTrajectory =
        planner.run(referencePath, leftRoadBoundary, rightRoadBoundary,
                    latState, longState, longBehaviour, validTrajectories);

    if (!optimalTrajectory) {
        std::cout << "Did not find a valid trajectory.\n";
    }

    std::cout << "Found " << validTrajectories.size()
              << " dynamically valid trajectories\n\n";

#ifdef HAS_MATPLOTPP
    if (optimalTrajectory) {
        matplot::figure();
        constexpr float dt = 0.02f;

        // Longitudinal subplot
        matplot::subplot(2, 1, 0);
        const auto &longTraj = optimalTrajectory->longTrajectory;
        std::vector<double> t_vals, s_vals;
        for (float t = 0.0f; t <= longTraj.endTime(); t += dt) {
            t_vals.push_back(static_cast<double>(t));
            s_vals.push_back(static_cast<double>(longTraj.evaluate(t)));
        }
        matplot::plot(t_vals, s_vals)->line_width(2).color("blue");
        matplot::xlabel("t [s]");
        matplot::ylabel("s [m]");
        matplot::title("Longitudinal Trajectory");

        // Lateral subplot
        matplot::subplot(2, 1, 1);
        const auto &latTraj = optimalTrajectory->latTrajectory;
        t_vals.clear();
        std::vector<double> d_vals;
        for (float t = 0.0f; t <= latTraj.endTime(); t += dt) {
            t_vals.push_back(static_cast<double>(t));
            d_vals.push_back(static_cast<double>(latTraj.evaluate(t)));
        }
        matplot::plot(t_vals, d_vals)->line_width(2).color("blue");
        matplot::xlabel("t [s]");
        matplot::ylabel("d [m]");
        matplot::title("Lateral Trajectory");

        matplot::show();
    }
#endif

    return 0;
}