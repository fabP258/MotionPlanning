#include "polynomial_trajectory.h"
#include <gtest/gtest.h>
#include <cmath>

using Common::FrenetState;
using Common::PolynomialTrajectory;

// ============================================================================
// Helper Functions
// ============================================================================

FrenetState makeState(float d, float v, float a) {
    FrenetState s;
    s.distance = d;
    s.velocity = v;
    s.accel = a;
    return s;
}

// ============================================================================
// Factory Method Tests - fromBoundaryStates
// ============================================================================

TEST(PolynomialTrajectoryFromBoundaryStates, CreatesValidTrajectory) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);

    ASSERT_TRUE(traj.has_value());
    EXPECT_FLOAT_EQ(traj->endTime(), 2.0f);
    EXPECT_TRUE(traj->hasFullEndState());
}

TEST(PolynomialTrajectoryFromBoundaryStates, MatchesStartState) {
    FrenetState start = makeState(5.0f, 2.0f, 1.0f);
    FrenetState end = makeState(20.0f, 3.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 3.0f);
    ASSERT_TRUE(traj.has_value());

    auto evalStart = traj->evaluateState(0.0f);
    ASSERT_TRUE(evalStart.has_value());

    EXPECT_NEAR(evalStart->distance, start.distance, 1e-4f);
    EXPECT_NEAR(evalStart->velocity, start.velocity, 1e-4f);
    EXPECT_NEAR(evalStart->accel, start.accel, 1e-4f);
}

TEST(PolynomialTrajectoryFromBoundaryStates, MatchesEndState) {
    FrenetState start = makeState(0.0f, 5.0f, 0.0f);
    FrenetState end = makeState(15.0f, 2.0f, -1.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 4.0f);
    ASSERT_TRUE(traj.has_value());

    auto evalEnd = traj->evaluateState(4.0f);
    ASSERT_TRUE(evalEnd.has_value());

    EXPECT_NEAR(evalEnd->distance, end.distance, 1e-4f);
    EXPECT_NEAR(evalEnd->velocity, end.velocity, 1e-4f);
    EXPECT_NEAR(evalEnd->accel, end.accel, 1e-4f);
}

TEST(PolynomialTrajectoryFromBoundaryStates, ZeroEndTimeReturnsNullopt) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 0.0f);

    EXPECT_FALSE(traj.has_value());
}

TEST(PolynomialTrajectoryFromBoundaryStates, StationaryTrajectory) {
    FrenetState start = makeState(5.0f, 0.0f, 0.0f);
    FrenetState end = makeState(5.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Position should remain constant
    EXPECT_NEAR(traj->evaluate(0.0f), 5.0f, 1e-5f);
    EXPECT_NEAR(traj->evaluate(1.0f), 5.0f, 1e-5f);
    EXPECT_NEAR(traj->evaluate(2.0f), 5.0f, 1e-5f);
}

// ============================================================================
// Factory Method Tests - fromStartStateAndEndVelocity
// ============================================================================

TEST(PolynomialTrajectoryFromEndVelocity, CreatesValidTrajectory) {
    FrenetState start = makeState(0.0f, 10.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromStartStateAndEndVelocity(
        start, 20.0f, 0.0f, 3.0f);

    ASSERT_TRUE(traj.has_value());
    EXPECT_FLOAT_EQ(traj->endTime(), 3.0f);
    EXPECT_FALSE(traj->hasFullEndState());
}

TEST(PolynomialTrajectoryFromEndVelocity, MatchesStartState) {
    FrenetState start = makeState(10.0f, 5.0f, 2.0f);

    auto traj = PolynomialTrajectory::fromStartStateAndEndVelocity(
        start, 15.0f, 0.0f, 2.0f);
    ASSERT_TRUE(traj.has_value());

    auto evalStart = traj->evaluateState(0.0f);
    ASSERT_TRUE(evalStart.has_value());

    EXPECT_NEAR(evalStart->distance, start.distance, 1e-4f);
    EXPECT_NEAR(evalStart->velocity, start.velocity, 1e-4f);
    EXPECT_NEAR(evalStart->accel, start.accel, 1e-4f);
}

TEST(PolynomialTrajectoryFromEndVelocity, MatchesEndVelocityAndAcceleration) {
    FrenetState start = makeState(0.0f, 10.0f, 0.0f);
    float endVelocity = 25.0f;
    float endAcceleration = -2.0f;
    float endTime = 5.0f;

    auto traj = PolynomialTrajectory::fromStartStateAndEndVelocity(
        start, endVelocity, endAcceleration, endTime);
    ASSERT_TRUE(traj.has_value());

    EXPECT_NEAR(traj->velocity(endTime), endVelocity, 1e-4f);
    EXPECT_NEAR(traj->acceleration(endTime), endAcceleration, 1e-4f);
}

TEST(PolynomialTrajectoryFromEndVelocity, ZeroEndTimeReturnsNullopt) {
    FrenetState start = makeState(0.0f, 10.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromStartStateAndEndVelocity(
        start, 20.0f, 0.0f, 0.0f);

    EXPECT_FALSE(traj.has_value());
}

// ============================================================================
// Evaluation Tests
// ============================================================================

TEST(PolynomialTrajectoryEvaluate, EvaluateAndOperatorAreEquivalent) {
    FrenetState start = makeState(0.0f, 5.0f, 0.0f);
    FrenetState end = makeState(20.0f, 5.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 4.0f);
    ASSERT_TRUE(traj.has_value());

    EXPECT_FLOAT_EQ(traj->evaluate(2.0f), (*traj)(2.0f));
}

TEST(PolynomialTrajectoryEvaluate, EvaluateStateOutOfRangeReturnsNullopt) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    EXPECT_FALSE(traj->evaluateState(-0.1f).has_value());
    EXPECT_FALSE(traj->evaluateState(2.1f).has_value());
}

TEST(PolynomialTrajectoryEvaluate, EvaluateStateAtBoundaries) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Boundaries should be valid
    EXPECT_TRUE(traj->evaluateState(0.0f).has_value());
    EXPECT_TRUE(traj->evaluateState(2.0f).has_value());
}

// ============================================================================
// Derivative Tests
// ============================================================================

TEST(PolynomialTrajectoryDerivatives, VelocityMatchesFirstDerivative) {
    FrenetState start = makeState(0.0f, 3.0f, 1.0f);
    FrenetState end = makeState(20.0f, 5.0f, -1.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 4.0f);
    ASSERT_TRUE(traj.has_value());

    // Check at multiple points
    for (float t = 0.0f; t <= 4.0f; t += 0.5f) {
        auto state = traj->evaluateState(t);
        ASSERT_TRUE(state.has_value());
        EXPECT_NEAR(traj->velocity(t), state->velocity, 1e-5f);
    }
}

TEST(PolynomialTrajectoryDerivatives, AccelerationMatchesSecondDerivative) {
    FrenetState start = makeState(0.0f, 3.0f, 2.0f);
    FrenetState end = makeState(20.0f, 5.0f, -1.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 4.0f);
    ASSERT_TRUE(traj.has_value());

    // Check at multiple points
    for (float t = 0.0f; t <= 4.0f; t += 0.5f) {
        auto state = traj->evaluateState(t);
        ASSERT_TRUE(state.has_value());
        EXPECT_NEAR(traj->acceleration(t), state->accel, 1e-5f);
    }
}

TEST(PolynomialTrajectoryDerivatives, JerkIsThirdDerivative) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Jerk should match polynomial's 3rd derivative
    float jerkAtStart = traj->jerk(0.0f);
    float jerkFromPolynom = traj->polynom().derivative(3).evaluate(0.0f);
    EXPECT_FLOAT_EQ(jerkAtStart, jerkFromPolynom);
}

// ============================================================================
// Limit Checking Tests
// ============================================================================

TEST(PolynomialTrajectoryLimits, AccelerationBelowLimit) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 5.0f);
    ASSERT_TRUE(traj.has_value());

    // With a long duration, acceleration should be low
    EXPECT_TRUE(traj->isMaxAccelerationBelowLimit(10.0f));
}

TEST(PolynomialTrajectoryLimits, AccelerationExceedsLimit) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(100.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 1.0f);
    ASSERT_TRUE(traj.has_value());

    // Short duration with large distance change requires high acceleration
    EXPECT_FALSE(traj->isMaxAccelerationBelowLimit(1.0f));
}

TEST(PolynomialTrajectoryLimits, JerkBelowLimit) {
    FrenetState start = makeState(0.0f, 5.0f, 0.0f);
    FrenetState end = makeState(50.0f, 5.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 10.0f);
    ASSERT_TRUE(traj.has_value());

    // Constant velocity trajectory should have low jerk
    EXPECT_TRUE(traj->isMaxJerkBelowLimit(10.0f));
}

TEST(PolynomialTrajectoryLimits, JerkExceedsLimit) {
    FrenetState start = makeState(0.0f, 0.0f, 10.0f);
    FrenetState end = makeState(10.0f, 0.0f, -10.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 0.5f);
    ASSERT_TRUE(traj.has_value());

    // Large acceleration change in short time requires high jerk
    EXPECT_FALSE(traj->isMaxJerkBelowLimit(1.0f));
}

TEST(PolynomialTrajectoryLimits, ChecksBoundaryPoints) {
    FrenetState start = makeState(0.0f, 0.0f, 5.0f);  // High initial accel
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Verify the boundary acceleration is detected
    float startAccel = std::abs(traj->acceleration(0.0f));
    EXPECT_NEAR(startAccel, 5.0f, 1e-4f);

    // Limit below start acceleration should fail
    EXPECT_FALSE(traj->isMaxAccelerationBelowLimit(4.0f));
    // Limit above max acceleration should pass
    EXPECT_TRUE(traj->isMaxAccelerationBelowLimit(20.0f));
}

// ============================================================================
// Cost Function Tests
// ============================================================================

TEST(PolynomialTrajectoryCost, JerkCostIsNonNegative) {
    FrenetState start = makeState(0.0f, 5.0f, 0.0f);
    FrenetState end = makeState(20.0f, 5.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 4.0f);
    ASSERT_TRUE(traj.has_value());

    EXPECT_GE(traj->jerkCost(), 0.0f);
}

TEST(PolynomialTrajectoryCost, SmoothTrajectoryHasLowJerkCost) {
    // Constant velocity trajectory
    FrenetState start = makeState(0.0f, 10.0f, 0.0f);
    FrenetState end = makeState(100.0f, 10.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 10.0f);
    ASSERT_TRUE(traj.has_value());

    // Nearly constant velocity should have very low jerk cost
    EXPECT_NEAR(traj->jerkCost(), 0.0f, 1e-3f);
}

TEST(PolynomialTrajectoryCost, AggressiveTrajectoryHasHighJerkCost) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(100.0f, 0.0f, 0.0f);
    float shortTime = 1.0f;
    float longTime = 10.0f;

    auto shortTraj = PolynomialTrajectory::fromBoundaryStates(start, end, shortTime);
    auto longTraj = PolynomialTrajectory::fromBoundaryStates(start, end, longTime);
    ASSERT_TRUE(shortTraj.has_value());
    ASSERT_TRUE(longTraj.has_value());

    // Shorter duration requires more aggressive maneuver = higher jerk cost
    EXPECT_GT(shortTraj->jerkCost(), longTraj->jerkCost());
}

// Note: distanceCost and velocityCost require squaring polynomials.
// For degree-5 trajectories (from boundary states), squaring exceeds MAX_DEGREE=5.
// These tests verify the expected behavior with high-degree polynomials.

TEST(PolynomialTrajectoryCost, DistanceCostThrowsForHighDegree) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Degree-5 polynomial squared = degree-10, exceeds MAX_DEGREE
    EXPECT_THROW(traj->distanceCost(5.0f), std::runtime_error);
}

TEST(PolynomialTrajectoryCost, VelocityCostThrowsForHighDegree) {
    FrenetState start = makeState(0.0f, 10.0f, 0.0f);
    FrenetState end = makeState(100.0f, 10.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 10.0f);
    ASSERT_TRUE(traj.has_value());

    // Velocity is degree-4, squared = degree-8, exceeds MAX_DEGREE
    EXPECT_THROW(traj->velocityCost(10.0f), std::runtime_error);
}

// ============================================================================
// Cost Accessor Tests
// ============================================================================

TEST(PolynomialTrajectoryCostAccessor, DefaultCostIsMax) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Default cost should be max float (from implementation)
    EXPECT_GT(traj->cost(), 1e30f);
}

TEST(PolynomialTrajectoryCostAccessor, SetCostUpdatesValue) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(10.0f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    traj->setCost(42.5f);
    EXPECT_FLOAT_EQ(traj->cost(), 42.5f);
}

// ============================================================================
// End State Accessor Tests
// ============================================================================

TEST(PolynomialTrajectoryEndState, ReturnsCorrectEndState) {
    FrenetState start = makeState(0.0f, 5.0f, 1.0f);
    FrenetState end = makeState(20.0f, 10.0f, -2.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 3.0f);
    ASSERT_TRUE(traj.has_value());

    FrenetState resultEnd = traj->endState();
    EXPECT_NEAR(resultEnd.distance, end.distance, 1e-4f);
    EXPECT_NEAR(resultEnd.velocity, end.velocity, 1e-4f);
    EXPECT_NEAR(resultEnd.accel, end.accel, 1e-4f);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(PolynomialTrajectoryEdgeCases, VeryShortDuration) {
    FrenetState start = makeState(0.0f, 0.0f, 0.0f);
    FrenetState end = makeState(0.1f, 0.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 0.01f);
    // May or may not succeed depending on numerical stability
    if (traj.has_value()) {
        EXPECT_NEAR(traj->evaluate(0.0f), 0.0f, 1e-3f);
    }
}

TEST(PolynomialTrajectoryEdgeCases, NegativeVelocities) {
    FrenetState start = makeState(10.0f, -5.0f, 0.0f);
    FrenetState end = makeState(0.0f, -5.0f, 0.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Position should decrease
    EXPECT_GT(traj->evaluate(0.0f), traj->evaluate(2.0f));
}

TEST(PolynomialTrajectoryEdgeCases, LargeAccelerationChange) {
    FrenetState start = makeState(0.0f, 0.0f, 10.0f);
    FrenetState end = makeState(10.0f, 0.0f, -10.0f);

    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, 2.0f);
    ASSERT_TRUE(traj.has_value());

    // Should still satisfy boundary conditions
    EXPECT_NEAR(traj->acceleration(0.0f), 10.0f, 1e-4f);
    EXPECT_NEAR(traj->acceleration(2.0f), -10.0f, 1e-4f);
}
