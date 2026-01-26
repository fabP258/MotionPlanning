#include "polynomial_spline_trajectory.h"
#include <cmath>
#include <gtest/gtest.h>

using Common::FrenetState;
using Common::PolynomialSplineTrajectory;
using Common::PolynomialTrajectory;

// ============================================================================
// Helper Functions
// ============================================================================

FrenetState makeFrenetState(float d, float v, float a) {
    FrenetState s;
    s.distance = d;
    s.velocity = v;
    s.accel = a;
    return s;
}

PolynomialTrajectory createSegment(float startPos, float startVel,
                                   float startAccel, float endPos, float endVel,
                                   float endAccel, float duration) {
    FrenetState start = makeFrenetState(startPos, startVel, startAccel);
    FrenetState end = makeFrenetState(endPos, endVel, endAccel);
    auto traj = PolynomialTrajectory::fromBoundaryStates(start, end, duration);
    return traj.value();
}

// ============================================================================
// Single Segment Tests
// ============================================================================

TEST(PolynomialSplineTrajectory, SingleSegmentEvaluateAtStart) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    auto result = spline.evaluate(0.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 0.0f, 1e-5f);
}

TEST(PolynomialSplineTrajectory, SingleSegmentEvaluateAtEnd) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    auto result = spline.evaluate(2.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 10.0f, 1e-4f);
}

TEST(PolynomialSplineTrajectory, SingleSegmentEvaluateMidpoint) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    auto result = spline.evaluate(1.0f);
    ASSERT_TRUE(result.has_value());
    // For symmetric boundary conditions, midpoint should be at half distance
    EXPECT_NEAR(*result, 5.0f, 1e-4f);
}

TEST(PolynomialSplineTrajectory, SingleSegmentNegativeTimeReturnsNullopt) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    EXPECT_FALSE(spline.evaluate(-0.1f).has_value());
}

TEST(PolynomialSplineTrajectory, SingleSegmentBeyondEndReturnsNullopt) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    std::array<PolynomialTrajectory, 1> segments = {segment};
    PolynomialSplineTrajectory<1> spline(segments);

    EXPECT_FALSE(spline.evaluate(2.1f).has_value());
}

// ============================================================================
// Multiple Segment Tests - evaluate
// ============================================================================

TEST(PolynomialSplineTrajectory, TwoSegmentsEvaluateFirstSegment) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 5.0f, 0.0f, 2.0f);
    auto seg2 = createSegment(10.0f, 5.0f, 0.0f, 30.0f, 5.0f, 0.0f, 4.0f);
    std::array<PolynomialTrajectory, 2> segments = {seg1, seg2};
    PolynomialSplineTrajectory<2> spline(segments);

    auto result = spline.evaluate(1.0f);
    ASSERT_TRUE(result.has_value());
    // Should match first segment at t=1.0
    EXPECT_FLOAT_EQ(*result, seg1.evaluate(1.0f));
}

TEST(PolynomialSplineTrajectory, TwoSegmentsEvaluateSecondSegment) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 5.0f, 0.0f, 2.0f);
    auto seg2 = createSegment(10.0f, 5.0f, 0.0f, 30.0f, 5.0f, 0.0f, 4.0f);
    PolynomialSplineTrajectory<2> spline({seg1, seg2});

    // t=4.0 is 2 seconds into the second segment
    auto result = spline.evaluate(4.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(*result, seg2.evaluate(2.0f));
}

TEST(PolynomialSplineTrajectory, TwoSegmentsEvaluateAtBoundary) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 5.0f, 0.0f, 2.0f);
    auto seg2 = createSegment(10.0f, 5.0f, 0.0f, 30.0f, 5.0f, 0.0f, 4.0f);
    PolynomialSplineTrajectory<2> spline({seg1, seg2});

    // At t=2.0 (boundary), should be in first segment at its end
    auto result = spline.evaluate(2.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 10.0f, 1e-4f);
}

TEST(PolynomialSplineTrajectory, TwoSegmentsTotalDuration) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 5.0f, 0.0f, 2.0f);
    auto seg2 = createSegment(10.0f, 5.0f, 0.0f, 30.0f, 5.0f, 0.0f, 4.0f);
    PolynomialSplineTrajectory<2> spline({seg1, seg2});

    // Total duration is 2 + 4 = 6
    EXPECT_TRUE(spline.evaluate(6.0f).has_value());
    EXPECT_FALSE(spline.evaluate(6.1f).has_value());
}

TEST(PolynomialSplineTrajectory, ThreeSegmentsTraversal) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 5.0f, 2.0f, 0.0f, 1.0f);
    auto seg2 = createSegment(5.0f, 2.0f, 0.0f, 15.0f, 4.0f, 0.0f, 2.0f);
    auto seg3 = createSegment(15.0f, 4.0f, 0.0f, 30.0f, 0.0f, 0.0f, 3.0f);
    PolynomialSplineTrajectory<3> spline({seg1, seg2, seg3});

    // Check start
    auto atStart = spline.evaluate(0.0f);
    ASSERT_TRUE(atStart.has_value());
    EXPECT_NEAR(*atStart, 0.0f, 1e-5f);

    // Check end of first segment (t=1.0)
    auto atSeg1End = spline.evaluate(1.0f);
    ASSERT_TRUE(atSeg1End.has_value());
    EXPECT_NEAR(*atSeg1End, 5.0f, 1e-4f);

    // Check end of second segment (t=3.0)
    auto atSeg2End = spline.evaluate(3.0f);
    ASSERT_TRUE(atSeg2End.has_value());
    EXPECT_NEAR(*atSeg2End, 15.0f, 1e-4f);

    // Check end of third segment (t=6.0)
    auto atEnd = spline.evaluate(6.0f);
    ASSERT_TRUE(atEnd.has_value());
    EXPECT_NEAR(*atEnd, 30.0f, 1e-4f);
}

// ============================================================================
// evaluateState Tests
// ============================================================================

TEST(PolynomialSplineTrajectory, EvaluateStateAtStart) {
    auto segment = createSegment(5.0f, 3.0f, 1.0f, 20.0f, 6.0f, -1.0f, 4.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    auto state = spline.evaluateState(0.0f);
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->distance, 5.0f, 1e-5f);
    EXPECT_NEAR(state->velocity, 3.0f, 1e-5f);
    EXPECT_NEAR(state->accel, 1.0f, 1e-5f);
}

TEST(PolynomialSplineTrajectory, EvaluateStateAtEnd) {
    auto segment = createSegment(5.0f, 3.0f, 1.0f, 20.0f, 6.0f, -1.0f, 4.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    auto state = spline.evaluateState(4.0f);
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->distance, 20.0f, 1e-4f);
    EXPECT_NEAR(state->velocity, 6.0f, 1e-4f);
    EXPECT_NEAR(state->accel, -1.0f, 1e-4f);
}

TEST(PolynomialSplineTrajectory, EvaluateStateNegativeTimeReturnsNullopt) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    EXPECT_FALSE(spline.evaluateState(-0.5f).has_value());
}

TEST(PolynomialSplineTrajectory, EvaluateStateBeyondEndReturnsNullopt) {
    auto segment = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    EXPECT_FALSE(spline.evaluateState(3.0f).has_value());
}

TEST(PolynomialSplineTrajectory, EvaluateStateTwoSegments) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 10.0f, 5.0f, 2.0f, 2.0f);
    auto seg2 = createSegment(10.0f, 5.0f, 2.0f, 40.0f, 10.0f, 0.0f, 3.0f);
    PolynomialSplineTrajectory<2> spline({seg1, seg2});

    // Query in second segment at local time 1.5 (global time 3.5)
    auto splineState = spline.evaluateState(3.5f);
    auto directState = seg2.evaluateState(1.5f);

    ASSERT_TRUE(splineState.has_value());
    ASSERT_TRUE(directState.has_value());
    EXPECT_FLOAT_EQ(splineState->distance, directState->distance);
    EXPECT_FLOAT_EQ(splineState->velocity, directState->velocity);
    EXPECT_FLOAT_EQ(splineState->accel, directState->accel);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(PolynomialSplineTrajectory, ZeroTimeQuery) {
    auto segment = createSegment(5.0f, 2.0f, 1.0f, 15.0f, 4.0f, 0.0f, 3.0f);
    PolynomialSplineTrajectory<1> spline({segment});

    auto result = spline.evaluate(0.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 5.0f, 1e-5f);
}

TEST(PolynomialSplineTrajectory, VerySmallTimeStep) {
    auto segment = createSegment(0.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f, 1.0f);
    std::array<PolynomialTrajectory, 1> segments = {segment};
    PolynomialSplineTrajectory<1> spline(segments);

    // Query at very small time
    auto result = spline.evaluate(0.001f);
    ASSERT_TRUE(result.has_value());
    // Should be approximately 0 + 10 * 0.001 = 0.01 for constant velocity
    EXPECT_NEAR(*result, 0.01f, 1e-3f);
}

TEST(PolynomialSplineTrajectory, StationarySpline) {
    auto seg1 = createSegment(5.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 1.0f);
    auto seg2 = createSegment(5.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 1.0f);
    std::array<PolynomialTrajectory, 2> segments = {seg1, seg2};
    PolynomialSplineTrajectory<2> spline(segments);

    // Position should be constant throughout
    for (float t = 0.0f; t <= 2.0f; t += 0.25f) {
        auto result = spline.evaluate(t);
        ASSERT_TRUE(result.has_value());
        EXPECT_NEAR(*result, 5.0f, 1e-5f);
    }
}

TEST(PolynomialSplineTrajectory, NegativeVelocitySegments) {
    // Moving backwards
    auto seg1 = createSegment(20.0f, -5.0f, 0.0f, 10.0f, -5.0f, 0.0f, 2.0f);
    auto seg2 = createSegment(10.0f, -5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    std::array<PolynomialTrajectory, 2> segments = {seg1, seg2};
    PolynomialSplineTrajectory<2> spline(segments);

    auto atStart = spline.evaluate(0.0f);
    auto atMiddle = spline.evaluate(2.0f);
    auto atEnd = spline.evaluate(4.0f);

    ASSERT_TRUE(atStart.has_value());
    ASSERT_TRUE(atMiddle.has_value());
    ASSERT_TRUE(atEnd.has_value());

    // Position should decrease
    EXPECT_GT(*atStart, *atMiddle);
    EXPECT_GT(*atMiddle, *atEnd);
}

TEST(PolynomialSplineTrajectory, SegmentsWithDifferentDurations) {
    auto seg1 = createSegment(0.0f, 0.0f, 0.0f, 5.0f, 2.0f, 0.0f, 1.0f);
    auto seg2 = createSegment(5.0f, 2.0f, 0.0f, 25.0f, 4.0f, 0.0f, 5.0f);
    auto seg3 = createSegment(25.0f, 4.0f, 0.0f, 30.0f, 0.0f, 0.0f, 0.5f);
    std::array<PolynomialTrajectory, 3> segments = {seg1, seg2, seg3};
    PolynomialSplineTrajectory<3> spline(segments);

    // Total duration: 1 + 5 + 0.5 = 6.5
    EXPECT_TRUE(spline.evaluate(6.5f).has_value());
    EXPECT_FALSE(spline.evaluate(6.6f).has_value());

    // Check correct segment is accessed
    // t=0.5 -> first segment
    EXPECT_TRUE(spline.evaluate(0.5f).has_value());
    // t=3.0 -> second segment (local time 2.0)
    auto atSeg2 = spline.evaluate(3.0f);
    ASSERT_TRUE(atSeg2.has_value());
    EXPECT_FLOAT_EQ(*atSeg2, seg2.evaluate(2.0f));
    // t=6.25 -> third segment (local time 0.25)
    auto atSeg3 = spline.evaluate(6.25f);
    ASSERT_TRUE(atSeg3.has_value());
    EXPECT_FLOAT_EQ(*atSeg3, seg3.evaluate(0.25f));
}

// ============================================================================
// State Consistency Tests
// ============================================================================

TEST(PolynomialSplineTrajectory, EvaluateAndEvaluateStateConsistent) {
    auto segment = createSegment(0.0f, 5.0f, 1.0f, 20.0f, 8.0f, -1.0f, 3.0f);
    std::array<PolynomialTrajectory, 1> segments = {segment};
    PolynomialSplineTrajectory<1> spline(segments);

    // Check that evaluate() returns the distance component of evaluateState()
    for (float t = 0.0f; t <= 3.0f; t += 0.5f) {
        auto evalResult = spline.evaluate(t);
        auto stateResult = spline.evaluateState(t);

        ASSERT_TRUE(evalResult.has_value());
        ASSERT_TRUE(stateResult.has_value());
        EXPECT_FLOAT_EQ(*evalResult, stateResult->distance);
    }
}

TEST(PolynomialSplineTrajectory, MultiSegmentStateConsistency) {
    auto seg1 = createSegment(0.0f, 2.0f, 0.0f, 10.0f, 4.0f, 1.0f, 2.0f);
    auto seg2 = createSegment(10.0f, 4.0f, 1.0f, 30.0f, 6.0f, 0.0f, 3.0f);
    std::array<PolynomialTrajectory, 2> segments = {seg1, seg2};
    PolynomialSplineTrajectory<2> spline(segments);

    // Sample across both segments
    for (float t = 0.0f; t <= 5.0f; t += 0.5f) {
        auto evalResult = spline.evaluate(t);
        auto stateResult = spline.evaluateState(t);

        ASSERT_TRUE(evalResult.has_value());
        ASSERT_TRUE(stateResult.has_value());
        EXPECT_FLOAT_EQ(*evalResult, stateResult->distance);
    }
}
