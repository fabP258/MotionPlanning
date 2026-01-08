#include "polynomial_trajectory.h"
#include "geometry.h"
#include "linalg.h"
#include <cmath>
#include <limits>

namespace Common {

PolynomialTrajectory::PolynomialTrajectory(Polynom poly, FrenetState start,
                                           FrenetState end, float time,
                                           bool fullEndState)
    : polynom_(poly), startState_(start), endState_(end), endTime_(time),
      hasFullEndState_(fullEndState), cost_(std::numeric_limits<float>::max()) {
}

std::optional<PolynomialTrajectory> PolynomialTrajectory::fromBoundaryStates(
    const FrenetState &startState, const FrenetState &endState, float endTime) {
    std::array<float, 6> coefs;

    // From left boundary
    coefs[0] = startState.distance;
    coefs[1] = startState.velocity;
    coefs[2] = 0.5f * startState.accel;

    // From right boundary - solve 3x3 system
    Linalg::Matrix3x3f A;

    // TODO: A and therefore Ainv only depends on endTime and should be
    // calculated once!
    A[0][0] = std::pow(endTime, 3.0f);
    A[0][1] = std::pow(endTime, 4.0f);
    A[0][2] = std::pow(endTime, 5.0f);

    A[1][0] = 3.0f * std::pow(endTime, 2.0f);
    A[1][1] = 4.0f * std::pow(endTime, 3.0f);
    A[1][2] = 5.0f * std::pow(endTime, 4.0f);

    A[2][0] = 6.0f * endTime;
    A[2][1] = 12.0f * std::pow(endTime, 2.0f);
    A[2][2] = 20.0f * std::pow(endTime, 3.0f);

    std::optional<Linalg::Matrix3x3f> Ainv = Linalg::inverse(A);
    if (!Ainv)
        return std::nullopt;

    Linalg::Vector3f b;

    b[0] = endState.distance - startState.distance -
           startState.velocity * endTime - coefs[2] * std::pow(endTime, 2.0f);
    b[1] = endState.velocity - startState.velocity - startState.accel * endTime;
    b[2] = endState.accel - startState.accel;

    Linalg::Vector3f highCoefs = Linalg::multiply(*Ainv, b);
    coefs[3] = highCoefs[0];
    coefs[4] = highCoefs[1];
    coefs[5] = highCoefs[2];

    // Create polynomial and trajectory
    Polynom poly(coefs);
    return PolynomialTrajectory(poly, startState, endState, endTime, true);
}

std::optional<PolynomialTrajectory>
PolynomialTrajectory::fromStartStateAndEndVelocity(
    const FrenetState &startState, float endVelocity, float endAcceleration,
    float endTime) {
    std::array<float, 6> coefs;
    coefs[5] = 0.0f;

    // Left boundary
    coefs[0] = startState.distance;
    coefs[1] = startState.velocity;
    coefs[2] = 0.5f * startState.accel;

    // Solve 2x2 system for velocity and acceleration constraints
    Linalg::Matrix2x2f A;

    A[0][0] = 3.0f * std::pow(endTime, 2.0f);
    A[0][1] = 4.0f * std::pow(endTime, 3.0f);

    A[1][0] = 6.0f * endTime;
    A[1][1] = 12.0f * std::pow(endTime, 2.0f);

    std::optional<Linalg::Matrix2x2f> Ainv = Linalg::inverse(A);
    if (!Ainv)
        return std::nullopt;

    Linalg::Vector2f b;

    b[0] = endVelocity - startState.velocity - startState.accel * endTime;
    b[1] = endAcceleration - startState.accel;

    Linalg::Vector2f highCoefs = Linalg::multiply(*Ainv, b);
    coefs[3] = highCoefs[0];
    coefs[4] = highCoefs[1];

    // Create polynomial and trajectory
    Polynom poly(coefs);

    // Create end state for storage (position will be computed from polynomial)
    FrenetState endState;
    endState.distance = poly.evaluate(endTime);
    endState.velocity = endVelocity;
    endState.accel = endAcceleration;

    return PolynomialTrajectory(poly, startState, endState, endTime, false);
}

bool PolynomialTrajectory::isInValidRange(float t) const {
    return t >= 0 && t <= endTime_;
}

std::optional<FrenetState> PolynomialTrajectory::evaluateState(float t) const {
    if (!isInValidRange(t))
        return std::nullopt;

    Polynom velocityPolynom = polynom_.derivative();
    Polynom accelerationPolynom = velocityPolynom.derivative();

    FrenetState state;
    state.distance = evaluate(t);
    state.velocity = velocityPolynom.evaluate(t);
    state.accel = accelerationPolynom.evaluate(t);

    return state;
}

bool PolynomialTrajectory::isMaxAccelerationBelowLimit(
    const float maxAcceleration) const {

    // Get acceleration polynomial (2nd derivative)
    Polynom accel = polynom_.derivative(2);

    // Get jerk polynomial (3rd derivative) to find critical points
    Polynom jerk = polynom_.derivative(3);

    // Collect all time points where we need to check acceleration
    // Max 4 points: 2 boundaries + 2 quadratic roots
    FixedCapacityBuffer<float, 4> checkPoints;

    // Always check boundaries
    checkPoints.push_back(0.0f);
    checkPoints.push_back(endTime_);

    // Find critical points: roots of jerk polynomial where acceleration has
    // extrema
    auto jerkCoefs = jerk.coefficients();

    // Handle linear jerk (degree 1) or degenerate quadratic (degree 2 but a ≈
    // 0)
    if (jerk.degree() == 1 ||
        (jerk.degree() == 2 && std::abs(jerkCoefs[2]) < 1e-9f)) {
        // Linear jerk: at + b = 0 → t = -b/a
        float a = jerkCoefs[1]; // coefficient of t
        float b = jerkCoefs[0]; // constant term

        if (std::abs(a) > 1e-9f) { // Avoid division by near-zero
            float t = -b / a;
            if (t > 0.0f && t < endTime_) {
                checkPoints.push_back(t);
            }
        }
    } else if (jerk.degree() == 2) {
        // Quadratic: at² + bt + c = 0
        float a = jerkCoefs[2]; // coefficient of t²
        float b = jerkCoefs[1]; // coefficient of t
        float c = jerkCoefs[0]; // constant term

        float discriminant = b * b - 4 * a * c;

        if (discriminant >= 0) {
            // Real roots exist
            float sqrtDisc = std::sqrt(discriminant);
            float t1 = (-b + sqrtDisc) / (2 * a);
            float t2 = (-b - sqrtDisc) / (2 * a);

            // Only consider roots within trajectory time bounds (open interval)
            if (t1 > 0.0f && t1 < endTime_) {
                checkPoints.push_back(t1);
            }
            if (t2 > 0.0f && t2 < endTime_) {
                checkPoints.push_back(t2);
            }
        }
    }
    // If jerk.degree() == 0, acceleration is linear, no interior extrema

    // Check if absolute acceleration at all critical points is below limit
    for (const auto& timePoint : checkPoints) {
        float accelValue = accel.evaluate(timePoint);
        if (std::abs(accelValue) > maxAcceleration) {
            return false;
        }
    }

    return true;
}

bool PolynomialTrajectory::isMaxJerkBelowLimit(const float maxJerk) const {
    Polynom jerk = polynom_.derivative(3);
    Polynom snap = jerk.derivative();

    FixedCapacityBuffer<float, 3> checkPoints;

    // Always check boundaries
    checkPoints.push_back(0.0f);
    checkPoints.push_back(endTime_);

    if (snap.degree() == 1) {
        auto snapCoefs = snap.coefficients();
        if (std::abs(snapCoefs[1]) >= 1e-9f) {
            float t = -snapCoefs[0] / snapCoefs[1];
            if (t > 0 && t < endTime_) {
                checkPoints.push_back(t);
            }
        }
    }
    // If snap.degree() == 0, jerk is linear, no interior extrema

    for (const auto& timePoint : checkPoints) {
        float jerkValue = jerk.evaluate(timePoint);
        if (std::abs(jerkValue) > maxJerk) {
            return false;
        }
    }

    return true;
}

} // namespace Common
