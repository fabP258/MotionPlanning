#include "polynomial_trajectory.h"
#include "geometry.h"
#include "linalg.h"
#include <cmath>

namespace Common {

PolynomialTrajectory::PolynomialTrajectory(Polynom poly, FrenetState start,
                                           FrenetState end, float time,
                                           bool fullEndState)
    : polynom_(poly), startState_(start), endState_(end), endTime_(time),
      hasFullEndState_(fullEndState) {
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

} // namespace Common
