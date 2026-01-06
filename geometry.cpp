#include "geometry.h"
#include "linalg.h"
#include <cmath>

namespace Common {

float Point2D::euclidiandistance(const Point2D &other) const {
    return std::sqrt(std::pow(x - other.x, 2.0f) + std::pow(y - other.y, 2.0f));
}

std::optional<Common::TPolynom<float, 5>>
solveBoundaryValueProblem(const FrenetState &startState,
                          const FrenetState &endState, const float endTime) {
    std::array<float, 6> coefs;

    // from left boundary
    coefs[0] = startState.distance;
    coefs[1] = startState.velocity;
    coefs[2] = startState.accel;

    // from right boundary
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

    return {coefs};
}

std::optional<Common::TPolynom<float, 5>>
solveBoundaryValueProblem(const FrenetState &startState,
                          const float endVelocity, const float endAcceleration,
                          const float endTime) {
    std::array<float, 6> coefs;
    coefs[5] = 0.0f;

    // left boundary
    coefs[0] = startState.distance;
    coefs[1] = startState.velocity;
    coefs[2] = 0.5f * startState.accel;

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

    return {coefs};
}

std::vector<float>
Path2D::calculateArcLength(const std::vector<Point2D> &points) const {
    if (points.size() == 0) {
        return {};
    }

    std::vector<float> arcLength(points.size(), 0.0f);
    for (int i = 1; i < points.size(); i++) {
        arcLength[i] =
            arcLength[i - 1] + points[i].euclidiandistance(points[i - 1]);
    }

    return arcLength;
}

void Path2D::fit(const std::vector<Point2D> &referencePoints) {
    if (referencePoints.size() < POLYNOM_DEGREE + 1) {
        return;
    }

    std::vector<float> arcLength = calculateArcLength(referencePoints);

    // TODO: fit polynoms x(s) and y(s)
}

} // namespace Common