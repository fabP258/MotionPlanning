#include "geometry.h"
#include <cmath>

namespace Common {

float Point2D::euclidiandistance(const Point2D &other) const {
    return std::sqrt(std::pow(x - other.x, 2.0f) + std::pow(y - other.y, 2.0f));
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
