#include "path2d.h"

namespace Common {

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

FrenetPoint Path2D::projectPointIntoFrenet(const Point2D &point) const {
    FrenetPoint transformedPoint;
    // TODO: implement transformation
    return transformedPoint;
}

} // namespace Common