#include "geometry.h"
#include <cmath>

namespace Common {

float Point2D::euclidiandistance(const Point2D &other) const {
    return std::sqrt(std::pow(x - other.x, 2.0f) + std::pow(y - other.y, 2.0f));
}

} // namespace Common
