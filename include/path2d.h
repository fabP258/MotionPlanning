#ifndef PATH2D_H_INCLUDED
#define PATH2D_H_INCLUDED

#include "fixed_capacity_buffer.h"
#include "geometry.h"
#include "polynom.h"
#include <vector>

namespace Common {
class Path2D {
  private:
    static constexpr size_t POLYNOM_DEGREE = 3;
    using PathPolynom = Polynom<POLYNOM_DEGREE>;

    PathPolynom poly_x;
    PathPolynom poly_y;

    std::vector<float>
    calculateArcLength(const std::vector<Point2D> &points) const;

  public:
    Path2D(const PathPolynom &px, const PathPolynom &py)
        : poly_x(px), poly_y(py) {
    }

    template <int N>
    static Path2D fromPoints(const FixedCapacityBuffer<Point2D, N> &points);

    Point2D evaluate(const float arcLength) const {
        return {poly_x.evaluate(arcLength), poly_y.evaluate(arcLength)};
    }

    FrenetPoint projectPointIntoFrenet(const Point2D &point) const;
};
} // namespace Common

#endif // PATH2D_H_INCLUDED