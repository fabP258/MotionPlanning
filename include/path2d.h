#ifndef PATH2D_H_INCLUDED
#define PATH2D_H_INCLUDED

#include "fixed_capacity_buffer.h"
#include "geometry.h"
#include "polynom.h"
#include <vector>

namespace Common {
class Path2D {
  private:
    static constexpr int POLYNOM_DEGREE = 3;
    Polynom poly_x;
    Polynom poly_y;

    std::vector<float>
    calculateArcLength(const std::vector<Point2D> &points) const;

  public:
    Path2D(const Polynom &px, const Polynom &py) : poly_x(px), poly_y(py) {
        // Verify that polynomials are degree 3
        if (px.degree() != POLYNOM_DEGREE || py.degree() != POLYNOM_DEGREE) {
            throw std::invalid_argument("Path2D requires degree-3 polynomials");
        }
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