#ifndef GEOMETRY_H_INCLUDED
#define GEOMETRY_H_INCLUDED

#include "polynom.h"
#include <vector>

namespace Common {

struct Point2D {
    float x;
    float y;

    float euclidiandistance(const Point2D &other) const;
};

struct FrenetState {
    float distance;
    float velocity;
    float accel;
};

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
            throw std::invalid_argument(
                "Path2D requires degree-3 polynomials");
        }
    }

    Point2D evaluate(const float arcLength) const {
        return {poly_x.evaluate(arcLength), poly_y.evaluate(arcLength)};
    }

    void fit(const std::vector<Point2D> &referencePoints);
};

} // namespace Common

#endif // GEOMETRY_H_INCLUDED
