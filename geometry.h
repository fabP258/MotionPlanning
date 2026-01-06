#ifndef GEOMETRY_H_INCLUDED
#define GEOMETRY_H_INCLUDED

#include <array>
#include <optional>
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

template <typename T, int DEGREE> struct TPolynom {
  private:
    using CoefficientArray = std::array<T, DEGREE + 1>;

    // polynom coefficients in ascending order
    CoefficientArray coefficients;

  public:
    TPolynom() = default;
    TPolynom(const CoefficientArray &coefs) : coefficients(coefs) {
    }

    T evaluate(T x) const {
        T result = T();
        T xp = static_cast<T>(1);
        for (const T &coef : coefficients) {
            result += coef * xp;
            xp *= x;
        }
        return result;
    }

    CoefficientArray getCoefficients() const {
        return coefficients;
    }
};

std::optional<Common::TPolynom<float, 5>>
solveBoundaryValueProblem(const FrenetState &startState,
                          const FrenetState &endState, const float endTime);

std::optional<Common::TPolynom<float, 5>>
solveBoundaryValueProblem(const FrenetState &startState,
                          const float endVelocity, const float endAcceleration,
                          const float endTime);

class Path2D {
  private:
    static constexpr int POLYNOM_DEGREE = 3;
    TPolynom<float, POLYNOM_DEGREE> poly_x;
    TPolynom<float, POLYNOM_DEGREE> poly_y;

    std::vector<float>
    calculateArcLength(const std::vector<Point2D> &points) const;

  public:
    Path2D(const TPolynom<float, POLYNOM_DEGREE> &px,
           const TPolynom<float, POLYNOM_DEGREE> &py)
        : poly_x(px), poly_y(py) {
    }

    Point2D evaluate(const float arcLength) const {
        return {poly_x.evaluate(arcLength), poly_y.evaluate(arcLength)};
    }

    void fit(const std::vector<Point2D> &referencePoints);
};

} // namespace Common

#endif // GEOMETRY_H_INCLUDED