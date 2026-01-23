#ifndef POLYNOM_H_INCLUDED
#define POLYNOM_H_INCLUDED

#include <algorithm>
#include <array>
#include <span>
#include <stdexcept>

namespace Common {

class Polynom {
  private:
    static constexpr int MAX_DEGREE = 5;
    std::array<float, MAX_DEGREE + 1> coefficients_;
    int degree_;

    Polynom(const std::array<float, MAX_DEGREE + 1> &coefficients, int degree)
        : coefficients_(coefficients), degree_(degree) {
    }

  public:
    // Default constructor - creates zero polynomial
    Polynom() : coefficients_{}, degree_(0) {
    }

    // Construct from std::array (for known degrees at compile time)
    template <std::size_t N>
    explicit Polynom(const std::array<float, N> &coefficients)
        : coefficients_{}, degree_(N - 1) {
        static_assert(N <= MAX_DEGREE + 1,
                      "Polynomial degree exceeds maximum supported degree");

        std::copy(coefficients.begin(), coefficients.end(), coefficients_.begin());
    }

    // Construct from initializer list for convenience
    explicit Polynom(std::initializer_list<float> coefficients);

    int degree() const {
        return degree_;
    }

    // Returns view of active coefficients only
    std::span<const float> coefficients() const {
        return std::span<const float>(coefficients_.data(), degree_ + 1);
    }

    float evaluate(float x) const;

    float operator()(float x) const {
        return evaluate(x);
    }

    // Returns derivative polynomial (degree n-1)
    Polynom derivative() const;

    // Returns nth order derivative
    Polynom derivative(int order) const;

    // Returns squared polynomial: p(x)² = p(x) * p(x)
    Polynom square() const;

    // Returns definite integral over [a, b]: ∫ₐᵇ p(x) dx
    float integrateDefinite(float a, float b) const;

    // Returns antiderivative polynomial (indefinite integral)
    Polynom integrate() const;

    // Subtract scalar
    Polynom operator-(float rhs) const;
};

} // namespace Common

#endif // POLYNOM_H_INCLUDED
