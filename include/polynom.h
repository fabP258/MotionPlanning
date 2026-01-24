#ifndef POLYNOM_H_INCLUDED
#define POLYNOM_H_INCLUDED

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <stdexcept>

namespace Common {

template <size_t DEGREE> class Polynom {
  private:
    std::array<float, DEGREE + 1> coefficients_;

    Polynom(const std::array<float, DEGREE + 1> &coefficients)
        : coefficients_(coefficients) {
    }

  public:
    // Default constructor - creates zero polynomial
    Polynom() : coefficients_{} {
    }

    int degree() const {
        return DEGREE;
    }

    const std::array<float, DEGREE + 1> &coefficients() const {
        return coefficients_;
    }

    float evaluate(float x) const {
        float result = 0.0f;
        float xp = 1.0f; // x⁰

        for (size_t i = 0; i <= DEGREE; ++i) {
            result += coefficients_[i] * xp;
            xp *= x;
        }

        return result;
    }

    float operator()(float x) const {
        return evaluate(x);
    }

    // Returns derivative polynomial (degree n-1)
    Polynom derivative() const {
        if (DEGREE == 0) {
            // Derivative of constant is zero
            return Polynom<0>(std::array<float, 1>{});
        }

        std::array<float, DEGREE> derivCoefs{};
        for (size_t i = 0; i < DEGREE; ++i) {
            derivCoefs[i] = coefficients_[i + 1] * (i + 1);
        }

        return Polynom(derivCoefs);
    }

    // Returns nth order derivative
    Polynom derivative(size_t order) const {
        Polynom result = *this;
        for (size_t i = 0; i < order; ++i) {
            result = result.derivative();
        }
        return result;
    }

    // Returns squared polynomial: p(x)² = p(x) * p(x)
    Polynom square() const {
        std::array<float, DEGREE * 2> newCoefs{};
        // Multiply: (a₀ + a₁x + a₂x² + ...) × (a₀ + a₁x + a₂x² + ...)
        // Result coefficient for x^k is sum of aᵢ*aⱼ where i+j=k
        for (size_t i = 0; i <= DEGREE; ++i) {
            for (size_t j = 0; j <= DEGREE; ++j) {
                newCoefs[i + j] += coefficients_[i] * coefficients_[j];
            }
        }

        return Polynom(newCoefs);
    }

    // Returns definite integral over [a, b]: ∫ₐᵇ p(x) dx
    float integrateDefinite(float a, float b) const {
        float result = 0.0f;

        // For each term aᵢxⁱ, integral is aᵢ/(i+1) × x^(i+1)
        // Definite integral: [aᵢ/(i+1) × b^(i+1)] - [aᵢ/(i+1) × a^(i+1)]
        for (size_t i = 0; i <= DEGREE; ++i) {
            float coef = coefficients_[i] / (i + 1);
            result += coef * (std::pow(b, i + 1) - std::pow(a, i + 1));
        }

        return result;
    }

    // Returns antiderivative polynomial (indefinite integral)
    Polynom integrate() const {
        const size_t newDegree = DEGREE + 1;
        std::array<float, newDegree + 1> newCoefs{};

        // ∫ aᵢxⁱ dx = aᵢ/(i+1) × x^(i+1) + C
        // Integration constant C is stored in newCoefs[0] (defaults to 0)
        for (size_t i = 0; i <= DEGREE; ++i) {
            newCoefs[i + 1] = coefficients_[i] / (i + 1);
        }

        return Polynom(newCoefs);
    }

    // Subtract scalar
    Polynom operator-(float rhs) const {
        std::array<float, DEGREE + 1> newCoefs = coefficients_;
        newCoefs[0] -= rhs;
        return Polynom(newCoefs);
    }
};

} // namespace Common

#endif // POLYNOM_H_INCLUDED
