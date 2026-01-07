#include "polynom.h"
#include <cmath>

namespace Common {

Polynom::Polynom(std::initializer_list<float> coefficients)
    : degree_(coefficients.size() - 1) {
    if (coefficients.size() == 0) {
        throw std::invalid_argument(
            "Polynomial must have at least one coefficient");
    }

    if (coefficients.size() > MAX_DEGREE + 1) {
        throw std::invalid_argument("Polynomial degree exceeds maximum "
                                    "supported degree");
    }

    // Copy coefficients from initializer list
    std::size_t i = 0;
    for (float coef : coefficients) {
        coefficients_[i++] = coef;
    }

    // Zero out unused coefficients
    for (; i < MAX_DEGREE + 1; ++i) {
        coefficients_[i] = 0.0f;
    }
}

float Polynom::evaluate(float x) const {
    float result = 0.0f;
    float xp = 1.0f; // x^0

    // Evaluate polynomial using ascending order: c0 + c1*x + c2*x^2 + ...
    for (int i = 0; i <= degree_; ++i) {
        result += coefficients_[i] * xp;
        xp *= x;
    }

    return result;
}

Polynom Polynom::derivative() const {
    if (degree_ == 0) {
        // Derivative of constant is zero
        return Polynom({0.0f});
    }

    // Create array for derivative coefficients (one less degree)
    std::array<float, MAX_DEGREE + 1> derivCoefs{};

    for (int i = 0; i < degree_; ++i) {
        derivCoefs[i] = coefficients_[i + 1] * (i + 1);
    }

    return Polynom(derivCoefs, degree_ - 1);
}

Polynom Polynom::derivative(int order) const {
    if (order < 0) {
        throw std::invalid_argument("Derivative order must be non-negative");
    }

    if (order == 0) {
        return *this;
    }

    Polynom result = *this;
    for (int i = 0; i < order; ++i) {
        result = result.derivative();
    }

    return result;
}

Polynom Polynom::square() const {
    int newDegree = 2 * degree_;

    if (newDegree > MAX_DEGREE) {
        throw std::runtime_error(
            "Squared polynomial exceeds maximum degree");
    }

    std::array<float, MAX_DEGREE + 1> newCoefs{};

    // Multiply: (a₀ + a₁x + a₂x² + ...) × (a₀ + a₁x + a₂x² + ...)
    // Result coefficient for x^k is sum of aᵢ*aⱼ where i+j=k
    for (int i = 0; i <= degree_; ++i) {
        for (int j = 0; j <= degree_; ++j) {
            newCoefs[i + j] += coefficients_[i] * coefficients_[j];
        }
    }

    return Polynom(newCoefs, newDegree);
}

float Polynom::integrateDefinite(float a, float b) const {
    float result = 0.0f;

    // For each term aᵢxⁱ, integral is aᵢ/(i+1) × x^(i+1)
    // Definite integral: [aᵢ/(i+1) × b^(i+1)] - [aᵢ/(i+1) × a^(i+1)]
    for (int i = 0; i <= degree_; ++i) {
        float coef = coefficients_[i] / (i + 1);
        result += coef * (std::pow(b, i + 1) - std::pow(a, i + 1));
    }

    return result;
}

Polynom Polynom::integrate() const {
    if (degree_ >= MAX_DEGREE) {
        throw std::runtime_error(
            "Integrated polynomial would exceed maximum degree");
    }

    std::array<float, MAX_DEGREE + 1> newCoefs{};

    // ∫ aᵢxⁱ dx = aᵢ/(i+1) × x^(i+1) + C
    // Integration constant C is stored in newCoefs[0] (defaults to 0)
    for (int i = 0; i <= degree_; ++i) {
        newCoefs[i + 1] = coefficients_[i] / (i + 1);
    }

    return Polynom(newCoefs, degree_ + 1);
}

} // namespace Common
