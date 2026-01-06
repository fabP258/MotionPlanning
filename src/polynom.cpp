#include "polynom.h"

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

    // Create new polynomial with degree-1 coefficients
    // We need to construct it properly based on the actual degree
    if (degree_ == 1) {
        return Polynom({derivCoefs[0]});
    } else if (degree_ == 2) {
        return Polynom(std::array<float, 2>{derivCoefs[0], derivCoefs[1]});
    } else if (degree_ == 3) {
        return Polynom(std::array<float, 3>{derivCoefs[0], derivCoefs[1],
                                             derivCoefs[2]});
    } else if (degree_ == 4) {
        return Polynom(std::array<float, 4>{
            derivCoefs[0], derivCoefs[1], derivCoefs[2], derivCoefs[3]});
    } else { // degree_ == 5
        return Polynom(std::array<float, 5>{derivCoefs[0], derivCoefs[1],
                                             derivCoefs[2], derivCoefs[3],
                                             derivCoefs[4]});
    }
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

} // namespace Common
