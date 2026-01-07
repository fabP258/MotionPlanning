#ifndef POLYNOM_H_INCLUDED
#define POLYNOM_H_INCLUDED

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
    // Construct from std::array (for known degrees at compile time)
    template <std::size_t N>
    explicit Polynom(const std::array<float, N> &coefficients)
        : degree_(N - 1) {
        static_assert(N <= MAX_DEGREE + 1,
                      "Polynomial degree exceeds maximum supported degree");

        // Copy coefficients to internal array
        for (std::size_t i = 0; i < N; ++i) {
            coefficients_[i] = coefficients[i];
        }

        // Zero out unused coefficients
        for (std::size_t i = N; i < MAX_DEGREE + 1; ++i) {
            coefficients_[i] = 0.0f;
        }
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
};

} // namespace Common

#endif // POLYNOM_H_INCLUDED
