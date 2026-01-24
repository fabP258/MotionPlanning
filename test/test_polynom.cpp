#include "polynom.h"
#include <gtest/gtest.h>
#include <cmath>

using Common::Polynom;

// ============================================================================
// Construction Tests
// ============================================================================

TEST(PolynomConstructor, DefaultConstructorCreatesZeroPolynomial) {
    Polynom p;
    EXPECT_EQ(p.degree(), 0);
    EXPECT_FLOAT_EQ(p(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(p(1.0f), 0.0f);
    EXPECT_FLOAT_EQ(p(5.0f), 0.0f);
}

TEST(PolynomConstructor, InitializerListConstant) {
    Polynom p({3.0f});
    EXPECT_EQ(p.degree(), 0);
    EXPECT_FLOAT_EQ(p(0.0f), 3.0f);
    EXPECT_FLOAT_EQ(p(100.0f), 3.0f);
}

TEST(PolynomConstructor, InitializerListLinear) {
    // p(x) = 2 + 3x
    Polynom p({2.0f, 3.0f});
    EXPECT_EQ(p.degree(), 1);
    EXPECT_FLOAT_EQ(p(0.0f), 2.0f);
    EXPECT_FLOAT_EQ(p(1.0f), 5.0f);
    EXPECT_FLOAT_EQ(p(2.0f), 8.0f);
}

TEST(PolynomConstructor, InitializerListQuadratic) {
    // p(x) = 1 + 2x + 3x^2
    Polynom p({1.0f, 2.0f, 3.0f});
    EXPECT_EQ(p.degree(), 2);
    EXPECT_FLOAT_EQ(p(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(p(1.0f), 6.0f);   // 1 + 2 + 3
    EXPECT_FLOAT_EQ(p(2.0f), 17.0f);  // 1 + 4 + 12
}

TEST(PolynomConstructor, StdArrayConstruction) {
    std::array<float, 3> coeffs = {1.0f, 0.0f, 1.0f};  // p(x) = 1 + x^2
    Polynom p(coeffs);
    EXPECT_EQ(p.degree(), 2);
    EXPECT_FLOAT_EQ(p(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(p(2.0f), 5.0f);   // 1 + 4
    EXPECT_FLOAT_EQ(p(3.0f), 10.0f);  // 1 + 9
}

TEST(PolynomConstructor, MaxDegreePolynomial) {
    // p(x) = 1 + x + x^2 + x^3 + x^4 + x^5 (degree 5 is max)
    Polynom p({1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f});
    EXPECT_EQ(p.degree(), 5);
    EXPECT_FLOAT_EQ(p(1.0f), 6.0f);
}

TEST(PolynomConstructor, EmptyInitializerListThrows) {
    EXPECT_THROW(Polynom({}), std::invalid_argument);
}

TEST(PolynomConstructor, ExceedsMaxDegreeThrows) {
    EXPECT_THROW(Polynom({1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}),
                 std::invalid_argument);
}

// ============================================================================
// Coefficient Access Tests
// ============================================================================

TEST(PolynomCoefficients, ReturnsCorrectSpan) {
    Polynom p({1.0f, 2.0f, 3.0f});
    auto coeffs = p.coefficients();
    EXPECT_EQ(coeffs.size(), 3u);
    EXPECT_FLOAT_EQ(coeffs[0], 1.0f);
    EXPECT_FLOAT_EQ(coeffs[1], 2.0f);
    EXPECT_FLOAT_EQ(coeffs[2], 3.0f);
}

TEST(PolynomCoefficients, ConstantPolynomialHasOneCoefficient) {
    Polynom p({5.0f});
    auto coeffs = p.coefficients();
    EXPECT_EQ(coeffs.size(), 1u);
    EXPECT_FLOAT_EQ(coeffs[0], 5.0f);
}

// ============================================================================
// Evaluation Tests
// ============================================================================

TEST(PolynomEvaluate, EvaluateAndOperatorAreEquivalent) {
    Polynom p({1.0f, 2.0f, 3.0f});
    EXPECT_FLOAT_EQ(p.evaluate(2.5f), p(2.5f));
}

TEST(PolynomEvaluate, NegativeInput) {
    // p(x) = 1 + 2x + x^2
    Polynom p({1.0f, 2.0f, 1.0f});
    EXPECT_FLOAT_EQ(p(-1.0f), 0.0f);  // 1 - 2 + 1 = 0
    EXPECT_FLOAT_EQ(p(-2.0f), 1.0f);  // 1 - 4 + 4 = 1
}

TEST(PolynomEvaluate, ZeroCoefficients) {
    // p(x) = x^2 (coefficients: 0, 0, 1)
    Polynom p({0.0f, 0.0f, 1.0f});
    EXPECT_FLOAT_EQ(p(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(p(3.0f), 9.0f);
}

// ============================================================================
// Derivative Tests
// ============================================================================

TEST(PolynomDerivative, ConstantDerivativeIsZero) {
    Polynom p({5.0f});
    Polynom dp = p.derivative();
    EXPECT_EQ(dp.degree(), 0);
    EXPECT_FLOAT_EQ(dp(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(dp(100.0f), 0.0f);
}

TEST(PolynomDerivative, LinearDerivativeIsConstant) {
    // p(x) = 3 + 4x  =>  p'(x) = 4
    Polynom p({3.0f, 4.0f});
    Polynom dp = p.derivative();
    EXPECT_EQ(dp.degree(), 0);
    EXPECT_FLOAT_EQ(dp(0.0f), 4.0f);
    EXPECT_FLOAT_EQ(dp(10.0f), 4.0f);
}

TEST(PolynomDerivative, QuadraticDerivativeIsLinear) {
    // p(x) = 1 + 2x + 3x^2  =>  p'(x) = 2 + 6x
    Polynom p({1.0f, 2.0f, 3.0f});
    Polynom dp = p.derivative();
    EXPECT_EQ(dp.degree(), 1);
    EXPECT_FLOAT_EQ(dp(0.0f), 2.0f);
    EXPECT_FLOAT_EQ(dp(1.0f), 8.0f);
    EXPECT_FLOAT_EQ(dp(2.0f), 14.0f);
}

TEST(PolynomDerivative, CubicDerivative) {
    // p(x) = 1 + x + x^2 + x^3  =>  p'(x) = 1 + 2x + 3x^2
    Polynom p({1.0f, 1.0f, 1.0f, 1.0f});
    Polynom dp = p.derivative();
    EXPECT_EQ(dp.degree(), 2);
    EXPECT_FLOAT_EQ(dp(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(dp(1.0f), 6.0f);   // 1 + 2 + 3
}

TEST(PolynomDerivative, NthOrderDerivativeZeroReturnsOriginal) {
    Polynom p({1.0f, 2.0f, 3.0f});
    Polynom d0 = p.derivative(0);
    EXPECT_EQ(d0.degree(), p.degree());
    EXPECT_FLOAT_EQ(d0(2.0f), p(2.0f));
}

TEST(PolynomDerivative, NthOrderDerivative) {
    // p(x) = 1 + 2x + 3x^2 + 4x^3
    // p'(x) = 2 + 6x + 12x^2
    // p''(x) = 6 + 24x
    // p'''(x) = 24
    Polynom p({1.0f, 2.0f, 3.0f, 4.0f});

    Polynom d1 = p.derivative(1);
    EXPECT_EQ(d1.degree(), 2);
    EXPECT_FLOAT_EQ(d1(0.0f), 2.0f);

    Polynom d2 = p.derivative(2);
    EXPECT_EQ(d2.degree(), 1);
    EXPECT_FLOAT_EQ(d2(0.0f), 6.0f);
    EXPECT_FLOAT_EQ(d2(1.0f), 30.0f);

    Polynom d3 = p.derivative(3);
    EXPECT_EQ(d3.degree(), 0);
    EXPECT_FLOAT_EQ(d3(0.0f), 24.0f);
}

TEST(PolynomDerivative, HighOrderDerivativeBecomesZero) {
    Polynom p({1.0f, 2.0f, 3.0f});  // degree 2
    Polynom d5 = p.derivative(5);    // derivative order > degree
    EXPECT_EQ(d5.degree(), 0);
    EXPECT_FLOAT_EQ(d5(0.0f), 0.0f);
}

TEST(PolynomDerivative, NegativeOrderThrows) {
    Polynom p({1.0f, 2.0f});
    EXPECT_THROW(p.derivative(-1), std::invalid_argument);
}

// ============================================================================
// Square Tests
// ============================================================================

TEST(PolynomSquare, ConstantSquare) {
    Polynom p({3.0f});
    Polynom sq = p.square();
    EXPECT_EQ(sq.degree(), 0);
    EXPECT_FLOAT_EQ(sq(0.0f), 9.0f);
}

TEST(PolynomSquare, LinearSquare) {
    // p(x) = 1 + x  =>  p(x)^2 = 1 + 2x + x^2
    Polynom p({1.0f, 1.0f});
    Polynom sq = p.square();
    EXPECT_EQ(sq.degree(), 2);
    EXPECT_FLOAT_EQ(sq(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(sq(1.0f), 4.0f);   // (1+1)^2 = 4
    EXPECT_FLOAT_EQ(sq(2.0f), 9.0f);   // (1+2)^2 = 9
}

TEST(PolynomSquare, QuadraticSquare) {
    // p(x) = 1 + x  =>  p(x)^2 = (1+x)^2
    // Verify by evaluation
    Polynom p({1.0f, 2.0f});  // 1 + 2x
    Polynom sq = p.square();
    EXPECT_EQ(sq.degree(), 2);

    // Compare squared polynomial evaluation with direct squaring
    for (float x = -2.0f; x <= 2.0f; x += 0.5f) {
        float expected = p(x) * p(x);
        EXPECT_NEAR(sq(x), expected, 1e-5f);
    }
}

TEST(PolynomSquare, ExceedsMaxDegreeThrows) {
    // Degree 3 polynomial squared = degree 6, which exceeds MAX_DEGREE=5
    Polynom p({1.0f, 1.0f, 1.0f, 1.0f});  // degree 3
    EXPECT_THROW(p.square(), std::runtime_error);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(PolynomIntegrate, ConstantIntegral) {
    // p(x) = 3  =>  ∫p(x)dx = 3x (+ C, where C=0)
    Polynom p({3.0f});
    Polynom integral = p.integrate();
    EXPECT_EQ(integral.degree(), 1);
    EXPECT_FLOAT_EQ(integral(0.0f), 0.0f);  // C = 0
    EXPECT_FLOAT_EQ(integral(2.0f), 6.0f);  // 3*2
}

TEST(PolynomIntegrate, LinearIntegral) {
    // p(x) = 2 + 4x  =>  ∫p(x)dx = 2x + 2x^2
    Polynom p({2.0f, 4.0f});
    Polynom integral = p.integrate();
    EXPECT_EQ(integral.degree(), 2);
    EXPECT_FLOAT_EQ(integral(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(integral(1.0f), 4.0f);  // 2*1 + 2*1 = 4
    EXPECT_FLOAT_EQ(integral(2.0f), 12.0f); // 2*2 + 2*4 = 12
}

TEST(PolynomIntegrate, QuadraticIntegral) {
    // p(x) = 3x^2  =>  ∫p(x)dx = x^3
    Polynom p({0.0f, 0.0f, 3.0f});
    Polynom integral = p.integrate();
    EXPECT_EQ(integral.degree(), 3);
    EXPECT_FLOAT_EQ(integral(2.0f), 8.0f);  // 2^3 = 8
}

TEST(PolynomIntegrate, DerivativeOfIntegralIsOriginal) {
    Polynom p({1.0f, 2.0f, 3.0f});
    Polynom integral = p.integrate();
    Polynom back = integral.derivative();

    // Check that p == back for several x values
    for (float x = -2.0f; x <= 2.0f; x += 0.5f) {
        EXPECT_NEAR(p(x), back(x), 1e-5f);
    }
}

TEST(PolynomIntegrate, MaxDegreeThrows) {
    // Degree 5 polynomial integrated would become degree 6
    Polynom p({1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f});  // degree 5
    EXPECT_THROW(p.integrate(), std::runtime_error);
}

// ============================================================================
// Definite Integration Tests
// ============================================================================

TEST(PolynomIntegrateDefinite, ConstantIntegral) {
    // ∫₀² 3 dx = 6
    Polynom p({3.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(0.0f, 2.0f), 6.0f);
}

TEST(PolynomIntegrateDefinite, LinearIntegral) {
    // ∫₀¹ (2 + 4x) dx = [2x + 2x²]₀¹ = 2 + 2 = 4
    Polynom p({2.0f, 4.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(0.0f, 1.0f), 4.0f);
}

TEST(PolynomIntegrateDefinite, QuadraticIntegral) {
    // ∫₀² x² dx = [x³/3]₀² = 8/3
    Polynom p({0.0f, 0.0f, 1.0f});
    EXPECT_NEAR(p.integrateDefinite(0.0f, 2.0f), 8.0f / 3.0f, 1e-5f);
}

TEST(PolynomIntegrateDefinite, NonZeroLowerBound) {
    // ∫₁² x dx = [x²/2]₁² = 2 - 0.5 = 1.5
    Polynom p({0.0f, 1.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(1.0f, 2.0f), 1.5f);
}

TEST(PolynomIntegrateDefinite, NegativeBounds) {
    // ∫₋₁¹ x² dx = [x³/3]₋₁¹ = 1/3 - (-1/3) = 2/3
    Polynom p({0.0f, 0.0f, 1.0f});
    EXPECT_NEAR(p.integrateDefinite(-1.0f, 1.0f), 2.0f / 3.0f, 1e-5f);
}

TEST(PolynomIntegrateDefinite, ReversedBoundsGivesNegative) {
    // ∫₂⁰ 3 dx = -∫₀² 3 dx = -6
    Polynom p({3.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(2.0f, 0.0f), -6.0f);
}

TEST(PolynomIntegrateDefinite, SameBoundsGivesZero) {
    Polynom p({1.0f, 2.0f, 3.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(5.0f, 5.0f), 0.0f);
}

// ============================================================================
// Subtraction Operator Tests
// ============================================================================

TEST(PolynomSubtract, SubtractScalarFromConstant) {
    Polynom p({5.0f});
    Polynom result = p - 3.0f;
    EXPECT_EQ(result.degree(), 0);
    EXPECT_FLOAT_EQ(result(0.0f), 2.0f);
}

TEST(PolynomSubtract, SubtractScalarFromPolynomial) {
    // p(x) = 5 + 2x  =>  p(x) - 3 = 2 + 2x
    Polynom p({5.0f, 2.0f});
    Polynom result = p - 3.0f;
    EXPECT_EQ(result.degree(), 1);
    EXPECT_FLOAT_EQ(result(0.0f), 2.0f);
    EXPECT_FLOAT_EQ(result(1.0f), 4.0f);
}

TEST(PolynomSubtract, SubtractNegativeScalar) {
    Polynom p({2.0f});
    Polynom result = p - (-3.0f);
    EXPECT_FLOAT_EQ(result(0.0f), 5.0f);
}

TEST(PolynomSubtract, DoesNotModifyOriginal) {
    Polynom p({5.0f, 2.0f});
    (void)(p - 3.0f);  // Perform subtraction but discard result
    EXPECT_FLOAT_EQ(p(0.0f), 5.0f);  // Original unchanged
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(PolynomEdgeCases, ZeroPolynomial) {
    Polynom p({0.0f});
    EXPECT_FLOAT_EQ(p(100.0f), 0.0f);
    EXPECT_FLOAT_EQ(p.derivative()(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(p.integrate()(1.0f), 0.0f);
    EXPECT_FLOAT_EQ(p.integrateDefinite(0.0f, 10.0f), 0.0f);
}

TEST(PolynomEdgeCases, LargeInputValues) {
    Polynom p({1.0f, 1.0f});  // p(x) = 1 + x
    EXPECT_FLOAT_EQ(p(1000.0f), 1001.0f);
    EXPECT_FLOAT_EQ(p(-1000.0f), -999.0f);
}

TEST(PolynomEdgeCases, SmallInputValues) {
    Polynom p({1.0f, 1.0f, 1.0f});  // p(x) = 1 + x + x^2
    float tiny = 1e-6f;
    EXPECT_NEAR(p(tiny), 1.0f + tiny + tiny * tiny, 1e-10f);
}
