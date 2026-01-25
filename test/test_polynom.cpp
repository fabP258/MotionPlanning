#include "polynom.h"
#include <cmath>
#include <gtest/gtest.h>

using Common::Polynom;

// ============================================================================
// Construction Tests
// ============================================================================

TEST(PolynomConstructor, DefaultConstructorCreatesZeroPolynomial) {
    Polynom<2> p;
    EXPECT_FLOAT_EQ(p(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(p(1.0f), 0.0f);
    EXPECT_FLOAT_EQ(p(5.0f), 0.0f);
}

TEST(PolynomConstructor, StdArrayConstruction) {
    Polynom<2> p({1.0f, 0.0f, 1.0f}); // p(x) = 1 + x^2
    EXPECT_FLOAT_EQ(p(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(p(2.0f), 5.0f);  // 1 + 4
    EXPECT_FLOAT_EQ(p(3.0f), 10.0f); // 1 + 9
}

TEST(PolynomConstructor, DegreeMatchesTemplateArgument) {
    Polynom<2> p({1.0f, 0.0f, 1.0f});
    EXPECT_EQ(p.degree(), 2);
}

// ============================================================================
// Evaluation Tests
// ============================================================================

TEST(PolynomEvaluate, EvaluateAndOperatorAreEquivalent) {
    Polynom<2> p({1.0f, 2.0f, 3.0f});
    EXPECT_FLOAT_EQ(p.evaluate(2.5f), p(2.5f));
}

TEST(PolynomEvaluate, NegativeInput) {
    // p(x) = 1 + 2x + x^2
    Polynom<2> p({1.0f, 2.0f, 1.0f});
    EXPECT_FLOAT_EQ(p(-1.0f), 0.0f); // 1 - 2 + 1 = 0
    EXPECT_FLOAT_EQ(p(-2.0f), 1.0f); // 1 - 4 + 4 = 1
}

TEST(PolynomEvaluate, ZeroCoefficients) {
    Polynom<2> p({0.0f, 0.0f, 1.0f}); // p(x) = x^2
    EXPECT_FLOAT_EQ(p(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(p(3.0f), 9.0f);
}

// ============================================================================
// Derivative Tests
// ============================================================================

TEST(PolynomDerivative, ConstantDerivativeIsZero) {
    Polynom<0> p({5.0f});
    Polynom<0> dp = p.derivative();
    EXPECT_EQ(dp.degree(), 0);
    EXPECT_FLOAT_EQ(dp(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(dp(100.0f), 0.0f);
}

TEST(PolynomDerivative, LinearDerivativeIsConstant) {
    // p(x) = 3 + 4x  =>  p'(x) = 4
    Polynom<1> p({3.0f, 4.0f});
    Polynom<0> dp = p.derivative();
    EXPECT_EQ(dp.degree(), 0);
    EXPECT_FLOAT_EQ(dp(0.0f), 4.0f);
    EXPECT_FLOAT_EQ(dp(10.0f), 4.0f);
}

TEST(PolynomDerivative, QuadraticDerivativeIsLinear) {
    // p(x) = 1 + 2x + 3x^2  =>  p'(x) = 2 + 6x
    Polynom<2> p({1.0f, 2.0f, 3.0f});
    Polynom<1> dp = p.derivative();
    EXPECT_EQ(dp.degree(), 1);
    EXPECT_FLOAT_EQ(dp(0.0f), 2.0f);
    EXPECT_FLOAT_EQ(dp(1.0f), 8.0f);
    EXPECT_FLOAT_EQ(dp(2.0f), 14.0f);
}

TEST(PolynomDerivative, CubicDerivative) {
    // p(x) = 1 + x + x^2 + x^3  =>  p'(x) = 1 + 2x + 3x^2
    Polynom<3> p({1.0f, 1.0f, 1.0f, 1.0f});
    Polynom<2> dp = p.derivative();
    EXPECT_EQ(dp.degree(), 2);
    EXPECT_FLOAT_EQ(dp(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(dp(1.0f), 6.0f); // 1 + 2 + 3
}

TEST(PolynomDerivative, NthOrderDerivativeZeroReturnsOriginal) {
    Polynom<2> p({1.0f, 2.0f, 3.0f});
    Polynom<2> d0 = p.derivative<0>();
    EXPECT_EQ(d0.degree(), p.degree());
    EXPECT_FLOAT_EQ(d0(2.0f), p(2.0f));
}

TEST(PolynomDerivative, NthOrderDerivative) {
    // p(x) = 1 + 2x + 3x^2 + 4x^3
    // p'(x) = 2 + 6x + 12x^2
    // p''(x) = 6 + 24x
    // p'''(x) = 24
    Polynom<3> p({1.0f, 2.0f, 3.0f, 4.0f});

    Polynom<2> d1 = p.derivative<1>();
    EXPECT_EQ(d1.degree(), 2);
    EXPECT_FLOAT_EQ(d1(0.0f), 2.0f);

    Polynom<1> d2 = p.derivative<2>();
    EXPECT_EQ(d2.degree(), 1);
    EXPECT_FLOAT_EQ(d2(0.0f), 6.0f);
    EXPECT_FLOAT_EQ(d2(1.0f), 30.0f);

    Polynom<0> d3 = p.derivative<3>();
    EXPECT_EQ(d3.degree(), 0);
    EXPECT_FLOAT_EQ(d3(0.0f), 24.0f);
}

TEST(PolynomDerivative, HighOrderDerivativeBecomesZero) {
    Polynom<2> p({1.0f, 2.0f, 3.0f});  // degree 2
    Polynom<0> d5 = p.derivative<5>(); // derivative order > degree
    EXPECT_EQ(d5.degree(), 0);
    EXPECT_FLOAT_EQ(d5(0.0f), 0.0f);
}

// ============================================================================
// Square Tests
// ============================================================================

TEST(PolynomSquare, ConstantSquare) {
    Polynom<0> p({3.0f});
    Polynom<0> sq = p.square();
    EXPECT_EQ(sq.degree(), 0);
    EXPECT_FLOAT_EQ(sq(0.0f), 9.0f);
}

TEST(PolynomSquare, LinearSquare) {
    // p(x) = 1 + x  =>  p(x)^2 = 1 + 2x + x^2
    Polynom<1> p({1.0f, 1.0f});
    Polynom<2> sq = p.square();
    EXPECT_EQ(sq.degree(), 2);
    EXPECT_FLOAT_EQ(sq(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(sq(1.0f), 4.0f); // (1+1)^2 = 4
    EXPECT_FLOAT_EQ(sq(2.0f), 9.0f); // (1+2)^2 = 9
}

TEST(PolynomSquare, QuadraticSquare) {
    // p(x) = 1 + x  =>  p(x)^2 = (1+x)^2
    // Verify by evaluation
    Polynom<1> p({1.0f, 2.0f}); // 1 + 2x
    Polynom<2> sq = p.square();
    EXPECT_EQ(sq.degree(), 2);

    // Compare squared polynomial evaluation with direct squaring
    for (float x = -2.0f; x <= 2.0f; x += 0.5f) {
        float expected = p(x) * p(x);
        EXPECT_NEAR(sq(x), expected, 1e-5f);
    }
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(PolynomIntegrate, ConstantIntegral) {
    // p(x) = 3  =>  ∫p(x)dx = 3x (+ C, where C=0)
    Polynom<0> p({3.0f});
    Polynom<1> integral = p.integrate();
    EXPECT_EQ(integral.degree(), 1);
    EXPECT_FLOAT_EQ(integral(0.0f), 0.0f); // C = 0
    EXPECT_FLOAT_EQ(integral(2.0f), 6.0f); // 3*2
}

TEST(PolynomIntegrate, LinearIntegral) {
    // p(x) = 2 + 4x  =>  ∫p(x)dx = 2x + 2x^2
    Polynom<1> p({2.0f, 4.0f});
    Polynom<2> integral = p.integrate();
    EXPECT_EQ(integral.degree(), 2);
    EXPECT_FLOAT_EQ(integral(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(integral(1.0f), 4.0f);  // 2*1 + 2*1 = 4
    EXPECT_FLOAT_EQ(integral(2.0f), 12.0f); // 2*2 + 2*4 = 12
}

TEST(PolynomIntegrate, QuadraticIntegral) {
    // p(x) = 3x^2  =>  ∫p(x)dx = x^3
    Polynom<2> p({0.0f, 0.0f, 3.0f});
    Polynom<3> integral = p.integrate();
    EXPECT_EQ(integral.degree(), 3);
    EXPECT_FLOAT_EQ(integral(2.0f), 8.0f); // 2^3 = 8
}

TEST(PolynomIntegrate, DerivativeOfIntegralIsOriginal) {
    Polynom<2> p({1.0f, 2.0f, 3.0f});
    Polynom<3> integral = p.integrate();
    Polynom<2> back = integral.derivative();

    // Check that p == back for several x values
    for (float x = -2.0f; x <= 2.0f; x += 0.5f) {
        EXPECT_NEAR(p(x), back(x), 1e-5f);
    }
}

// ============================================================================
// Definite Integration Tests
// ============================================================================

TEST(PolynomIntegrateDefinite, ConstantIntegral) {
    // ∫₀² 3 dx = 6
    Polynom<0> p({3.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(0.0f, 2.0f), 6.0f);
}

TEST(PolynomIntegrateDefinite, LinearIntegral) {
    // ∫₀¹ (2 + 4x) dx = [2x + 2x²]₀¹ = 2 + 2 = 4
    Polynom<1> p({2.0f, 4.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(0.0f, 1.0f), 4.0f);
}

TEST(PolynomIntegrateDefinite, QuadraticIntegral) {
    // ∫₀² x² dx = [x³/3]₀² = 8/3
    Polynom<2> p({0.0f, 0.0f, 1.0f});
    EXPECT_NEAR(p.integrateDefinite(0.0f, 2.0f), 8.0f / 3.0f, 1e-5f);
}

TEST(PolynomIntegrateDefinite, NonZeroLowerBound) {
    // ∫₁² x dx = [x²/2]₁² = 2 - 0.5 = 1.5
    Polynom<1> p({0.0f, 1.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(1.0f, 2.0f), 1.5f);
}

TEST(PolynomIntegrateDefinite, NegativeBounds) {
    // ∫₋₁¹ x² dx = [x³/3]₋₁¹ = 1/3 - (-1/3) = 2/3
    Polynom<2> p({0.0f, 0.0f, 1.0f});
    EXPECT_NEAR(p.integrateDefinite(-1.0f, 1.0f), 2.0f / 3.0f, 1e-5f);
}

TEST(PolynomIntegrateDefinite, ReversedBoundsGivesNegative) {
    // ∫₂⁰ 3 dx = -∫₀² 3 dx = -6
    Polynom<0> p({3.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(2.0f, 0.0f), -6.0f);
}

TEST(PolynomIntegrateDefinite, SameBoundsGivesZero) {
    Polynom<2> p({1.0f, 2.0f, 3.0f});
    EXPECT_FLOAT_EQ(p.integrateDefinite(5.0f, 5.0f), 0.0f);
}

// ============================================================================
// Subtraction Operator Tests
// ============================================================================

TEST(PolynomSubtract, SubtractScalarFromConstant) {
    Polynom<0> p({5.0f});
    Polynom<0> result = p - 3.0f;
    EXPECT_EQ(result.degree(), 0);
    EXPECT_FLOAT_EQ(result(0.0f), 2.0f);
}

TEST(PolynomSubtract, SubtractScalarFromPolynomial) {
    // p(x) = 5 + 2x  =>  p(x) - 3 = 2 + 2x
    Polynom<1> p({5.0f, 2.0f});
    Polynom<1> result = p - 3.0f;
    EXPECT_EQ(result.degree(), 1);
    EXPECT_FLOAT_EQ(result(0.0f), 2.0f);
    EXPECT_FLOAT_EQ(result(1.0f), 4.0f);
}

TEST(PolynomSubtract, SubtractNegativeScalar) {
    Polynom<0> p({2.0f});
    Polynom<0> result = p - (-3.0f);
    EXPECT_FLOAT_EQ(result(0.0f), 5.0f);
}

TEST(PolynomSubtract, DoesNotModifyOriginal) {
    Polynom<1> p({5.0f, 2.0f});
    (void)(p - 3.0f);               // Perform subtraction but discard result
    EXPECT_FLOAT_EQ(p(0.0f), 5.0f); // Original unchanged
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(PolynomEdgeCases, ZeroPolynomial) {
    Polynom<0> p({0.0f});
    EXPECT_FLOAT_EQ(p(100.0f), 0.0f);
    EXPECT_FLOAT_EQ(p.derivative()(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(p.integrate()(1.0f), 0.0f);
    EXPECT_FLOAT_EQ(p.integrateDefinite(0.0f, 10.0f), 0.0f);
}

TEST(PolynomEdgeCases, LargeInputValues) {
    Polynom<1> p({1.0f, 1.0f}); // p(x) = 1 + x
    EXPECT_FLOAT_EQ(p(1000.0f), 1001.0f);
    EXPECT_FLOAT_EQ(p(-1000.0f), -999.0f);
}

TEST(PolynomEdgeCases, SmallInputValues) {
    Polynom<2> p({1.0f, 1.0f, 1.0f}); // p(x) = 1 + x + x^2
    float tiny = 1e-6f;
    EXPECT_NEAR(p(tiny), 1.0f + tiny + tiny * tiny, 1e-10f);
}
