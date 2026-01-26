#ifndef POLYNOMIAL_TRAJECTORY_H_INCLUDED
#define POLYNOMIAL_TRAJECTORY_H_INCLUDED

#include "geometry.h"
#include "polynom.h"
#include <optional>

namespace Common {

class PolynomialTrajectory {
  public:
    using PolynomDeg5 = Polynom<5>;

    PolynomialTrajectory()
        : polynom_(), endTime_(0.0f), hasFullEndState_(false), cost_(0.0f) {
    }

    PolynomialTrajectory(PolynomDeg5 poly, float time, bool fullEndState)
        : polynom_(poly), endTime_(time), hasFullEndState_(fullEndState),
          cost_(std::numeric_limits<float>::max()) {
    }

    // Factory methods for creating trajectories
    static std::optional<PolynomialTrajectory>
    fromBoundaryStates(const FrenetState &startState,
                       const FrenetState &endState, float endTime);

    static std::optional<PolynomialTrajectory>
    fromStartStateAndEndVelocity(const FrenetState &startState,
                                 float endVelocity, float endAcceleration,
                                 float endTime);

    // Accessors
    const PolynomDeg5 &polynom() const {
        return polynom_;
    }

    FrenetState endState() const {
        return evaluateState(endTime_).value();
    }

    float endTime() const {
        return endTime_;
    }

    bool hasFullEndState() const {
        return hasFullEndState_;
    }

    float cost() const {
        return cost_;
    }

    void setCost(float c) {
        cost_ = c;
    }

    // Convenience evaluation (delegates to underlying polynom)
    float evaluate(float t) const {
        return polynom_.evaluate(t);
    }

    float operator()(float t) const {
        return evaluate(t);
    }

    std::optional<FrenetState> evaluateState(float t) const;

    // Trajectory-specific queries
    float velocity(float t) const {
        return polynom_.derivative<1>().evaluate(t);
    }

    float acceleration(float t) const {
        return polynom_.derivative<2>().evaluate(t);
    }

    float jerk(float t) const {
        return polynom_.derivative<3>().evaluate(t);
    }

    bool isMaxAccelerationBelowLimit(const float maxAcceleration) const;

    bool isMaxJerkBelowLimit(const float maxJerk) const;

    // Cost function: integral of squared jerk over [0, endTime]
    // ∫₀ᵀ j(t)² dt - measures smoothness/comfort of trajectory
    float jerkCost() const {
        auto jerk = polynom_.derivative<3>();
        auto jerkSquared = jerk.square();
        return jerkSquared.integrateDefinite(0.0f, endTime_);
    }

    // Cost function: integral of squared distance error over [0, endTime]
    // ∫₀ᵀ (d(t) - reference)² dt - penalizes deviation from reference position
    float distanceCost(float reference) const;

    // Cost function: integral of squared velocity error over [0, endTime]
    // ∫₀ᵀ (v(t) - reference)² dt - penalizes deviation from reference velocity
    float velocityCost(float reference) const;

  private:
    PolynomDeg5 polynom_;
    float endTime_;
    bool hasFullEndState_;
    float cost_;

    bool isInValidRange(float t) const;
};

} // namespace Common

#endif // POLYNOMIAL_TRAJECTORY_H_INCLUDED
