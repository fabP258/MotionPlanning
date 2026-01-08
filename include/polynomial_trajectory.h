#ifndef POLYNOMIAL_TRAJECTORY_H_INCLUDED
#define POLYNOMIAL_TRAJECTORY_H_INCLUDED

#include "geometry.h"
#include "polynom.h"
#include <optional>

namespace Common {

class PolynomialTrajectory {
  private:
    Polynom polynom_;
    FrenetState startState_;
    FrenetState endState_;
    float endTime_;
    bool hasFullEndState_;

    PolynomialTrajectory(Polynom poly, FrenetState start, FrenetState end,
                         float time, bool fullEndState);

    bool isInValidRange(float t) const;

  public:
    // Factory methods for creating trajectories
    static std::optional<PolynomialTrajectory>
    fromBoundaryStates(const FrenetState &startState,
                       const FrenetState &endState, float endTime);

    static std::optional<PolynomialTrajectory>
    fromStartStateAndEndVelocity(const FrenetState &startState,
                                 float endVelocity, float endAcceleration,
                                 float endTime);

    // Accessors
    const Polynom &polynom() const {
        return polynom_;
    }

    const FrenetState &startState() const {
        return startState_;
    }

    const FrenetState &endState() const {
        return endState_;
    }

    float endTime() const {
        return endTime_;
    }

    bool hasFullEndState() const {
        return hasFullEndState_;
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
        return polynom_.derivative(1).evaluate(t);
    }

    float acceleration(float t) const {
        return polynom_.derivative(2).evaluate(t);
    }

    float jerk(float t) const {
        return polynom_.derivative(3).evaluate(t);
    }

    bool isMaxAccelerationBelowLimit(const float maxAcceleration) const;

    // Cost function: integral of squared jerk over [0, endTime]
    // ∫₀ᵀ j(t)² dt - measures smoothness/comfort of trajectory
    float jerkCost() const {
        Polynom jerk = polynom_.derivative(3);
        Polynom jerkSquared = jerk.square();
        return jerkSquared.integrateDefinite(0.0f, endTime_);
    }
};

} // namespace Common

#endif // POLYNOMIAL_TRAJECTORY_H_INCLUDED
