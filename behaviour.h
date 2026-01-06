#ifndef BEHAVIOUR_H_INCLUDED
#define BEHAVIOUR_H_INCLUDED

#include "geometry.h"
#include <optional>
#include <span>
#include <vector>

namespace Planner {

class LongitudinalBehaviour {
  public:
    std::vector<std::optional<Common::TPolynom<float, 5>>>
    sampleTrajectories(const Common::FrenetState &startState,
                       const float endTime);

    std::optional<Common::TPolynom<float, 5>>
    calcTrajectory(const Common::FrenetState &startState, const float endTime,
                   float offset) const;

    virtual ~LongitudinalBehaviour() = default;

  private:
    virtual std::span<const float> offsetGrid() const = 0;

    virtual Common::FrenetState calcTargetState(const float endTime,
                                                const float offset) const = 0;

    virtual std::optional<Common::TPolynom<float, 5>>
    dispatchCalculation(const Common::FrenetState &startState,
                        const Common::FrenetState &endState,
                        const float endTime) const = 0;
};

class FollowingBehaviour : public LongitudinalBehaviour {
  public:
    FollowingBehaviour(const Common::FrenetState &lvState, const float D,
                       const float tg)
        : leadVehicleState(lvState), minGap(D), timeGap(tg) {
    }

  private:
    std::span<const float> offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime,
                                        const float offset) const override;

    std::optional<Common::TPolynom<float, 5>>
    dispatchCalculation(const Common::FrenetState &startState,
                        const Common::FrenetState &endState,
                        const float endTime) const override;

    float calcLeadVehiclePosition(const float time) const;

    float calcLeadVehicleVelocity(const float time) const;

    float calcLeadVehicleAcceleration(const float time) const;

    static constexpr std::array<float, 5> POSITION_OFFSET_GRID = {
        -2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    Common::FrenetState leadVehicleState;
    float minGap;
    float timeGap;
};

class StoppingBehaviour : public LongitudinalBehaviour {
  public:
    StoppingBehaviour(const float sd) : stopDistance(sd) {
    }

  private:
    std::span<const float> offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime,
                                        const float offset) const override;

    std::optional<Common::TPolynom<float, 5>>
    dispatchCalculation(const Common::FrenetState &startState,
                        const Common::FrenetState &endState,
                        const float endTime) const override;

  private:
    static constexpr std::array<float, 5> POSITION_OFFSET_GRID = {0.0f};
    float stopDistance;
};

class MergingBehaviour : public LongitudinalBehaviour {
  public:
    MergingBehaviour(const Common::FrenetState &leadState,
                     const Common::FrenetState &lagState, const float D,
                     const float tg)
        : leadVehicleState(leadState), lagVehicleState(lagState), minGap(D),
          timeGap(tg) {
    }

  private:
    std::span<const float> offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime,
                                        const float offset) const override;

    std::optional<Common::TPolynom<float, 5>>
    dispatchCalculation(const Common::FrenetState &startState,
                        const Common::FrenetState &endState,
                        const float endTime) const override;

    float predictVehiclePosition(const Common::FrenetState &state,
                                 const float time) const;

    float predictVehicleVelocity(const Common::FrenetState &state,
                                 const float time) const;

    float predictVehicleAcceleration(const Common::FrenetState &state,
                                     const float time) const;

    static constexpr std::array<float, 5> POSITION_OFFSET_GRID = {
        -2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    Common::FrenetState leadVehicleState;
    Common::FrenetState lagVehicleState;
    float minGap;
    float timeGap;
};

class VelocityKeepingBehaviour : public LongitudinalBehaviour {
  public:
    VelocityKeepingBehaviour(float vd) : desiredVelocity(vd) {
    }

  private:
    std::span<const float> offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime,
                                        const float offset) const override;

    std::optional<Common::TPolynom<float, 5>>
    dispatchCalculation(const Common::FrenetState &startState,
                        const Common::FrenetState &endState,
                        const float endTime) const override;

    static constexpr std::array<float, 5> VELOCITY_OFFSET_GRID = {
        -5.0f, -2.5f, 0.0f, 2.5f, 5.0f};
    float desiredVelocity;
};

} // namespace Planner

#endif // BEHAVIOUR_H_INCLUDED