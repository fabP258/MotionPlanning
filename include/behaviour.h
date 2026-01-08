#ifndef BEHAVIOUR_H_INCLUDED
#define BEHAVIOUR_H_INCLUDED

#include "fixed_capacity_buffer.h"
#include "geometry.h"
#include "polynomial_trajectory.h"
#include <optional>
#include <vector>

namespace Planner {

// Maximum number of offset samples for any longitudinal behavior
// All longitudinal behaviors must ensure their offset grids do not exceed this
// size
static constexpr std::size_t MAX_LONGITUDINAL_OFFSET_SAMPLES = 5;

struct CostWeights {
    float squaredJerkIntegral = 1.0f;
    float maneuverTime = 1.0f;
    float squaredTargetdeviation = 1.0f;
};

class LongitudinalBehaviour {
  public:
    enum class PlanningStrategy { FULL, VELOCITY };

    virtual Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
    offsetGrid() const = 0;

    virtual Common::FrenetState calcTargetState(const float endTime) const = 0;

    virtual PlanningStrategy planningStrategy() const = 0;

    virtual CostWeights costWeights() const = 0;

    virtual ~LongitudinalBehaviour() = default;
};

class FollowingBehaviour : public LongitudinalBehaviour {
  public:
    FollowingBehaviour(const Common::FrenetState &lvState, const float D,
                       const float tg)
        : leadVehicleState_(lvState), minGap_(D), timeGap_(tg) {
    }

    Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
    offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime) const override;

    PlanningStrategy planningStrategy() const override;

    CostWeights costWeights() const override;

  private:
    float calcLeadVehiclePosition(const float time) const;

    float calcLeadVehicleVelocity(const float time) const;

    float calcLeadVehicleAcceleration(const float time) const;

    // TODO: use linspace()
    static constexpr std::array<float, 5> POSITION_OFFSET_GRID = {
        -2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    static constexpr CostWeights costWeights_{};
    Common::FrenetState leadVehicleState_;
    float minGap_;
    float timeGap_;
};

class StoppingBehaviour : public LongitudinalBehaviour {
  public:
    StoppingBehaviour(const float sd) : stopDistance_(sd) {
    }

    Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
    offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime) const override;

    PlanningStrategy planningStrategy() const override;

    CostWeights costWeights() const override;

  private:
    static constexpr std::array<float, 1> POSITION_OFFSET_GRID = {0.0f};
    static constexpr CostWeights costWeights_{};
    float stopDistance_;
};

class MergingBehaviour : public LongitudinalBehaviour {
  public:
    MergingBehaviour(const Common::FrenetState &leadState,
                     const Common::FrenetState &lagState, const float D,
                     const float tg)
        : leadVehicleState(leadState), lagVehicleState(lagState), minGap(D),
          timeGap(tg) {
    }

    Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
    offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime) const override;

    PlanningStrategy planningStrategy() const override;

    CostWeights costWeights() const override;

  private:
    float predictVehiclePosition(const Common::FrenetState &state,
                                 const float time) const;

    float predictVehicleVelocity(const Common::FrenetState &state,
                                 const float time) const;

    float predictVehicleAcceleration(const Common::FrenetState &state,
                                     const float time) const;

    // TODO: use linspace()
    static constexpr std::array<float, 5> POSITION_OFFSET_GRID = {
        -2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    static constexpr CostWeights costWeights_{};
    Common::FrenetState leadVehicleState;
    Common::FrenetState lagVehicleState;
    float minGap;
    float timeGap;
};

class VelocityKeepingBehaviour : public LongitudinalBehaviour {
  public:
    VelocityKeepingBehaviour(float vd) : desiredVelocity(vd) {
    }

    Common::FixedCapacityBuffer<float, MAX_LONGITUDINAL_OFFSET_SAMPLES>
    offsetGrid() const override;

    Common::FrenetState calcTargetState(const float endTime) const override;

    PlanningStrategy planningStrategy() const override;

    CostWeights costWeights() const override;

  private:
    // TODO: use linspace()
    static constexpr std::array<float, 5> VELOCITY_OFFSET_GRID = {
        -5.0f, -2.5f, 0.0f, 2.5f, 5.0f};
    static constexpr CostWeights costWeights_{};
    float desiredVelocity;
};

} // namespace Planner

#endif // BEHAVIOUR_H_INCLUDED
