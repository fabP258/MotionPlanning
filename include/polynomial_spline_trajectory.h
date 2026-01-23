#ifndef POLYNOMIAL_SPLINE_H_INCLUDED
#define POLYNOMIAL_SPLINE_H_INCLUDED

#include "polynomial_trajectory.h"
#include <array>
#include <optional>

namespace Common {

template <int N> class PolynomialSplineTrajectory {
  private:
    struct SegmentQuery {
        const PolynomialTrajectory *segment;
        float localTime;
    };

    std::array<PolynomialTrajectory, N> segments_;

    std::optional<SegmentQuery> findSegment(float time) const {
        if (time < 0.0f) {
            return std::nullopt;
        }

        float prevEndTime = 0.0f;
        for (const PolynomialTrajectory &trajectory : segments_) {
            float endTime = prevEndTime + trajectory.endTime();
            if (time <= endTime) {
                return SegmentQuery{&trajectory, time - prevEndTime};
            }
            prevEndTime = endTime;
        }

        return std::nullopt;
    }

  public:
    PolynomialSplineTrajectory(
        const std::array<PolynomialTrajectory, N> &segments)
        : segments_(segments) {
    }

    std::optional<float> evaluate(float time) const {
        auto query = findSegment(time);
        if (!query) {
            return std::nullopt;
        }
        return query->segment->evaluate(query->localTime);
    }

    std::optional<FrenetState> evaluateState(float time) const {
        auto query = findSegment(time);
        if (!query) {
            return std::nullopt;
        }
        return query->segment->evaluateState(query->localTime);
    }
};

} // namespace Common

#endif // POLYNOMIAL_SPLINE_H_INCLUDED