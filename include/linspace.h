#ifndef LINSPACE_H_INCLUDED
#define LINSPACE_H_INCLUDED

#include <array>

template <std::size_t N>
constexpr std::array<float, N> linspace(float start, float stop) {
    std::array<float, N> result{};
    if constexpr (N == 1) {
        result[0] = start;
    } else {
        float step = (stop - start) / (N - 1);
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = start + i * step;
        }
    }
    return result;
}

#endif // LINSPACE_H_INCLUDED