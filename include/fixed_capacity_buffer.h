#ifndef FIXED_CAPACITY_BUFFER_H_INCLUDED
#define FIXED_CAPACITY_BUFFER_H_INCLUDED

#include <array>
#include <cstddef>
#include <utility>

namespace Common {

// Fixed-capacity buffer with runtime size tracking
// Provides STL-compatible interface for stack-based, zero-allocation storage
// Elements are densely packed at indices [0, size_)
template <typename T, std::size_t Capacity> class FixedCapacityBuffer {
  private:
    std::array<T, Capacity> data_;
    std::size_t size_ = 0;

  public:
    // Type aliases for STL compatibility
    using value_type = T;
    using reference = T &;
    using const_reference = const T &;
    using iterator = typename std::array<T, Capacity>::iterator;
    using const_iterator = typename std::array<T, Capacity>::const_iterator;
    using size_type = std::size_t;

    // Constructors
    constexpr FixedCapacityBuffer() = default;

    template <std::size_t N>
    constexpr FixedCapacityBuffer(const std::array<T, N> &data) {
        static_assert(N <= Capacity, "Array size exceeds buffer capacity");
        size_ = N;
        for (std::size_t i = 0; i < N; ++i) {
            data_[i] = data[i];
        }
    }

    // Capacity queries
    constexpr size_type size() const noexcept {
        return size_;
    }

    constexpr size_type capacity() const noexcept {
        return Capacity;
    }

    constexpr bool empty() const noexcept {
        return size_ == 0;
    }

    constexpr bool full() const noexcept {
        return size_ == Capacity;
    }

    // Element access (no bounds checking for performance)
    constexpr const_reference operator[](size_type idx) const noexcept {
        return data_[idx];
    }

    constexpr reference operator[](size_type idx) noexcept {
        return data_[idx];
    }

    // Add element (only if not full)
    // Returns true if added, false if capacity exceeded
    constexpr bool push_back(const T &value) noexcept {
        if (size_ >= Capacity) {
            return false;
        }
        data_[size_++] = value;
        return true;
    }

    constexpr bool push_back(T &&value) noexcept {
        if (size_ >= Capacity) {
            return false;
        }
        data_[size_++] = std::move(value);
        return true;
    }

    // Clear buffer (reset size to 0)
    constexpr void clear() noexcept {
        size_ = 0;
    }

    // Iterators (only over valid elements [0, size_))
    constexpr iterator begin() noexcept {
        return data_.begin();
    }

    constexpr const_iterator begin() const noexcept {
        return data_.begin();
    }

    constexpr const_iterator cbegin() const noexcept {
        return data_.cbegin();
    }

    constexpr iterator end() noexcept {
        return data_.begin() + size_;
    }

    constexpr const_iterator end() const noexcept {
        return data_.begin() + size_;
    }

    constexpr const_iterator cend() const noexcept {
        return data_.cbegin() + size_;
    }

    // Reserve (no-op for compatibility, capacity is fixed at compile-time)
    constexpr void reserve(size_type) noexcept {
    }
};

} // namespace Common

#endif // FIXED_CAPACITY_BUFFER_H_INCLUDED
