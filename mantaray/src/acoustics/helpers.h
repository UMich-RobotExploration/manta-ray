//
// Created by tko on 1/28/26.
//
#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <stdexcept>

namespace acoustics {

enum class RectangleCorners { kTopRight, kTopLeft, kBottomRight, kBottomLeft };

/**
 * @brief Generate linearly spaced values between start and end
 * @tparam T Numeric type (typically float or double)
 * @param start Starting value
 * @param end Ending value
 * @param num Number of points to generate
 * @return Vector of linearly spaced values
 */
template <typename T> std::vector<T> linspace(T start, T end, std::size_t num) {
  std::vector<T> result;
  result.reserve(num);

  if (num == 0)
    return result;
  if (num == 1) {
    result.emplace_back(start);
    return result;
  }

  T step = (end - start) / (num - 1);
  for (std::size_t i = 0; i < num; ++i)
    result.emplace_back(start + step * i);

  return result;
}

/**
 * @brief Setup array with linearly spaced values from low to high
 * @tparam T Numeric type (typically float or double)
 * @param arr Pointer to array to populate (must have size elements allocated)
 * @param low Lower bound value
 * @param high Upper bound value
 * @param size Number of elements in array
 *
 * @note No bounds checking is performed. Caller must ensure arr has sufficient
 * space.
 */
template <typename T> void unsafeSetupVector(T *arr, T low, T high, int size) {
  for (int i = 0; i < size; ++i) {
    arr[i] = low + double(i) / double(size - 1) * (high - low);
  }
}

// @brief Helper to setup vectors from ranges. Operates on copying output buffer
// size
template <class T> void unsafeSetupVector(T *arr, const std::vector<T> &vec, size_t size) {
  static_assert(std::is_copy_constructible_v<T>, "Template type T must be copyable");
  std::copy_n(vec.begin(), size, arr);
}

/**
 * @brief Print vector contents to console
 * @tparam T Type that supports operator<<
 * @param vec Vector to print
 */
template <typename T> void printVector(const std::vector<T> &vec) {
  std::cout << "[ ";
  for (const auto &element : vec) {
    std::cout << element << " ";
  }
  std::cout << "]" << std::endl;
}


/**
 * @brief Safely cast double to float with range and precision checks
 * @throws std::overflow_error if value exceeds float range
 * @throws std::runtime_error if significant precision loss occurs
 */
inline float safe_double_to_float(double value, bool strict = false) {
  // Check for infinity and NaN
  if (std::isinf(value) || std::isnan(value)) {
    return static_cast<float>(value); // Preserves special values
  }

  // Check range
  constexpr double max_float = std::numeric_limits<float>::max();
  constexpr double min_float = std::numeric_limits<float>::lowest();

  if (value > max_float || value < min_float) {
    throw std::overflow_error("Double value exceeds float range");
  }

  auto result = static_cast<float>(value);

  // Optional: check for significant precision loss
  if (strict) {
    double reconstructed = static_cast<double>(result);
    double relative_error = std::abs((value - reconstructed) / value);

    if (relative_error > 1e-6) { // ~float precision threshold
      throw std::runtime_error("Significant precision loss in double->float conversion");
    }
  }

  return result;
}

} // namespace acoustics