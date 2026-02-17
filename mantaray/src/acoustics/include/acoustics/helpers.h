//
// Created by tko on 1/28/26.
//
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <bhc/bhc.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace acoustics {

constexpr double kBoundaryEpsilonDouble =
    std::numeric_limits<double>::epsilon() * 100;

/**
 * @brief Generate linearly spaced values between start and end
 * @tparam T Numeric type (typically float or double)
 * @param start Starting value
 * @param end Ending value
 * @param num Number of points to generate
 * @return Vector of linearly spaced values
 */
namespace utils {
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
    arr[i] = low + static_cast<double>(i) / static_cast<double>(size - 1) *
                       (high - low);
  }
}

// @brief Helper to setup vectors from ranges. Operates on copying output buffer
// size
template <class T>
void unsafeSetupVector(T *arr, const std::vector<T> &vec, size_t size) {
  static_assert(std::is_copy_constructible_v<T>,
                "Template type T must be copyable");
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

/* @brief Helper to check if value is approximately equal OR satisfies
 * comparison
 * @tparam Vec1, Vec2 Vector types (e.g., Eigen::Vector2d or Eigen::Vector3d)
 */
template <typename Vec1, typename Vec2, typename CompOp>
bool eigenFloatSafeComparison(const Vec1 &vec1, const Vec2 &vec2, CompOp comp) {
  return vec1.isApprox(vec2, kBoundaryEpsilonDouble) || comp(vec1, vec2);
}

/**
 * @brief Safely compare two Eigen vectors with a custom comparator.
 * @details Uses minimum bounding box check
 *
 * @returns true if in bounds, false otherwise
 */
bool positionInBounds(const Eigen::Vector3d &position,
                      const Eigen::Vector3d &min, const Eigen::Vector3d &max);

/**
 * @brief Check if a vector of doubles is monotonically increasing
 * @details Uses an epsilon threshold to account for floating-point precision
 * issues.
 * @param vec Vector to check
 * @return true if vec is monotonically increasing, false otherwise
 */
bool isMonotonicallyIncreasing(const std::vector<double> &vec);

/**
 * @brief Safely cast double to float with range and precision checks
 * @throws std::overflow_error if value exceeds float range
 * @throws std::runtime_error if significant precision loss occurs
 */
float safeDoubleToFloat(double value, bool strict = false);

bhc::VEC23<true> safeEigenToVec23(const Eigen::Vector3d &vec,
                                  bool strict = false);

} // namespace utils
} // namespace acoustics