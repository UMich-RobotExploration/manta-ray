//
// Created by tko on 1/28/26.
//
#pragma once
#include <iostream>
#include <vector>

namespace acoustics {

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
 * @note No bounds checking is performed. Caller must ensure arr has sufficient space.
 */
template <typename T> void SetupVector(T *arr, T low, T high, int size) {
  for (int i = 0; i < size; ++i) {
    arr[i] = low + double(i) / double(size - 1) * (high - low);
  }
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

} // namespace acoustics