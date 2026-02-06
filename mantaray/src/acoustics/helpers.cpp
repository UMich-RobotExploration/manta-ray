//
// Created by tko on 2/6/26.
//

#include "include/acoustics/pch.h"

#include "include/acoustics/helpers.h"

namespace acoustics {
namespace utils {
bool isMonotonicallyIncreasing(const std::vector<double> &vec) {
  size_t size = vec.size();
  if (size == 1) {
    return true;
  } else if (size == 0) {
    return false;
  }
  // now we are only dealing with vecs.size() >= 2 so we can safely assume
  // iterations start at 1
  for (size_t i = 1; i < size; ++i) {
    double delta = std::abs(vec[i] - vec[i - 1]);
    if (delta <= kBoundaryEpsilonDouble || vec[i] <= vec[i - 1]) {
      return false;
    }
  }
  return true;
}

bool positionInBounds(const Eigen::Vector3d &position, const Eigen::Vector3d &min,
                      const Eigen::Vector3d &max) {
  bool isMinValid = utils::eigenFloatSafeComparison(
      position, min, [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });
  bool isMaxValid = utils::eigenFloatSafeComparison(
      position, max, [](const auto &a, const auto &b) {
        return (a.array() <= b.array()).all();
      });

  if (!isMinValid || !isMaxValid) {
    return false;
  }
  return true;
}

/**
 * @brief Safely cast double to float with range and precision checks
 * @throws std::overflow_error if value exceeds float range
 * @throws std::runtime_error if significant precision loss occurs
 */
float safe_double_to_float(double value, bool strict) {
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
      throw std::runtime_error(
          "Significant precision loss in double->float conversion");
    }
  }

  return result;
}
} // namespace utils
} // namespace acoustics