//
// Created by tko on 2/6/26.
//

#include "pch.h"

#include "helpers.h"

namespace acoustics {
namespace utils {
namespace {
constexpr double kBoundaryEpsilon =
    std::numeric_limits<double>::epsilon() * 100;
}

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
    if (delta <= kBoundaryEpsilon || vec[i] <= vec[i - 1]) {
      return false;
    }
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