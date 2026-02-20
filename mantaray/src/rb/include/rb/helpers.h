//
// Created by tko on 2/18/26.
//

#pragma once

namespace rb {
constexpr double kBoundaryEpsilonDouble =
    std::numeric_limits<double>::epsilon() * 100;
namespace detail {
bool isEqual(double x, double y);

/**
 * @brief Checks to see if time is a valid frequency = 1/dt multiple.
 * @details Handles floating point complexity
 * - Evaluates 2 cases:
 *  - A remainder near zero,
 *  - A remainder near dt.
 */
bool validDeltaTMultiple(double time, double dt);

} // namespace detail
} // namespace rb