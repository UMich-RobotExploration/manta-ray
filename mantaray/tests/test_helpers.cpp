//
// Created by tko on 2/4/26.
//

#include "acoustics/helpers.h"
#include "catch2/matchers/catch_matchers_vector.hpp"
#include "rb/helpers.h"

#include <Eigen/Dense>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <cmath>
using Catch::Approx;

TEST_CASE("Linspace generates correct number of points", "[linspace]") {
  auto result = acoustics::utils::linspace(0.0, 1.0, 5);
  REQUIRE(result.size() == 5);
  std::vector<double> expected = {0.0, 0.25, 0.5, 0.75, 1.0};
  REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
  result = acoustics::utils::linspace(-1.0, 1.0, 5);
  expected = {-1.0, -0.5, 0.0, 0.5, 1.0};
  REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
  result = acoustics::utils::linspace(1.0, -1.0, 5);
  expected = {1.0, 0.5, 0.0, -0.5, -1.0};
  REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("Bounds checking on position", "[position]") {
  Eigen::Vector3d minBounds(0.0, 0.0, 0.0);
  Eigen::Vector3d maxBounds(100.0, 100.0, 100.0);

  // Test position within bounds
  Eigen::Vector3d position1(50.0, 50.0, 50.0);
  REQUIRE(acoustics::utils::positionInBounds(position1, minBounds, maxBounds) ==
          true);

  // Test position on the edge of bounds
  Eigen::Vector3d position2(0.0, 0.0, 0.0);
  REQUIRE(acoustics::utils::positionInBounds(position2, minBounds, maxBounds) ==
          true);

  Eigen::Vector3d position3(100.0, 100.0, 100.0);
  REQUIRE(acoustics::utils::positionInBounds(position3, minBounds, maxBounds) ==
          true);

  // Test position outside bounds
  Eigen::Vector3d position4(-1.0, -1.0, -1.0);
  REQUIRE(acoustics::utils::positionInBounds(position4, minBounds, maxBounds) ==
          false);

  Eigen::Vector3d position5(101.0, 101.0, 101.0);
  REQUIRE(acoustics::utils::positionInBounds(position5, minBounds, maxBounds) ==
          false);
}


TEST_CASE("computeNumTimeSteps computes correct number of steps", "[helpers]") {
  using rb::computeNumTimeSteps;

  // Basic cases
  REQUIRE(computeNumTimeSteps(1.0, 1.0) == 2);
  REQUIRE(computeNumTimeSteps(1.0, 2.0) == 3);
  REQUIRE(computeNumTimeSteps(2.0, 2.0) == 5);

  // Edge cases
  REQUIRE(computeNumTimeSteps(0.0, 10.0) == 1);
  REQUIRE(computeNumTimeSteps(1.0, 0.0) == 1);

  // Non-integer frequency
  REQUIRE(computeNumTimeSteps(1.0, 1.5) == 3);

  // Large values
  REQUIRE(computeNumTimeSteps(10.0, 100.0) == 1001);
}

////////////////////////////////////////////////////////////////////////////////
// Floating point comparison helpers
////////////////////////////////////////////////////////////////////////////////

TEST_CASE("isEqual handles exact and near-zero equality", "[fp]") {
  using rb::detail::isEqual;

  // Exact equality
  CHECK(isEqual(1.0, 1.0));
  CHECK(isEqual(0.0, 0.0));
  CHECK(isEqual(-5.0, -5.0));

  // Both near zero — within kFloatingPointToleranceNearZero
  CHECK(isEqual(1e-13, 0.0));
  CHECK(isEqual(0.0, 1e-13));
  CHECK(isEqual(1e-13, -1e-13));

  // Not near zero — outside tolerance
  CHECK_FALSE(isEqual(1e-9, 0.0));
  CHECK_FALSE(isEqual(0.0, 1e-9));
}

TEST_CASE("isEqual handles relative tolerance", "[fp]") {
  using rb::detail::isEqual;

  // Within relative tolerance (1e-10 * maxAbs)
  // 1000 * 1e-10 = 1e-7, so 1e-8 diff is within tolerance
  CHECK(isEqual(1000.0, 1000.0 + 1e-8));

  // Outside relative tolerance
  CHECK_FALSE(isEqual(1000.0, 1001.0));
  CHECK_FALSE(isEqual(1.0, 2.0));
}

TEST_CASE("validDeltaTMultiple identifies correct multiples", "[fp]") {
  using rb::detail::validDeltaTMultiple;

  // Exact multiples
  CHECK(validDeltaTMultiple(0.0, 0.1));   // 0 is a multiple of anything
  CHECK(validDeltaTMultiple(100.0, 100.0)); // 1x
  CHECK(validDeltaTMultiple(200.0, 100.0)); // 2x
  CHECK(validDeltaTMultiple(1.0, 0.1));     // 10x

  // Not multiples
  CHECK_FALSE(validDeltaTMultiple(0.15, 0.1));
  CHECK_FALSE(validDeltaTMultiple(50.0, 100.0));
  CHECK_FALSE(validDeltaTMultiple(0.05, 0.1));
}

TEST_CASE("validDeltaTMultiple handles floating-point accumulation", "[fp]") {
  using rb::detail::validDeltaTMultiple;

  // Simulate the advanceWorld bug: 100 subtractions of 0.1 from 10.0
  // In IEEE 754, 0.1 is not exact, so the residual may not be exactly 0.0
  double accumulated = 10.0;
  for (int i = 0; i < 100; ++i) {
    accumulated -= 0.1;
  }
  // accumulated is ~0 but may be a tiny positive or negative residual
  CAPTURE(accumulated);

  // The function should recognize this as effectively 0 (a valid multiple)
  CHECK(validDeltaTMultiple(accumulated, 0.1));

  // Sensor timing: 0.01 Hz sensor → period = 100s
  // After accumulating steps, t=100 should be a valid multiple
  CHECK(validDeltaTMultiple(100.0, 100.0));
  // t=99.9 should NOT be a valid multiple of 100
  CHECK_FALSE(validDeltaTMultiple(99.9, 100.0));
}

////////////////////////////////////////////////////////////////////////////////
// Beam geometry utilities
////////////////////////////////////////////////////////////////////////////////

TEST_CASE("computeElevationAngle - horizontal", "[beam]") {
  using acoustics::utils::computeElevationAngle;

  // Purely horizontal: delta in X only → elevation = 0
  Eigen::Vector3d horizontal(1000, 0, 0);
  CHECK(computeElevationAngle(horizontal) == Approx(0.0).margin(1e-10));

  // Horizontal in XY plane
  Eigen::Vector3d diagonal(1000, 1000, 0);
  CHECK(computeElevationAngle(diagonal) == Approx(0.0).margin(1e-10));
}

TEST_CASE("computeElevationAngle - vertical", "[beam]") {
  using acoustics::utils::computeElevationAngle;

  // Straight down: +Z only → elevation = +π/2
  Eigen::Vector3d down(0, 0, 500);
  CHECK(computeElevationAngle(down) == Approx(M_PI / 2.0).margin(1e-10));

  // Straight up: -Z only → elevation = -π/2
  Eigen::Vector3d up(0, 0, -500);
  CHECK(computeElevationAngle(up) == Approx(-M_PI / 2.0).margin(1e-10));
}

TEST_CASE("computeElevationAngle - 45 degrees", "[beam]") {
  using acoustics::utils::computeElevationAngle;

  // Equal horizontal and vertical → 45°
  Eigen::Vector3d diag(1000, 0, 1000);
  CHECK(computeElevationAngle(diag) == Approx(M_PI / 4.0).margin(1e-10));
}

TEST_CASE("computeBeamBox - symmetric case", "[beam]") {
  using acoustics::utils::computeBeamBox;

  // Equal X and Y displacement, no Z
  Eigen::Vector3d delta(1000, 1000, 0);
  auto box = computeBeamBox(delta, 1.5, 1.0 / 150.0);

  // scaled delta = 1.5 * 1000 = 1500 each axis
  // range = sqrt(2)*1000 ≈ 1414
  // minDim = max(1414 * 1.5 * 0.5, 100) = max(1060, 100) = 1060
  // boxX = max(1500, 1060) = 1500
  CHECK(box.boxX == Approx(1500.0));
  CHECK(box.boxY == Approx(1500.0));
  CHECK(box.stepSize == Approx(delta.norm() / 150.0));
}

TEST_CASE("computeBeamBox - nearly aligned pair uses proportional floor",
          "[beam]") {
  using acoustics::utils::computeBeamBox;

  // Almost pure Y displacement — tiny X component
  // This is the bug case: without floor, boxX would be tiny
  Eigen::Vector3d delta(10, 10000, 0);
  auto box = computeBeamBox(delta, 1.5, 1.0 / 150.0);

  // range ≈ 10000
  // minDim = max(10000 * 1.5 * 0.5, 100) = max(7500, 100) = 7500
  // scaled X = 1.5 * 10 = 15 → boxX = max(15, 7500) = 7500
  CHECK(box.boxX == Approx(7500.0).margin(1.0));
  // scaled Y = 1.5 * 10000 = 15000 → boxY = max(15000, 7500) = 15000
  CHECK(box.boxY == Approx(15000.0));
}

TEST_CASE("computeBeamBox - very short range uses absolute floor", "[beam]") {
  using acoustics::utils::computeBeamBox;

  // Very short range: 10m
  Eigen::Vector3d delta(10, 0, 0);
  auto box = computeBeamBox(delta, 1.5, 1.0 / 150.0);

  // range = 10, minDim = max(10 * 1.5 * 0.5, 100) = max(7.5, 100) = 100
  // scaled X = 15 → boxX = max(15, 100) = 100
  CHECK(box.boxX == Approx(100.0));
  CHECK(box.boxY == Approx(100.0));
}
