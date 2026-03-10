//
// Created by tko on 2/4/26.
//

#include "acoustics/helpers.h"
#include "catch2/matchers/catch_matchers_vector.hpp"
#include "rb/helpers.h"

#include <Eigen/Dense>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>

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
