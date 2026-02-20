//
// Created by tko on 2/4/26.
//

#include "../src/acoustics/include/acoustics/Grid.h"
#include "acoustics/helpers.h"
#include "catch2/matchers/catch_matchers_vector.hpp"
#include <stdexcept>

#include <Eigen/Dense>
#include <catch2/catch_approx.hpp>
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
TEST_CASE("Grid Incorrect construction", "[grid]") {
  std::vector<double> x = {0.0, 1.0, 2.0};
  std::vector<double> y = {0.0, 1.0};
  std::vector<double> data = {1500.0, 1501.0}; // Incorrect size
  REQUIRE_THROWS_AS((acoustics::Grid2D(x, y, data)), std::invalid_argument);
  std::vector<double> z = {1.0, 2.0};
  REQUIRE_THROWS_AS((acoustics::Grid3D(x, y, z, data)), std::invalid_argument);
}

// Grid Testing
class GridTestsFixture {
public:
  GridTestsFixture() {}
  acoustics::Grid2D get2DGrid() {
    std::vector<double> x2D = {0.0, 1.0};
    std::vector<double> y2D = {0.0, 1.0};
    auto grid2D_ = acoustics::Grid2D(x2D, y2D, 1500.0);
    return grid2D_;
  }
  acoustics::Grid3D get3DGrid() {
    std::vector<double> x3D = {0.0, 1.0};
    std::vector<double> y3D = {0.0, 1.0};
    std::vector<double> z3D = {10.0, 1000.0};
    auto grid3D_ = acoustics::Grid3D(x3D, y3D, z3D, 1500.0);
    return grid3D_;
  }
};
TEST_CASE_METHOD(GridTestsFixture, "Grid's within each other", "[grid]") {
  GridTestsFixture test = GridTestsFixture();
  auto grid2D = test.get2DGrid();
  auto grid3D = test.get3DGrid();
  auto isInsideResult = grid2D.checkInside(grid3D);
  REQUIRE(isInsideResult == true);
}
TEST_CASE_METHOD(GridTestsFixture, "Grid's not within each other", "[grid]") {
  // x plus case
  GridTestsFixture test = GridTestsFixture();
  auto grid2D = test.get2DGrid();
  auto grid3D = test.get3DGrid();
  grid2D.xCoords[1] = 2.0; // Modify grid2D to be outside of grid3D
  INFO("Grid value: " << grid2D.xCoords[1]);
  auto isInsideResult = grid2D.checkInside(grid3D);
  CHECK(isInsideResult == false);

  // x minus case
  test = GridTestsFixture();
  grid2D = test.get2DGrid();
  grid3D = test.get3DGrid();
  grid2D.xCoords[0] = -12.0; // Modify grid2D to be outside of grid3D
  isInsideResult = grid2D.checkInside(grid3D);
  CHECK(isInsideResult == false);

  // y plus case
  test = GridTestsFixture();
  grid2D = test.get2DGrid();
  grid3D = test.get3DGrid();
  grid2D.yCoords[1] = 2.5; // Modify grid2D to be outside of grid3D
  isInsideResult = grid2D.checkInside(grid3D);
  CHECK(isInsideResult == false);

  test = GridTestsFixture();
  grid2D = test.get2DGrid();
  grid3D = test.get3DGrid();
  grid2D.yCoords[0] = -2.5; // Modify grid2D to be outside of grid3D
  isInsideResult = grid2D.checkInside(grid3D);
  CHECK(isInsideResult == false);

  // one 3d case
  test = GridTestsFixture();
  grid2D = test.get2DGrid();
  grid3D = test.get3DGrid();
  grid3D.yCoords[1] = 0.0;   // Modify grid2D to be outside of grid3D
  grid3D.yCoords[0] = -10.0; // Modify grid2D to be outside of grid3D
  isInsideResult = grid2D.checkInside(grid3D);
  CHECK(isInsideResult == false);
}
TEST_CASE_METHOD(GridTestsFixture, "Grid's Monotonic", "[grid]") {
  // x plus case
  GridTestsFixture test = GridTestsFixture();
  auto grid2D = test.get2DGrid();
  grid2D.xCoords[0] = 1.0;
  grid2D.xCoords[1] = 0.0;
  CHECK_THROWS_AS(
      (acoustics::Grid2D(grid2D.xCoords, grid2D.yCoords, grid2D.data)),
      std::invalid_argument);

  test = GridTestsFixture();
  grid2D = test.get2DGrid();
  grid2D.xCoords[0] = 0.0;
  grid2D.xCoords[1] = 0.1;
  CHECK_NOTHROW(
      (acoustics::Grid2D(grid2D.xCoords, grid2D.yCoords, grid2D.data)));
  test = GridTestsFixture();
  grid2D = test.get2DGrid();
  grid2D.xCoords[0] = 0.0;
  grid2D.xCoords[1] = 0.0;
  CHECK_THROWS_AS(
      (acoustics::Grid2D(grid2D.xCoords, grid2D.yCoords, grid2D.data)),
      std::invalid_argument);
}

TEST_CASE("Bounds checking on position", "[position]") {
  Eigen::Vector3d minBounds(0.0, 0.0, 0.0);
  Eigen::Vector3d maxBounds(100.0, 100.0, 100.0);

  // Test position within bounds
  Eigen::Vector3d position1(50.0, 50.0, 50.0);
  REQUIRE(acoustics::utils::positionInBounds(position1, minBounds, maxBounds) == true);

  // Test position on the edge of bounds
  Eigen::Vector3d position2(0.0, 0.0, 0.0);
  REQUIRE(acoustics::utils::positionInBounds(position2, minBounds, maxBounds) == true);

  Eigen::Vector3d position3(100.0, 100.0, 100.0);
  REQUIRE(acoustics::utils::positionInBounds(position3, minBounds, maxBounds) == true);

  // Test position outside bounds
  Eigen::Vector3d position4(-1.0, -1.0, -1.0);
  REQUIRE(acoustics::utils::positionInBounds(position4, minBounds, maxBounds) == false);

  Eigen::Vector3d position5(101.0, 101.0, 101.0);
  REQUIRE(acoustics::utils::positionInBounds(position5, minBounds, maxBounds) == false);
}

TEST_CASE_METHOD(GridTestsFixture, "Interpolation on flat 2D grid", "[interpolation]") {
  auto grid2D = get2DGrid();
  double x = 0.5;
  double y = 0.5;
  double interpolatedValue = grid2D.interpolateDataValue(x, y);
  REQUIRE(interpolatedValue == Catch::Approx(1500.0));
}

TEST_CASE_METHOD(GridTestsFixture, "Interpolation on 2D grid sloped in x", "[interpolation]") {
  auto grid2D = get2DGrid();
  grid2D.data = {1500.0, 1501.0, 1500.0, 1501.0};

  double x = 0.5;
  double y = 0.5;
  double interpolatedValue = grid2D.interpolateDataValue(x, y);
  REQUIRE(interpolatedValue == Catch::Approx(1500.5));
}

TEST_CASE_METHOD(GridTestsFixture, "Interpolation on 2D grid sloped in y", "[interpolation]") {
  auto grid2D = get2DGrid();
  grid2D.data = {1500.0, 1500.0, 1501.0, 1501.0};

  double x = 0.5;
  double y = 0.5;
  double interpolatedValue = grid2D.interpolateDataValue(x, y);
  REQUIRE(interpolatedValue == Catch::Approx(1500.5));
}
