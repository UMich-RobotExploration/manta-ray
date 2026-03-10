//
// Grid-specific tests, extracted from test_helpers.cpp
//

#include "acoustics/Grid.h"
#include "acoustics/helpers.h"

#include <Eigen/Dense>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <stdexcept>

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

TEST_CASE_METHOD(GridTestsFixture, "Interpolation on flat 2D grid",
                 "[interpolation]") {
  auto grid2D = get2DGrid();
  double x = 0.5;
  double y = 0.5;
  double interpolatedValue = grid2D.interpolateDataValue(x, y);
  REQUIRE(interpolatedValue == Catch::Approx(1500.0));
}

TEST_CASE_METHOD(GridTestsFixture, "Interpolation on 2D grid sloped in x",
                 "[interpolation]") {
  auto grid2D = get2DGrid();
  grid2D.data = {1500.0, 1501.0, 1500.0, 1501.0};

  double x = 0.5;
  double y = 0.5;
  double interpolatedValue = grid2D.interpolateDataValue(x, y);
  REQUIRE(interpolatedValue == Catch::Approx(1500.5));
}

TEST_CASE_METHOD(GridTestsFixture, "Interpolation on 2D grid sloped in y",
                 "[interpolation]") {
  auto grid2D = get2DGrid();
  grid2D.data = {1500.0, 1500.0, 1501.0, 1501.0};

  double x = 0.5;
  double y = 0.5;
  double interpolatedValue = grid2D.interpolateDataValue(x, y);
  REQUIRE(interpolatedValue == Catch::Approx(1500.5));
}


