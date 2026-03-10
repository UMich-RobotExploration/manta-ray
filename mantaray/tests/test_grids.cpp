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

// ============================================================================
// GridVec (trilinear) Interpolation Tests
// ============================================================================

class GridVecTestsFixture {
public:
  /**
   * @brief Builds a 2x2x2 GridVec with uniform Vector2d values
   */
  acoustics::GridVec getUniformGridVec(Eigen::Vector2d fillValue = {1.0,
                                                                    2.0}) {
    std::vector<double> x = {0.0, 1.0};
    std::vector<double> y = {0.0, 1.0};
    std::vector<double> z = {0.0, 1.0};
    // 2*2*2 = 8 elements, row-major: index(ix,iy,iz) = (ix*ny + iy)*nz + iz
    std::vector<Eigen::Vector2d> data(8, fillValue);
    return acoustics::GridVec(std::move(x), std::move(y), std::move(z),
                              std::move(data));
  }
};

TEST_CASE_METHOD(GridVecTestsFixture,
                 "GridVec interpolation on uniform field returns constant",
                 "[interpolation]") {
  auto grid = getUniformGridVec({3.0, 5.0});
  Eigen::Vector3d result = grid.interpolateDataValue(0.5, 0.5, 0.5);
  // Uniform field → interpolation at the center should return the same value
  REQUIRE(result.x() == Catch::Approx(3.0));
  REQUIRE(result.y() == Catch::Approx(5.0));
  REQUIRE(result.z() == Catch::Approx(0.0)); // z component is always 0
}

TEST_CASE_METHOD(GridVecTestsFixture,
                 "GridVec interpolation with linear gradient in x",
                 "[interpolation]") {
  auto grid = getUniformGridVec({0.0, 0.0});
  // Set data so that x=0 face has (0,0), x=1 face has (1,0)
  // index(ix, iy, iz) = (ix*2 + iy)*2 + iz
  for (size_t iy = 0; iy < 2; ++iy) {
    for (size_t iz = 0; iz < 2; ++iz) {
      grid.at(0, iy, iz) = Eigen::Vector2d(0.0, 0.0);
      grid.at(1, iy, iz) = Eigen::Vector2d(1.0, 0.0);
    }
  }
  Eigen::Vector3d result = grid.interpolateDataValue(0.5, 0.5, 0.5);
  REQUIRE(result.x() == Catch::Approx(0.5));
  REQUIRE(result.y() == Catch::Approx(0.0));
}

TEST_CASE_METHOD(GridVecTestsFixture,
                 "GridVec interpolation with linear gradient in y",
                 "[interpolation]") {
  auto grid = getUniformGridVec({0.0, 0.0});
  for (size_t ix = 0; ix < 2; ++ix) {
    for (size_t iz = 0; iz < 2; ++iz) {
      grid.at(ix, 0, iz) = Eigen::Vector2d(0.0, 0.0);
      grid.at(ix, 1, iz) = Eigen::Vector2d(0.0, 4.0);
    }
  }
  Eigen::Vector3d result = grid.interpolateDataValue(0.5, 0.5, 0.5);
  REQUIRE(result.x() == Catch::Approx(0.0));
  REQUIRE(result.y() == Catch::Approx(2.0));
}

TEST_CASE_METHOD(GridVecTestsFixture,
                 "GridVec interpolation with linear gradient in z",
                 "[interpolation]") {
  auto grid = getUniformGridVec({0.0, 0.0});
  for (size_t ix = 0; ix < 2; ++ix) {
    for (size_t iy = 0; iy < 2; ++iy) {
      grid.at(ix, iy, 0) = Eigen::Vector2d(0.0, 0.0);
      grid.at(ix, iy, 1) = Eigen::Vector2d(2.0, 6.0);
    }
  }
  Eigen::Vector3d result = grid.interpolateDataValue(0.5, 0.5, 0.5);
  REQUIRE(result.x() == Catch::Approx(1.0));
  REQUIRE(result.y() == Catch::Approx(3.0));
}

TEST_CASE_METHOD(GridVecTestsFixture,
                 "GridVec interpolation out of bounds throws",
                 "[interpolation]") {
  auto grid = getUniformGridVec();
  // Below lower bound on x
  REQUIRE_THROWS_AS(grid.interpolateDataValue(-0.1, 0.5, 0.5),
                    std::runtime_error);
  // Above upper bound on y
  REQUIRE_THROWS_AS(grid.interpolateDataValue(0.5, 1.5, 0.5),
                    std::runtime_error);
  // Above upper bound on z
  REQUIRE_THROWS_AS(grid.interpolateDataValue(0.5, 0.5, 1.5),
                    std::runtime_error);
}
