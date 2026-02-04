//
// Created by tko on 2/4/26.
//

#include "acoustics/Grid.h"
#include "acoustics/helpers.h"
#include "catch2/matchers/catch_matchers_vector.hpp"
#include <stdexcept>

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
TEST_CASE("Grid Incorrect construction", "[grid]") {
  std::vector<double> x = {0.0, 1.0, 2.0};
  std::vector<double> y = {0.0, 1.0};
  std::vector<double> data = {1500.0, 1501.0}; // Incorrect size
  REQUIRE_THROWS_AS((acoustics::Grid2D<double>(x, y, data)),
                    std::invalid_argument);
}

// Grid Testing
class GridTestsFixture {
public:
  GridTestsFixture() {}
  acoustics::Grid2D<double> get2DGrid() {
    std::vector<double> x2D = {0.0, 1.0};
    std::vector<double> y2D = {0.0, 1.0};
    auto grid2D_ = acoustics::Grid2D<double>(x2D, y2D, 1500.0);
    return grid2D_;
  }
  acoustics::Grid3D<double> get3DGrid() {
    std::vector<double> x3D = {0.0, 1.0};
    std::vector<double> y3D = {0.0, 1.0};
    std::vector<double> z3D = {0.0, 100.0};
    auto grid3D_ = acoustics::Grid3D<double>(x3D, y3D, z3D, 1500.0);
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