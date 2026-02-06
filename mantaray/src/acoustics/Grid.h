// Grid.h
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "helpers.h"

namespace acoustics {

enum GridDimension { kGrid2D, kGrid3D };

// Forward declaration
class Grid3D;

/**
 * @brief Grid class for 2D grids for Bellhop
 * @detail The index structure MUST align with Bellhop's internal storage order
 */
class Grid2D {
public:
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<double> data;

  Grid2D() = delete;
  Grid2D(const Grid2D &) = delete;
  Grid2D &operator=(const Grid2D &) = delete;
  Grid2D(Grid2D &&) noexcept = default;
  Grid2D &operator=(Grid2D &&) = default;

  Grid2D(std::vector<double> x, std::vector<double> y, double defaultValue = double{});
  Grid2D(std::vector<double> x, std::vector<double> y, std::vector<double> initData);

  void clear();

  size_t nx() const;
  size_t ny() const;
  size_t size() const;

  /**
   * Index Structure MUST Align with Bellhop's Internal Storage Order
   */
  size_t index(size_t ix, size_t iy) const;

  double &at(size_t ix, size_t iy);
  const double &at(size_t ix, size_t iy) const;

  double &operator()(size_t ix, size_t iy);
  const double &operator()(size_t ix, size_t iy) const;

  bool isValid() const;
  std::pair<Eigen::Vector2d, Eigen::Vector2d> boundingBox() const;

  /** @brief Checks if this grid is completely inside other grid */
  bool checkInside(const Grid3D &other) const;

  /** @brief Checks if this grid contains other grid */
  bool checkContain(const Grid3D &other) const;

private:
  void validateInitialization() const;
  void boundsCheck(size_t ix, size_t iy) const;
};

/**
 * @brief Grid class for 3D grids for Bellhop
 * @detail The index structure MUST align with Bellhop's internal storage order
 */
class Grid3D {
public:
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<double> zCoords;
  std::vector<double> data;

  Grid3D() = delete;
  Grid3D(const Grid3D &) = delete;
  Grid3D &operator=(const Grid3D &) = delete;
  Grid3D(Grid3D &&) noexcept = default;
  Grid3D &operator=(Grid3D &&) = default;

  Grid3D(std::vector<double> x, std::vector<double> y, std::vector<double> z,
         double defaultValue = double{});
  Grid3D(std::vector<double> x, std::vector<double> y, std::vector<double> z,
         std::vector<double> initData);

  void clear();

  size_t nx() const;
  size_t ny() const;
  size_t nz() const;
  size_t size() const;

  /**
   * Index Structure MUST Align with Bellhop's Internal Storage Order
   */
  size_t index(size_t ix, size_t iy, size_t iz) const;

  double &at(size_t ix, size_t iy, size_t iz);
  const double &at(size_t ix, size_t iy, size_t iz) const;

  double &operator()(size_t ix, size_t iy, size_t iz);
  const double &operator()(size_t ix, size_t iy, size_t iz) const;

  bool isValid() const;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingBox() const;

  /** @brief Checks if this grid is completely inside other grid */
  bool checkInside(const Grid3D &other) const;

private:
  void validateInitialization() const;
  void boundsCheck(size_t ix, size_t iy, size_t iz) const;
};

void munkProfile(Grid3D &grid, double sofarSpeed, bool isKm);

} // namespace acoustics