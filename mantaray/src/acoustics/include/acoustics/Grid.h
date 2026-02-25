/**
 * @file Grid.h
 * @brief Library internal data structures for representing 2 and 3D grids
 */
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "acoustics/helpers.h"

namespace acoustics {

// Forward declaration
class Grid3D;

// ============================================================================
// Grid2D - 2D Regular Grid Storage
// ============================================================================
/**
 * @brief 2D grid with contiguous memory layout matching Bellhop's internal
 * order
 *
 * @par Memory layout:
 * - Coordinates: SoA (separate xCoords, yCoords arrays)
 * - Values: Single flat array, row-major order
 * - data[ix * ny + iy] = value at (ix, iy)
 *
 * @par Index order:
 * - index(ix, iy) = ix * ny + iy  (row-major)
 * - Must match Bellhop's Fortran layout when interfacing
 *
 * @par Invariants:
 * - xCoords.size() > 0 && yCoords.size() > 0
 * - data.size() == nx() * ny()
 * - Coordinates must be monotonically increasing (strictly)
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

  Grid2D(std::vector<double> x, std::vector<double> y,
         double defaultValue = double{});
  Grid2D(std::vector<double> x, std::vector<double> y,
         std::vector<double> initData);

  void clear();

  size_t nx() const;
  size_t ny() const;
  size_t size() const;

  /**
   * @brief Index Structure MUST Align with Bellhop's Internal Storage Order
   */
  size_t index(size_t ix, size_t iy) const;

  double &at(size_t ix, size_t iy);
  const double &at(size_t ix, size_t iy) const;

  double &operator()(size_t ix, size_t iy);
  const double &operator()(size_t ix, size_t iy) const;

  bool isValid() const;

  /** @brief Returns axis aligned bounding box representation of grid */
  std::pair<Eigen::Vector2d, Eigen::Vector2d> boundingBox() const;

  /** @brief Checks if this grid is completely inside other grid */
  bool checkInside(const Grid3D &other) const;

  /** @brief Checks if this grid contains other grid */
  bool checkContain(const Grid3D &other) const;

  /** @brief Interpolates bathymetry linearly and returns depth of bathymetry
   * @details if this represents bathymetry, this ensures it is not below the
   * "floor"
   */
  double interpolateDataValue(double x, double y) const;

private:
  void validateInitialization() const;
  void boundsCheck(size_t ix, size_t iy) const;
};

/**
 * @brief 3D grid - same design principles as Grid2D
 *
 * @sa Grid2D
 * @see Grid2D
 *
 * @note Must match Bellhop's Fortran layout when interfacing
 *
 * @par INDEX ORDER:
 * - index(ix, iy, iz) = (ix * ny + iy) * nz + iz  (row-major)
 * - See Grid2D documentation for memory layout rationale
 *
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

  /** @brief Returns axis aligned bounding box representation of grid */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingBox() const;

  /** @brief Checks if this grid is completely inside other grid */
  bool checkInside(const Grid3D &other) const;

private:
  void validateInitialization() const;
  void boundsCheck(size_t ix, size_t iy, size_t iz) const;
};

/**
 * @brief 4D grid - same design principles as Grid2D
 *
 * @sa Grid2D
 * @see Grid2D
 *
 *
 * @par INDEX ORDER:
 * - index(ix, iy, iz) = (ix * ny + iy) * nz + iz  (row-major)
 * - See Grid2D documentation for memory layout rationale
 *
 */
class GridVec {
public:
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<double> zCoords;
  std::vector<double> data;

  GridVec() = delete;
  GridVec(const GridVec &) = delete;
  GridVec &operator=(const GridVec &) = delete;
  GridVec(GridVec &&) noexcept = default;
  GridVec &operator=(GridVec &&) = default;

  GridVec(std::vector<double> x, std::vector<double> y, std::vector<double> z,
          double defaultValue = double{});
  GridVec(std::vector<double> x, std::vector<double> y, std::vector<double> z,
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

  /** @brief Returns axis aligned bounding box representation of grid */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingBox() const;

  /** @brief Interpolates vector field linearly
   */
  double interpolateDataValue(double x, double y) const;

private:
  void validateInitialization() const;
  void boundsCheck(size_t ix, size_t iy, size_t iz) const;
};

/** @brief Utilizes Munk profile equation to generate a sound speed profile */
void munkProfile(Grid3D &grid, double sofarSpeed, bool isKm);

/**
 * @brief Validates grids for all grid class via usage of ptr's
 *
 * @details Can take pointers as we only call this function with the class
 * where to pointers are valid. **WARNING** do not use outside Grid classes
 * as there is no certainty pointers are nulled.
 *
 * @invariant Assumes passed in values are in the x,y,z order or x,y
 */
void GridCheckViaPtr(const std::vector<const std::vector<double> *> &coords,
                     const std::vector<double> &data);

} // namespace acoustics
