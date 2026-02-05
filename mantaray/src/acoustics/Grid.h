// Grid3D.h
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace acoustics {

enum GridDimension { kGrid2D, kGrid3D };

// TODO: Create enum for template
/**
 * @brief Grid class for 2D and 3D grids or Bellhop
 * @detail The index structure MUST align with Bellhop's internal storage order
 *
 * @tparam Is3D 3D or 2D grid
 * @tparam T double or float
 */
template <GridDimension, typename T> class Grid;

// 2D Grid Specialization
template <typename T> class Grid<kGrid2D, T> {
public:
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<T> data;

  Grid() = delete;
  Grid(const Grid &) = delete;
  Grid &operator=(const Grid &) = delete;
  Grid(Grid &&) noexcept = default;
  Grid &operator=(Grid &&) = default;

  Grid(std::vector<double> x, std::vector<double> y, T defaultValue = T{})
      : xCoords(x),
        yCoords(y),
        data(xCoords.size() * yCoords.size(), defaultValue) {
    validateInitialization();
  }
  Grid(std::vector<double> x, std::vector<double> y, std::vector<T> initData)
      : xCoords(x), yCoords(y), data(initData) {
    validateInitialization();
  }
  void clear() {
    xCoords.clear();
    yCoords.clear();
    data.clear();
  }

  inline size_t nx() const { return xCoords.size(); }
  inline size_t ny() const { return yCoords.size(); }
  inline size_t size() const { return data.size(); }

  /**
   * Index Structure MUST Align with Bellhop's Internal Storage Order
   */
  inline size_t index(size_t ix, size_t iy) const {
    return ix * yCoords.size() + iy;
  }

  T &at(size_t ix, size_t iy) {
    boundsCheck(ix, iy);
    return data[index(ix, iy)];
  }

  const T &at(size_t ix, size_t iy) const {
    boundsCheck(ix, iy);
    return data[index(ix, iy)];
  }

  T &operator()(size_t ix, size_t iy) { return data[index(ix, iy)]; }

  const T &operator()(size_t ix, size_t iy) const {
    return data[index(ix, iy)];
  }

  bool isValid() const {
    return data.size() == xCoords.size() * yCoords.size() && !xCoords.empty() &&
           !yCoords.empty();
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> boundingBox() const {
    if (!isValid()) {
      throw std::runtime_error("Cannot compute bounding box of invalid grid");
    }
    // TODO: consider a monotonic check and then we can not do this here
    // these are sorted so technically we could just take the first and last
    // element but this is more robust to unsorted input
    auto [xMin, xMax] = std::minmax_element(xCoords.begin(), xCoords.end());
    auto [yMin, yMax] = std::minmax_element(yCoords.begin(), yCoords.end());
    auto minVec = Eigen::Vector2d(*xMin, *yMin);
    auto maxVec = Eigen::Vector2d(*xMax, *yMax);
    return std::pair(minVec, maxVec);
  }

  /** @brief Checks if this grid is completely inside other grid
   */
  bool checkInside(const Grid<kGrid3D, T> &other) const {
    auto [minOther, maxOther] = other.boundingBox();
    auto [minThis, maxThis] = boundingBox();

    const T epsilon = std::numeric_limits<T>::epsilon() * 100;

    // Extract only x and y components for 2D comparison
    Eigen::VectorBlock minOther2D = minOther(Eigen::seq(0, 1));
    auto maxOther2D = maxOther(Eigen::seq(0, 1));

    bool isFloatingPointError = minOther2D.isApprox(minThis, epsilon);
    bool isMinValid =
        isFloatingPointError || (minOther2D.array() <= minThis.array()).all();
    isFloatingPointError = maxOther2D.isApprox(maxThis, epsilon);
    bool isMaxValid =
        isFloatingPointError || (maxOther2D.array() >= maxThis.array()).all();

    return isMinValid && isMaxValid;
  }

  /** @brief Checks if this grid contains other grid
   */
  bool checkContain(const Grid<kGrid3D, T> &other) const {
    auto [minOther, maxOther] = other.boundingBox();
    auto [minThis, maxThis] = boundingBox();

    const T epsilon = std::numeric_limits<T>::epsilon() * 100;

    // Extract only x and y components for 2D comparison
    Eigen::VectorBlock minOther2D = minOther(Eigen::seq(0, 1));
    auto maxOther2D = maxOther(Eigen::seq(0, 1));

    bool isFloatingPointError = minOther2D.isApprox(minThis, epsilon);
    bool isMinValid =
        isFloatingPointError || (minOther2D.array() >= minThis.array()).all();
    isFloatingPointError = maxOther2D.isApprox(maxThis, epsilon);
    bool isMaxValid =
        isFloatingPointError || (maxOther2D.array() <= maxThis.array()).all();

    return isMinValid && isMaxValid;
  }

private:
  void validateInitialization() const {
    if (xCoords.empty() || yCoords.empty()) {
      throw std::runtime_error("Grid cannot have empty coordinate vectors");
    }
    if (data.size() != xCoords.size() * yCoords.size()) {
      throw std::invalid_argument("Grid data size mismatch");
    }
  }

  void boundsCheck(size_t ix, size_t iy) const {
    if (ix >= nx() || iy >= ny()) {
      std::stringstream msg;
      msg << "Grid index out of bounds: (" << ix << ", " << iy << ") for grid ("
          << nx() << ", " << ny() << ")";
      throw std::out_of_range(msg.str());
    }
  }
};

// 3D Grid Specialization
template <typename T> class Grid<kGrid3D, T> {
public:
  std::vector<double> xCoords;
  std::vector<double> yCoords;
  std::vector<double> zCoords;
  std::vector<T> data;

  Grid() = delete;
  Grid(const Grid &) = delete;
  Grid &operator=(const Grid &) = delete;
  Grid(Grid &&) noexcept = default;
  Grid &operator=(Grid &&) = default;

  Grid(std::vector<double> x, std::vector<double> y, std::vector<double> z,
       T defaultValue = T{})
      : xCoords(x),
        yCoords(y),
        zCoords(z),
        data(xCoords.size() * yCoords.size() * zCoords.size(), defaultValue) {
    validateInitialization();
  }
  Grid(std::vector<double> x, std::vector<double> y, std::vector<double> z, std::vector<T> initData)
      : xCoords(x), yCoords(y), zCoords(z), data(initData) {
    validateInitialization();
  }

  void clear() {
    xCoords.clear();
    yCoords.clear();
    zCoords.clear();
    data.clear();
  }
  inline size_t nx() const { return xCoords.size(); }
  inline size_t ny() const { return yCoords.size(); }
  inline size_t nz() const { return zCoords.size(); }
  inline size_t size() const { return data.size(); }

  /**
   * Index Structure MUST Align with Bellhop's Internal Storage Order
   */
  inline size_t index(size_t ix, size_t iy, size_t iz) const {
    return (ix * yCoords.size() + iy) * zCoords.size() + iz;
  }

  T &at(size_t ix, size_t iy, size_t iz) {
    boundsCheck(ix, iy, iz);
    return data[index(ix, iy, iz)];
  }

  const T &at(size_t ix, size_t iy, size_t iz) const {
    boundsCheck(ix, iy, iz);
    return data[index(ix, iy, iz)];
  }

  T &operator()(size_t ix, size_t iy, size_t iz) {
    return data[index(ix, iy, iz)];
  }

  const T &operator()(size_t ix, size_t iy, size_t iz) const {
    return data[index(ix, iy, iz)];
  }

  bool isValid() const {
    return data.size() == xCoords.size() * yCoords.size() * zCoords.size() &&
           !xCoords.empty() && !yCoords.empty() && !zCoords.empty();
  }
  std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingBox() const {
    if (!isValid()) {
      throw std::runtime_error("Cannot compute bounding box of invalid grid");
    }
    auto [xMin, xMax] = std::minmax_element(xCoords.begin(), xCoords.end());
    auto [yMin, yMax] = std::minmax_element(yCoords.begin(), yCoords.end());
    auto [zMin, zMax] = std::minmax_element(zCoords.begin(), zCoords.end());
    auto minVec = Eigen::Vector3d(*xMin, *yMin, *zMin);
    auto maxVec = Eigen::Vector3d(*xMax, *yMax, *zMax);
    return std::pair(minVec, maxVec);
  }

  /** @brief Checks if this grid is completely inside other grid
   */
  bool checkInside(const Grid<kGrid3D, T> &other) const {
    auto [minOther, maxOther] = other.boundingBox();
    auto [minThis, maxThis] = boundingBox();

    const T epsilon = std::numeric_limits<T>::epsilon() * 100;

    bool isMinValid = minOther.isApprox(minThis, epsilon) ||
                      (minOther.array() <= minThis.array()).all();
    bool isMaxValid = maxOther.isApprox(maxThis, epsilon) ||
                      (maxOther.array() >= maxThis.array()).all();

    return isMinValid && isMaxValid;
  }

private:
  void validateInitialization() const {
    if (xCoords.empty() || yCoords.empty() || zCoords.empty()) {
      throw std::runtime_error("Grid cannot have empty coordinate vectors");
    }
    if (data.size() != xCoords.size() * yCoords.size() * zCoords.size()) {
      throw std::invalid_argument("Grid data size mismatch");
    }
  }

  void boundsCheck(size_t ix, size_t iy, size_t iz) const {
    if (ix >= nx() || iy >= ny() || iz >= nz()) {
      std::stringstream msg;
      msg << "Grid index out of bounds: (" << ix << ", " << iy << ", " << iz
          << ") for grid (" << nx() << ", " << ny() << ", " << nz() << ")";
      throw std::out_of_range(msg.str());
    }
  }
};

// Type aliases for convenience
template <typename T> using Grid2D = Grid<kGrid2D, T>;

template <typename T> using Grid3D = Grid<kGrid3D, T>;

inline void munkProfile(Grid3D<double> &grid, double sofarDepth,
                        double sofarSpeed, bool isKm) {
  const double kmScaler = isKm ? 1000.0 : 1.0;
  sofarDepth *= kmScaler;
  const double kMunk = 1.0 / 1300.0;
  const double kEpsilon = 0.00737;
  for (size_t ix = 0; ix < grid.xCoords.size(); ++ix) {
    for (size_t iy = 0; iy < grid.yCoords.size(); ++iy) {
      for (size_t iz = 0; iz < grid.zCoords.size(); ++iz) {
        double zVal = grid.zCoords[iz] * kmScaler;
        double zBar = 2 * (zVal - 1300.0) * kMunk;
        grid.at(ix, iy, iz) =
            sofarSpeed * (1.0 + kEpsilon * (zBar - 1 + std::exp(-zBar)));
      }
    }
  }
};

} // namespace acoustics