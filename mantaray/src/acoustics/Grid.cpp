//
// Created by tko on 2/6/26.
//

#include "acoustics/pch.h"

#include "acoustics/Grid.h"

namespace acoustics {


// ============================================================================
// Grid2D Implementations
// ============================================================================

Grid2D::Grid2D(std::vector<double> x, std::vector<double> y,
               double defaultValue)
    : xCoords(std::move(x)),
      yCoords(std::move(y)),
      data(xCoords.size() * yCoords.size(), defaultValue) {
  validateInitialization();
}

Grid2D::Grid2D(std::vector<double> x, std::vector<double> y,
               std::vector<double> initData)
    : xCoords(std::move(x)), yCoords(std::move(y)), data(std::move(initData)) {
  validateInitialization();
}

void Grid2D::clear() {
  xCoords.clear();
  yCoords.clear();
  data.clear();
}

size_t Grid2D::nx() const { return xCoords.size(); }

size_t Grid2D::ny() const { return yCoords.size(); }

size_t Grid2D::size() const { return data.size(); }

size_t Grid2D::index(size_t ix, size_t iy) const {
  return ix * yCoords.size() + iy;
}

double &Grid2D::at(size_t ix, size_t iy) {
  boundsCheck(ix, iy);
  return data[index(ix, iy)];
}

const double &Grid2D::at(size_t ix, size_t iy) const {
  boundsCheck(ix, iy);
  return data[index(ix, iy)];
}

double &Grid2D::operator()(size_t ix, size_t iy) { return data[index(ix, iy)]; }

const double &Grid2D::operator()(size_t ix, size_t iy) const {
  return data[index(ix, iy)];
}

bool Grid2D::isValid() const {
  return data.size() == xCoords.size() * yCoords.size() && !xCoords.empty() &&
         !yCoords.empty();
}

void Grid2D::validateInitialization() const {
  if (xCoords.empty() || yCoords.empty()) {
    throw std::runtime_error("Grid cannot have empty coordinate vectors");
  }
  if (data.size() != xCoords.size() * yCoords.size()) {
    throw std::invalid_argument("Grid data size mismatch");
  }
  if (utils::isMonotonicallyIncreasing(xCoords) == false) {
    throw std::invalid_argument(
        "x coordinates must be monotonically increasing");
  }
  if (utils::isMonotonicallyIncreasing(yCoords) == false) {
    throw std::invalid_argument(
        "y coordinates must be monotonically increasing");
  }
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Grid2D::boundingBox() const {
  if (!isValid()) {
    throw std::runtime_error("Cannot compute bounding box of invalid grid");
  }
  // grids are monotonically increasing
  auto xMin = xCoords.front();
  auto xMax = xCoords.back();
  auto yMin = yCoords.front();
  auto yMax = yCoords.back();
  auto minVec = Eigen::Vector2d(xMin, yMin);
  auto maxVec = Eigen::Vector2d(xMax, yMax);
  return {minVec, maxVec};
}

bool Grid2D::checkInside(const Grid3D &other) const {
  auto [minOther, maxOther] = other.boundingBox();
  auto [minThis, maxThis] = boundingBox();

  // Extract only x and y components for 2D comparison
  auto minOther2D = minOther.head<2>();
  auto maxOther2D = maxOther.head<2>();

  bool isMinValid = utils::eigenFloatSafeComparison(
      minOther2D, minThis, [](const auto &a, const auto &b) {
        return (a.array() <= b.array()).all();
      });
  bool isMaxValid = utils::eigenFloatSafeComparison(
      maxOther2D, maxThis, [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });

  return isMinValid && isMaxValid;
}

bool Grid2D::checkContain(const Grid3D &other) const {
  auto [minOther, maxOther] = other.boundingBox();
  auto [minThis, maxThis] = boundingBox();

  // Extract only x and y components for 2D comparison
  auto minOther2D = minOther.head<2>();
  auto maxOther2D = maxOther.head<2>();

  bool isMinValid = utils::eigenFloatSafeComparison(
      minOther2D, minThis, [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });
  bool isMaxValid = utils::eigenFloatSafeComparison(
      maxOther2D, maxThis, [](const auto &a, const auto &b) {
        return (a.array() <= b.array()).all();
      });

  return isMinValid && isMaxValid;
}

void Grid2D::boundsCheck(size_t ix, size_t iy) const {
  if (ix >= nx() || iy >= ny()) {
    std::stringstream msg;
    msg << "Grid index out of bounds: (" << ix << ", " << iy << ") for grid ("
        << nx() << ", " << ny() << ")";
    throw std::out_of_range(msg.str());
  }
}

// ============================================================================
// Grid3D Implementations
// ============================================================================

Grid3D::Grid3D(std::vector<double> x, std::vector<double> y,
               std::vector<double> z, double defaultValue)
    : xCoords(std::move(x)),
      yCoords(std::move(y)),
      zCoords(std::move(z)),
      data(xCoords.size() * yCoords.size() * zCoords.size(), defaultValue) {
  validateInitialization();
}

Grid3D::Grid3D(std::vector<double> x, std::vector<double> y,
               std::vector<double> z, std::vector<double> initData)
    : xCoords(std::move(x)),
      yCoords(std::move(y)),
      zCoords(std::move(z)),
      data(std::move(initData)) {
  validateInitialization();
}

void Grid3D::clear() {
  xCoords.clear();
  yCoords.clear();
  zCoords.clear();
  data.clear();
}

size_t Grid3D::nx() const { return xCoords.size(); }

size_t Grid3D::ny() const { return yCoords.size(); }

size_t Grid3D::nz() const { return zCoords.size(); }

size_t Grid3D::size() const { return data.size(); }

size_t Grid3D::index(size_t ix, size_t iy, size_t iz) const {
  return (ix * yCoords.size() + iy) * zCoords.size() + iz;
}

double &Grid3D::at(size_t ix, size_t iy, size_t iz) {
  boundsCheck(ix, iy, iz);
  return data[index(ix, iy, iz)];
}

const double &Grid3D::at(size_t ix, size_t iy, size_t iz) const {
  boundsCheck(ix, iy, iz);
  return data[index(ix, iy, iz)];
}

double &Grid3D::operator()(size_t ix, size_t iy, size_t iz) {
  return data[index(ix, iy, iz)];
}

const double &Grid3D::operator()(size_t ix, size_t iy, size_t iz) const {
  return data[index(ix, iy, iz)];
}

bool Grid3D::isValid() const {
  return data.size() == xCoords.size() * yCoords.size() * zCoords.size() &&
         !xCoords.empty() && !yCoords.empty() && !zCoords.empty();
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> Grid3D::boundingBox() const {
  if (!isValid()) {
    throw std::runtime_error("Cannot compute bounding box of invalid grid");
  }
  // grids are monotonically increasing
  auto xMin = xCoords.front();
  auto xMax = xCoords.back();
  auto yMin = yCoords.front();
  auto yMax = yCoords.back();
  auto zMin = zCoords.front();
  auto zMax = zCoords.back();
  auto minVec = Eigen::Vector3d(xMin, yMin, zMin);
  auto maxVec = Eigen::Vector3d(xMax, yMax, zMax);
  return {minVec, maxVec};
}

bool Grid3D::checkInside(const Grid3D &other) const {
  auto [minOther, maxOther] = other.boundingBox();
  auto [minThis, maxThis] = boundingBox();

  bool isMinValid = utils::eigenFloatSafeComparison(
      minOther, minThis, [](const auto &a, const auto &b) {
        return (a.array() <= b.array()).all();
      });
  bool isMaxValid = utils::eigenFloatSafeComparison(
      maxOther, maxThis, [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });

  return isMinValid && isMaxValid;
}

void Grid3D::validateInitialization() const {
  if (xCoords.empty() || yCoords.empty() || zCoords.empty()) {
    throw std::runtime_error("Grid cannot have empty coordinate vectors");
  }
  if (data.size() != xCoords.size() * yCoords.size() * zCoords.size()) {
    throw std::invalid_argument("Grid data size mismatch");
  }
  if (utils::isMonotonicallyIncreasing(xCoords) == false) {
    throw std::invalid_argument(
        "x coordinates must be monotonically increasing");
  }
  if (utils::isMonotonicallyIncreasing(yCoords) == false) {
    throw std::invalid_argument(
        "y coordinates must be monotonically increasing");
  }
  if (utils::isMonotonicallyIncreasing(zCoords) == false) {
    throw std::invalid_argument(
        "z coordinates must be monotonically increasing");
  }
  return;
}

  void Grid3D::boundsCheck(size_t ix, size_t iy, size_t iz) const {
    if (ix >= nx() || iy >= ny() || iz >= nz()) {
      std::stringstream msg;
      msg << "Grid index out of bounds: (" << ix << ", " << iy << ", " << iz
          << ") for grid (" << nx() << ", " << ny() << ", " << nz() << ")";
      throw std::out_of_range(msg.str());
    }
  }

  // ============================================================================
  // Utility Functions
  // ============================================================================

  void munkProfile(Grid3D & grid, double sofarSpeed, bool isKm) {
    const double kmScaler = isKm ? 1000.0 : 1.0;
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
  }

} // namespace acoustics