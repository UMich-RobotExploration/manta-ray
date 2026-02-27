#include "acoustics/pch.h"

#include "acoustics/Grid.h"

#include <manif/functions.h>

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

void Grid2D::validateInitialization() const {
  const std::vector<const std::vector<double> *> coordPtr = {&xCoords,
                                                             &yCoords};
  gridCheckViaPtr(coordPtr, data);
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Grid2D::boundingBox() const {
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
    SPDLOG_ERROR(
        "Grid index out of bounds: ({}, {}) for grid with size ({},{})", ix, iy,
        nx(), ny());
    throw std::out_of_range("Out of bounds attempt on grid");
  }
}

double Grid2D::interpolateDataValue(double x, double y) const {
  // Grids are monotonic, so no need to sort!
  // Want upper for strictly < and not <=.
  auto xLower = std::upper_bound(xCoords.begin(), xCoords.end(), x);
  auto yLower = std::upper_bound(yCoords.begin(), yCoords.end(), y);
  if (xLower == xCoords.end() || yLower == yCoords.end()) {
    // TODO: Would be good to have a function that does check this before
    // interpolation
    SPDLOG_ERROR("Before asking to interpolate, verify result in bounds. Check "
                 "units on input vs Grid data.");
    throw std::runtime_error("Requesting interpolation outside of grid");
  }
  size_t xUpperIdx = std::distance(xCoords.begin(), xLower);
  size_t yUpperIdx = std::distance(yCoords.begin(), yLower);

  // Using safe access methods to prevent segfaults and UB
  // Syntax reference:
  // https://en.wikipedia.org/wiki/Bilinear_interpolation

  size_t xLowerIdx = xUpperIdx - 1;
  size_t yLowerIdx = yUpperIdx - 1;
  double q11 = at(xLowerIdx, yLowerIdx);
  double q21 = at(xUpperIdx, yLowerIdx);
  double q12 = at(xLowerIdx, yUpperIdx);
  double q22 = at(xUpperIdx, yUpperIdx);

  double deltaX = xCoords.at(xUpperIdx) - xCoords.at(xLowerIdx);
  double deltaY = yCoords.at(yUpperIdx) - yCoords.at(yLowerIdx);

  double delta2 = xCoords.at(xUpperIdx) - x;
  double delta1 = x - xCoords.at(xLowerIdx);
  double aTerm = delta2 / deltaX;
  double bTerm = delta1 / deltaX;
  // breaking camel case convention for clearer math syntax
  // clang-format off
  double f_x_y1 = aTerm * q11 + bTerm * q21;
  double f_x_y2 = aTerm * q12 + bTerm * q22;
  // clang-format on
  // Reusing variables now for yCoords
  delta2 = yCoords.at(yUpperIdx) - y;
  delta1 = y - yCoords.at(yLowerIdx);
  aTerm = delta2 / deltaY;
  bTerm = delta1 / deltaY;

  return aTerm * f_x_y1 + bTerm * f_x_y2;
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

std::pair<Eigen::Vector3d, Eigen::Vector3d> Grid3D::boundingBox() const {
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
  std::vector<const std::vector<double> *> coords = {&xCoords, &yCoords,
                                                     &zCoords};
  gridCheckViaPtr(coords, data);

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
// GridVec Implementation
// ============================================================================

GridVec::GridVec(std::vector<double> x, std::vector<double> y,
                 std::vector<double> z, std::vector<Eigen::Vector2d> initData)
    : xCoords(std::move(x)),
      yCoords(std::move(y)),
      zCoords(std::move(z)),
      dataVec(std::move(initData)) {
  validateInitialization();
}

void GridVec::validateInitialization() const {
  const std::vector<const std::vector<double> *> coords = {&xCoords, &yCoords,
                                                           &zCoords};
  // need to check both datasets
  gridCheckViaPtr(coords, dataVec);
}

size_t GridVec::index(size_t ix, size_t iy, size_t iz) const {
  return (ix * yCoords.size() + iy) * zCoords.size() + iz;
}

size_t GridVec::nx() const { return xCoords.size(); }

size_t GridVec::ny() const { return yCoords.size(); }

size_t GridVec::nz() const { return zCoords.size(); }

size_t GridVec::size() const { return dataVec.size(); }

void GridVec::boundsCheck(size_t ix, size_t iy, size_t iz) const {
  if (ix >= nx() || iy >= ny() || iz >= nz()) {
    auto msg = fmt::format(
        "Grid index out of bounds: ({}, {}, {}) for grid ({}, {}, {})", ix, iy,
        iz, nx(), ny(), nz());
    throw std::out_of_range(msg);
  }
}

Eigen::Vector2d &GridVec::at(size_t ix, size_t iy, size_t iz) {
  boundsCheck(ix, iy, iz);
  auto idx = index(ix, iy, iz);
  return dataVec[idx];
}

Eigen::Vector2d &GridVec::operator()(size_t ix, size_t iy, size_t iz) {
  return dataVec[index(ix, iy, iz)];
}
const Eigen::Vector2d &GridVec::operator()(size_t ix, size_t iy,
                                           size_t iz) const {
  return dataVec[index(ix, iy, iz)];
}
Eigen::Vector3d GridVec::interpolateDataValue(double x, double y) const {}

// ============================================================================
// Utility Functions
// ============================================================================

void munkProfile(Grid3D &grid, double sofarSpeed, bool isKm) {
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
void gridCheckViaPtr(const std::vector<const std::vector<double> *> &coords,
                     const std::vector<double> &data) {
  size_t combinedSize = 1;
  for (size_t i = 0; i < coords.size(); ++i) {
    std::string coordName;
    if (i == 0) {
      coordName = "x";
    } else if (i == 1) {
      coordName = "y";
    } else if (i == 2) {
      coordName = "z";
    } else
      coordName = "unknown name";
    if (coords[i]->empty()) {
      throw std::runtime_error(coordName +
                               ": Grid cannot have empty coordinate vectors");
    }
    // accumulating dimensions
    combinedSize *= coords[i]->size();
    if (utils::isMonotonicallyIncreasing(*(coords[i])) == false) {
      throw std::invalid_argument(
          coordName + " coordinates must be monotonically increasing");
    }
  }
  if (data.size() != combinedSize) {
    throw std::invalid_argument("Grid data size mismatch");
  }
}

} // namespace acoustics
