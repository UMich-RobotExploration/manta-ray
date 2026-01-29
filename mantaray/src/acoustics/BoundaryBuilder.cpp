//
// Created by tko on 1/29/26.
//

#include "BoundaryBuilder.h"
#include "acousticsConstants.h"
#include <cmath>
#include <cstring>

namespace acoustics {

BoundaryBuilder::BoundaryBuilder(bhc::bhcParams<true> &params)
    : params_(params) {}

void BoundaryBuilder::setupBathymetry(const bhc::IORI2<true> &grid,
                                      int32_t nProvinces) {
  bhc::extsetup_bathymetry(params_, grid, nProvinces);
}

void BoundaryBuilder::setupAltimetry(const bhc::IORI2<true> &grid) {
  bhc::extsetup_altimetry(params_, grid);
}

void BoundaryBuilder::setFlatBottom(double depth,
                                    const std::vector<double> &gridX,
                                    const std::vector<double> &gridY) {
  flatBoundary3D(params_.bdinfo->bot, depth, gridX, gridY);
}

void BoundaryBuilder::setQuadraticBottom(double depth,
                                         const std::vector<double> &gridX,
                                         const std::vector<double> &gridY) {
  quadBoundary3D(params_.bdinfo->bot, depth, gridX, gridY);
}

void BoundaryBuilder::setFlatTop(double depth, const std::vector<double> &gridX,
                                 const std::vector<double> &gridY) {
  flatBoundary3D(params_.bdinfo->top, depth, gridX, gridY);
}

void BoundaryBuilder::setBottomInterpolationType(const char *type) {
  memcpy(params_.bdinfo->bot.type, type, kBathymetryBuffSize);
}

void BoundaryBuilder::setTopInterpolationType(const char *type) {
  memcpy(params_.bdinfo->top.type, type, kBathymetryBuffSize);
}

void BoundaryBuilder::markDirty() {
  params_.bdinfo->top.dirty = true;
  params_.bdinfo->bot.dirty = true;
}

ValidationResult BoundaryBuilder::validate() const {
  ValidationResult result;

  // TODO: Implement validation logic
  // - Check grid dimensions > 0
  // - Check valid depth ranges
  // - Check grid coordinate consistency
  // - Warn if grids are very small or large

  return result;
}

void BoundaryBuilder::flatBoundary3D(bhc::BdryInfoTopBot<true> &boundary,
                                     double depth,
                                     const std::vector<double> &gridX,
                                     const std::vector<double> &gridY) {
  boundary.dirty = true;
  boundary.rangeInKm = true;
  boundary.NPts[0] = static_cast<int>(gridX.size());
  boundary.NPts[1] = static_cast<int>(gridY.size());

  for (size_t iy = 0; iy < gridY.size(); ++iy) {
    for (size_t ix = 0; ix < gridX.size(); ++ix) {
      boundary.bd[ix * gridY.size() + iy].x.x = gridX[ix];
      boundary.bd[ix * gridY.size() + iy].x.y = gridY[iy];
      boundary.bd[ix * gridY.size() + iy].x.z = depth;
    }
  }
}

void BoundaryBuilder::quadBoundary3D(bhc::BdryInfoTopBot<true> &boundary,
                                     double depth,
                                     const std::vector<double> &gridX,
                                     const std::vector<double> &gridY) {
  boundary.dirty = true;
  boundary.rangeInKm = true;
  boundary.NPts[0] = static_cast<int>(gridX.size());
  boundary.NPts[1] = static_cast<int>(gridY.size());

  for (size_t iy = 0; iy < gridY.size(); ++iy) {
    for (size_t ix = 0; ix < gridX.size(); ++ix) {
      boundary.bd[ix * gridY.size() + iy].x.x = gridX[ix];
      boundary.bd[ix * gridY.size() + iy].x.y = gridY[iy];
      boundary.bd[ix * gridY.size() + iy].x.z =
          depth - std::pow(gridX[ix], 2.0) - std::pow(gridY[iy], 2.0);
      // PROVINCE IS 1 INDEXED
      boundary.bd[ix * gridY.size() + iy].Province = 1;
    }
  }
}

} // namespace acoustics
