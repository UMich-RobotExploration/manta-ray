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
  markDirty(false);
}

void BoundaryBuilder::setupAltimetry(const bhc::IORI2<true> &grid) {
  bhc::extsetup_altimetry(params_, grid);
  markDirty(true);
}

void BoundaryBuilder::setFlatBottom(double depth,
                                    const std::vector<double> &gridX,
                                    const std::vector<double> &gridY) {
  flatBoundary3D(params_.bdinfo->bot, depth, gridX, gridY);
  markDirty(true);
}

void BoundaryBuilder::setQuadraticBottom(double depth,
                                         const std::vector<double> &gridX,
                                         const std::vector<double> &gridY,
                                         bool autoGenerateTop) {
  quadBoundary3D(params_.bdinfo->bot, depth, gridX, gridY);
  if (autoGenerateTop) {
    double topDepth = 0.0;
    auto [minIt, maxIt] = std::minmax_element(gridX.begin(), gridX.end());
    double minX = *minIt;
    double maxX = *maxIt;

    std::tie(minIt, maxIt) = std::minmax_element(gridY.begin(), gridY.end());
    double minY = *minIt;
    double maxY = *maxIt;
    flatBoundary3D(params_.bdinfo->top, topDepth, {minX, maxX}, {minY, maxY});
    setInterpolationType(acoustics::BathyInterpolationType::kLinear, true);
    assertBoundariesEqual();
    markDirty(true);
  }
  markDirty(false);
}

void BoundaryBuilder::setFlatTop(double depth, const std::vector<double> &gridX,
                                 const std::vector<double> &gridY) {
  flatBoundary3D(params_.bdinfo->top, depth, gridX, gridY);
  markDirty(true);
}

void BoundaryBuilder::setInterpolationType(BathyInterpolationType interpType,
                                           bool setTop) {
  bhc::BdryInfoTopBot<true> &boundary =
      setTop ? params_.bdinfo->top : params_.bdinfo->bot;

  switch (interpType) {
  case BathyInterpolationType::kLinear:
    memcpy(boundary.type, kBathymetryInterpLinearShort, kBathymetryBuffSize);
    break;
  case BathyInterpolationType::kCurveInterp:
    memcpy(boundary.type, kBathymetryCurveInterpShort, kBathymetryBuffSize);
    break;
  }
}

void BoundaryBuilder::markDirty(bool setTop) {
  if (setTop) {
    params_.bdinfo->top.dirty = true;
  } else {
    params_.bdinfo->bot.dirty = true;
  }
}

ValidationResult BoundaryBuilder::validate() const {
  ValidationResult result;

  // TODO: Implement validation logic
  // - Check grid dimensions > 0
  // - Check valid depth ranges
  // - Check grid coordinate consistency
  // - Warn if grids are very small or large
  assertBoundariesValid();

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
      auto idx = getIndex(boundary, ix, iy);
      boundary.bd[idx].x.x = gridX[ix];
      boundary.bd[idx].x.y = gridY[iy];
      boundary.bd[idx].x.z =
          depth - std::pow(gridX[ix], 2.0) - std::pow(gridY[iy], 2.0);
      // PROVINCE IS 1 INDEXED
      boundary.bd[ix * gridY.size() + iy].Province = 1;
    }
  }
}

void BoundaryBuilder::assertBoundariesValid() {
  throw std::runtime_error("Function not implemented so you are not validating");
}
void BoundaryBuilder::assertBoundariesEqual() {

  auto xBotPts = params_.bdinfo->bot.NPts[0];
  auto yBotPts = params_.bdinfo->bot.NPts[1];
  auto xTopPts = params_.bdinfo->top.NPts[0];
  auto yTopPts = params_.bdinfo->top.NPts[1];
  /// lambda function to assert X and Y
  // testing the 4 corners
  assertCornerEq(0, 0, 0, 0);
  assertCornerEq(xBotPts - 1, 0, xTopPts - 1, 0);
  assertCornerEq(xBotPts - 1, yBotPts - 1, xTopPts - 1, yTopPts - 1);
  assertCornerEq(0, yBotPts - 1, 0, yTopPts - 1);
}

void BoundaryBuilder::assertCornerEq(size_t botX, size_t botY, size_t topX,
                                     size_t topY) {
  auto botIdx = getIndex(params_.bdinfo->bot, botX, botY);
  auto topIdx = getIndex(params_.bdinfo->top, topX, topY);
  assert(params_.bdinfo->bot.bd[botIdx].x.x ==
         params_.bdinfo->top.bd[topIdx].x.x);
  assert(params_.bdinfo->bot.bd[botIdx].x.y ==
         params_.bdinfo->top.bd[topIdx].x.y);
}


size_t getIndex(const bhc::BdryInfoTopBot<true> &boundary, size_t ix,
                size_t iy) {
  return ix * boundary.NPts[1] + iy;
}

} // namespace acoustics
