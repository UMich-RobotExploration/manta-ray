//
// Created by tko on 1/29/26.
//

#include "SspBuilder.h"

#include <algorithm>

namespace acoustics {

SspBuilder::SspBuilder(bhc::bhcParams<true> &params) : params_(params) {}

void SspBuilder::setupHexahedral(int nx, int ny, int nz, bool inKm) {
  bhc::extsetup_ssp_hexahedral(params_, nx, ny, nz);
  initialized_ = true;
  params_.ssp->Nx = nx;
  params_.ssp->Ny = ny;
  params_.ssp->Nz = nz;
  params_.ssp->NPts = nz;
  setRangeUnits(inKm);
}

Result SspBuilder::setCoordinateGrid(const std::vector<double> &xCoords,
                                     const std::vector<double> &yCoords,
                                     const std::vector<double> &zCoords) {
  if (!initialized_) {
    result_.addError(ErrorCode::UninitializedBellhop,
                     "Need to setupHexahedral before adding grid.");
    return result_;
  }
  // TODO: Add bounds checking
  if (xCoords.size() != static_cast<size_t>(params_.ssp->Nx) ||
      yCoords.size() != static_cast<size_t>(params_.ssp->Ny) ||
      zCoords.size() != static_cast<size_t>(params_.ssp->Nz)) {
    result_.addError(ErrorCode::MismatchedDimensions,
                     "Coordinate grid sizes do not match SSP "
                     "initialization dimensions.");
    return result_;
  }
  for (size_t i = 0; i < xCoords.size(); ++i) {
    params_.ssp->Seg.x[i] = xCoords[i];
  }
  for (size_t i = 0; i < yCoords.size(); ++i) {
    params_.ssp->Seg.y[i] = yCoords[i];
  }
  for (size_t i = 0; i < zCoords.size(); ++i) {
    params_.ssp->Seg.z[i] = zCoords[i];
    params_.ssp->z[i] = zCoords[i];
  }
  markDirty();
  result_.merge(syncBoundaryDepths(result_));
  return result_;
}

void SspBuilder::setRangeUnits(bool inKm) { params_.ssp->rangeInKm = inKm; }

Result SspBuilder::syncBoundaryDepths(Result &result) {
  // Library explanation
  //  After writing your SSP, make sure params.Bdry->Top.hs.Depth (nominal
  //  surface depth, normally zero) is equal to ssp->Seg.z[0], and
  //  params.Bdry->Bot.hs.Depth (nominal ocean bottom depth) is equal to
  //  ssp->Seg.z[Nz-1].

  if (!initialized_) {
    result.addError(ErrorCode::UninitializedBellhop,
                    "Need to setupHexahedral before syncing boundary depths.");
    return result_;
  }
  if (params_.Bdry == nullptr) {
    result.addError(ErrorCode::UninitializedBellhop,
                    "Boundary information is not initialized in params.");
    return result;
  }
  params_.Bdry->Top.hs.Depth = params_.ssp->Seg.z[0];
  params_.Bdry->Bot.hs.Depth = params_.ssp->Seg.z[params_.ssp->NPts - 1];
  return result;
}

void SspBuilder::markDirty() { params_.ssp->dirty = true; }

void SspBuilder::setConstantSsp(double speed) {
  auto nX = params_.ssp->Nx;
  auto nY = params_.ssp->Ny;
  auto nZ = params_.ssp->Nz;
  for (auto i = 0; i < nX; ++i) {
    for (auto j = 0; j < nY; ++j) {
      for (auto k = 0; k < nZ; ++k) {
        params_.ssp->cMat[(i * nY + j) * nZ + k] = speed;
      }
    }
  }
  markDirty();
}

void checkSsp(bhc::SSPStructure const *ssp, Result &result) {
  if (!ssp) {
    result.addError(ErrorCode::UninitializedBellhop,
                    "Received SSP Null pointer in checkSsp.");
    return;
  }
  auto nX = ssp->Nx;
  auto nY = ssp->Ny;
  auto nZ = ssp->Nz;
  for (auto i = 0; i < nX; ++i) {
    for (auto j = 0; j < nY; ++j) {
      for (auto k = 0; k < nZ; ++k) {
        auto &speed = ssp->cMat[(i * nY + j) * nZ + k];
        if (speed > 1600.0 || speed < 1400.0) {
          if (std::count(result.warningCodes.begin(), result.warningCodes.end(),
                         WarningCode::SSPOutOfBounds) >= 1) {
            break;
          }
          result.addWarning(
              WarningCode::SSPOutOfBounds,
              "Sound speed out of typical range (1400-1600 m/s).");
        }
      }
    }
  }
  return;
}

Result SspBuilder::validate() {
  // TODO: Identify if more checks are needed here
  checkSsp(params_.ssp, result_);
  return result_;
}

} // namespace acoustics
