//
// Created by tko on 1/29/26.
//

#include "SspBuilder.h"

namespace acoustics {

SspBuilder::SspBuilder(bhc::bhcParams<true> &params) : params_(params) {}

void SspBuilder::setupHexahedral(int nx, int ny, int nz) {
  bhc::extsetup_ssp_hexahedral(params_, nx, ny, nz);
  initialized_ = true;
  params_.ssp->Nx = nx;
  params_.ssp->Ny = ny;
  params_.ssp->Nz = nz;
  params_.ssp->NPts = nz;
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
  return result_;
}

void SspBuilder::setRangeUnits(bool inKm) { params_.ssp->rangeInKm = inKm; }

void SspBuilder::syncBoundaryDepths() {
  // Critical synchronization: boundary depths must match SSP depth range
  params_.Bdry->Top.hs.Depth = params_.ssp->Seg.z[0];
  params_.Bdry->Bot.hs.Depth = params_.ssp->Seg.z[params_.ssp->NPts - 1];
}

void SspBuilder::markDirty() { params_.ssp->dirty = true; }

Result SspBuilder::validate() const {
  Result result;

  // TODO: Implement validation logic
  // - Check consistent grid dimensions (Nx, Ny, Nz, NPts)
  // - Check monotonic depth ordering (z coordinates)
  // - Check sound speeds in realistic range (1400-1600 m/s typically)
  // - Warn if SSP grid coverage doesn't match boundary extents
  // - Check that cMat has correct size (Nx*Ny*Nz)

  return result;
}

} // namespace acoustics
