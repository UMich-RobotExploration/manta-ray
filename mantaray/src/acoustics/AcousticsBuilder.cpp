//
// Created by tko on 2/2/26.
//

#include "pch.h"

#include "AcousticsBuilder.h"

namespace acoustics {
AcousticsBuilder::AcousticsBuilder(bhc::bhcParams<true> &params,
                                   BathymetryConfig &bathConfig,
                                   SSPConfig &sspConfig,
                                   AgentsConfig &agentsConfig)
    : params_(params),
      bathymetryConfig_(std::move(bathConfig)),
      sspConfig_(std::move(sspConfig)),
      agentsConfig_(std::move(agentsConfig)) {};

AgentsConfig &AcousticsBuilder::getAgentsConfig() { return agentsConfig_; };
const SSPConfig &AcousticsBuilder::getSSPConfig() { return sspConfig_; };

void AcousticsBuilder::autogenerateAltimetry() {
  const bhc::IORI2<true> grid = {kNumAltimetryPts, kNumAltimetryPts};
  bhc::extsetup_altimetry(params_, grid);
  params_.bdinfo->top.dirty = true;
  flatAltimetery3D(params_.bdinfo->top, bathymetryConfig_);
};

void AcousticsBuilder::buildBathymetry() {
  const bhc::IORI2<true> grid = {bathymetryConfig_.Grid.nx(),
                                 bathymetryConfig_.Grid.ny()};
  bhc::extsetup_bathymetry(params_, grid, kNumProvince);
  bhc::BdryInfoTopBot<true> &boundary = params_.bdinfo->bot;
  boundary.dirty = true;
  boundary.rangeInKm = bathymetryConfig_.isKm;
  const double kmScaler = bathymetryConfig_.isKm ? 1000.0 : 1.0;

  boundary.NPts[0] = static_cast<int>(bathymetryConfig_.Grid.nx());
  boundary.NPts[1] = static_cast<int>(bathymetryConfig_.Grid.ny());
  switch (bathymetryConfig_.interpolation) {
  case BathyInterpolationType::kLinear:
    CHECK(std::strlen(kBathymetryInterpLinearShort) == 2,
          "Interpolation type should be two characters");
    boundary.type[0] = kBathymetryInterpLinearShort[0];
    boundary.type[1] = kBathymetryInterpLinearShort[1];
    break;
  case BathyInterpolationType::kCurveInterp:
    CHECK(std::strlen(kBathymetryCurveInterpShort) == 2,
          "Interpolation type should be two characters");
    boundary.type[0] = kBathymetryCurveInterpShort[0];
    boundary.type[1] = kBathymetryCurveInterpShort[1];
    break;
  default:
    throw std::invalid_argument("Unknown bathymetry interpolation type");
  }

  for (size_t ix = 0; ix < bathymetryConfig_.Grid.nx(); ++ix) {
    for (size_t iy = 0; iy < bathymetryConfig_.Grid.ny(); ++iy) {
      auto idx = bathymetryConfig_.Grid.index(ix, iy);
      boundary.bd[idx].x.x = bathymetryConfig_.Grid.xCoords[ix];
      boundary.bd[idx].x.y = bathymetryConfig_.Grid.yCoords[iy];
      auto depth = bathymetryConfig_.Grid.data[idx] * kmScaler;
      CHECK(depth >= 0.0, "Bathymetry depth values must be non-negative.");
      boundary.bd[idx].x.z = depth;
      // PROVINCE IS 1 INDEXED
      boundary.bd[idx].Province = 1;
    }
  }
  bathymetryBuilt_ = true;
};
void AcousticsBuilder::buildSSP() {
  const Grid3D &grid = sspConfig_.Grid;
  bhc::extsetup_ssp_hexahedral(params_, static_cast<int>(grid.nx()),
                               static_cast<int>(grid.ny()),
                               static_cast<int>(grid.nz()));
  params_.ssp->dirty = true;
  params_.ssp->Nx = static_cast<int>(grid.nx());
  params_.ssp->Ny = static_cast<int>(grid.ny());
  params_.ssp->Nz = static_cast<int>(grid.nz());
  params_.ssp->NPts = static_cast<int>(grid.nz());
  params_.ssp->rangeInKm = sspConfig_.isKm;

  const double kmScaler = sspConfig_.isKm ? 1000.0 : 1.0;
  // params_.ssp->AttenUnit[0] = 'M';

  // setup coordinate grid and ssp in single nested MEGA loop
  for (size_t ix = 0; ix < grid.nx(); ++ix) {
    params_.ssp->Seg.x[ix] = grid.xCoords[ix];

    for (size_t iy = 0; iy < grid.ny(); ++iy) {
      params_.ssp->Seg.y[iy] = grid.yCoords[iy];

      for (size_t iz = 0; iz < grid.nz(); ++iz) {
        size_t idx = grid.index(ix, iy, iz);
        double scaledZ = grid.zCoords[iz] * kmScaler;
        params_.ssp->Seg.z[iz] = scaledZ;
        params_.ssp->z[iz] = scaledZ;
        params_.ssp->cMat[idx] = grid.data[idx];
        CHECK((params_.ssp->cMat[idx] >= 1400.0) &&
                  (params_.ssp->cMat[idx] <= 1600.0),
              "Unrealistic sound speed profile input into grid.");
      }
    }
  }
}

void AcousticsBuilder::syncBoundaryAndSSP() {
  params_.Bdry->Top.hs.Depth = params_.ssp->Seg.z[0];
  params_.Bdry->Bot.hs.Depth = params_.ssp->Seg.z[sspConfig_.Grid.nz() - 1];
}

void AcousticsBuilder::constructBeam(double bearingAngle) {
  params_.Angles->alpha.inDegrees = false;
  params_.Angles->beta.inDegrees = false;
  auto delta = agentsConfig_.receiver - agentsConfig_.source;
  double elevationAngle = std::atan2(delta(2), delta(1));
  if (!beamBuilt_) {
    bhc::extsetup_raybearings(params_, kNumBeams);
    bhc::extsetup_rayelevations(params_, kNumBeams);
    beamBuilt_ = true;
  }
  utils::unsafeSetupVector(params_.Angles->beta.angles,
                           bearingAngle - kBeamSpreadRadians,
                           bearingAngle + kBeamSpreadRadians, kNumBeams);
  utils::unsafeSetupVector(params_.Angles->alpha.angles,
                           elevationAngle - kBeamSpreadRadians,
                           elevationAngle + kBeamSpreadRadians, kNumBeams);

  auto beam = params_.Beam;
  double boxScale = 1.10;
  beam->rangeInKm = agentsConfig_.isKm;
  double kmScaler = bathymetryConfig_.isKm ? 1000.0 : 1.0;
  beam->deltas = delta.norm() * kBeamStepSizeRatio;
  double deltaX = std::abs(delta(0));
  double deltaY = std::abs(delta(1));
  CHECK((deltaX > 0.0) && (deltaY > 0.0),
        "Delta's need to be positive in bellhop box");
  beam->Box.x = deltaX * boxScale;
  beam->Box.y = deltaY * boxScale;
  // TODO: Fix this override
  beam->Box.y = 10000.0;
  double max = *std::max_element(bathymetryConfig_.Grid.data.begin(),
                                 bathymetryConfig_.Grid.data.end());
  beam->Box.z = max * kmScaler + 10;
}

void AcousticsBuilder::updateAgents() {
  if (!agentsBuilt_) {
    throw std::runtime_error(
        "Cannot update agents: Agents have not been built yet.");
  }
  bool isReceiverCountIdentical =
      params_.Pos->NRr == static_cast<int32_t>(kNumRecievers);
  if (!isReceiverCountIdentical) {
    std::cout << "Reallocating receiver arrays for updated agents.\n";
    bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(kNumRecievers));
    bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(kNumRecievers));
    bhc::extsetup_rcvrdepths(params_, static_cast<int32_t>(kNumRecievers));
    params_.Pos->NRr = static_cast<int32_t>(kNumRecievers);
    params_.Pos->NRz = static_cast<int32_t>(kNumRecievers);
    params_.Pos->Ntheta = static_cast<int32_t>(kNumRecievers);
  }

  // no smart checking, everything is overwritten
  params_.Pos->RrInKm = agentsConfig_.isKm;
  const double kmScaler = agentsConfig_.isKm ? 1000.0 : 1.0;
  params_.Pos->Sx[0] = agentsConfig_.source(0);
  params_.Pos->Sy[0] = agentsConfig_.source(1);
  params_.Pos->Sz[0] = agentsConfig_.source(2) * kmScaler;

  auto delta = agentsConfig_.receiver(Eigen::seq(0, 1)) -
               agentsConfig_.source(Eigen::seq(0, 1));
  // std::cout << "Eigen Delta: " << delta.norm() << "\n";
  double bearingAngle = std::atan2(delta(1), delta(0));
  params_.Pos->theta[0] = bearingAngle * kRadians2Degree; // degrees by bellhop!
  params_.Pos->Rr[0] = delta.norm();
  params_.Pos->Rz[0] = agentsConfig_.receiver(2) * kmScaler;

  constructBeam(bearingAngle);
};

void AcousticsBuilder::buildAgents() {
  // Setup Sources but do not assign yet (assigned in update)
  bhc::extsetup_sxsy(params_, kNumSources, kNumSources);
  bhc::extsetup_sz(params_, kNumSources);
  params_.Pos->SxSyInKm = agentsConfig_.isKm;

  // Receivers
  size_t nReceivers = agentsConfig_.receiver.size();
  bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrdepths(params_, static_cast<int32_t>(nReceivers));
  agentsBuilt_ = true;
  // Assigning receiver ranges for update check to prevent reallocation
  params_.Pos->RrInKm = agentsConfig_.isKm;
  params_.Pos->NRr = static_cast<int32_t>(nReceivers);
  params_.Pos->NRz = static_cast<int32_t>(nReceivers);
  params_.Pos->Ntheta = static_cast<int32_t>(nReceivers);
  updateAgents();
}

void AcousticsBuilder::validateSPPandBathymetryBox(
    const Grid2D &bathGrid, const Grid3D &sspGrid) const {
  bool isBathInside = bathGrid.checkInside(sspGrid);
  if (isBathInside) {
    return;
  }
  throw std::runtime_error("SSP grid must completely enclose bathymetry grid "
                           "to avoid inability to interpolate rays.");
}

void AcousticsBuilder::build() {
  buildBathymetry();
  autogenerateAltimetry();
  if (bathymetryBuilt_) {
    buildSSP();
    syncBoundaryAndSSP();
    validateSPPandBathymetryBox(bathymetryConfig_.Grid, sspConfig_.Grid);
  } else {
    // ReSharper disable once CppDFAUnreachableCode
    throw std::runtime_error(
        "Cannot build simulation: Bathymetry must be built before SSP.");
  }
  buildAgents();
};

void AcousticsBuilder::flatAltimetery3D(bhc::BdryInfoTopBot<true> &boundary,
                                        const BathymetryConfig &bathConfig) {
  boundary.dirty = true;
  boundary.rangeInKm = bathConfig.isKm;
  boundary.NPts[0] = kNumAltimetryPts;
  boundary.NPts[1] = kNumAltimetryPts;

  auto [minIt, maxIt] = std::minmax_element(bathConfig.Grid.xCoords.begin(),
                                            bathConfig.Grid.xCoords.end());
  double minX = *minIt;
  double maxX = *maxIt;

  std::tie(minIt, maxIt) = std::minmax_element(bathConfig.Grid.yCoords.begin(),
                                               bathConfig.Grid.yCoords.end());
  double minY = *minIt;
  double maxY = *maxIt;
  std::array<double, kNumAltimetryPts> xVals{minX, maxX};
  std::array<double, kNumAltimetryPts> yVals{minY, maxY};

  for (size_t ix = 0; ix < xVals.size(); ++ix) {
    for (size_t iy = 0; iy < yVals.size(); ++iy) {
      boundary.bd[ix * kNumAltimetryPts + iy].x.x = xVals[ix];
      boundary.bd[ix * kNumAltimetryPts + iy].x.y = yVals[iy];
      boundary.bd[ix * kNumAltimetryPts + iy].x.z = 0;
    }
  }
};
void AcousticsBuilder::quadraticBathymetry3D(const std::vector<double> &gridX,
                                             const std::vector<double> &gridY,
                                             std::vector<double> &data,
                                             double depth) {
  if (data.size() != gridX.size() * gridY.size()) {
    // setting to -1.0 to indicate uninitialized for easier debugging
    data.resize(gridX.size() * gridY.size(), -1.0);
  }
  // TODO: Set back to 100.0 after testing
  double scalerReduction = 1.0 / 1000.0;
  for (size_t iy = 0; iy < gridY.size(); ++iy) {
    for (size_t ix = 0; ix < gridX.size(); ++ix) {
      auto currDepth =
          depth -
          (scalerReduction * (gridX[ix] * gridX[ix] + gridY[iy] * gridY[iy]));
      CHECK(currDepth > 0, "Quadratic bathymetry should have some depth.");
      data[ix * gridY.size() + iy] = currDepth;
    }
  }
}
void AcousticsBuilder::updateReceiver(double x, double y, double z) {
  agentsConfig_.receiver(0) = x;
  agentsConfig_.receiver(1) = y;
  agentsConfig_.receiver(2) = z;
  updateAgents();
}

void AcousticsBuilder::updateSource(double x, double y, double z) {
  agentsConfig_.source(0) = x;
  agentsConfig_.source(1) = y;
  agentsConfig_.source(2) = z;
  updateAgents();
}

} // namespace acoustics