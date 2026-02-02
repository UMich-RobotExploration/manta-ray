//
// Created by tko on 2/2/26.
//

#include "SimulationBuilder.h"

namespace acoustics {
SimulationBuilder::SimulationBuilder(bhc::bhcParams<true> &params,
                                     const BathymetryConfig &bathConfig,
                                     const SSPConfig &sspConfig,
                                     AgentsConfig &agentsConfig)
    : params_(params),
      bathymetryConfig_(bathConfig),
      sspConfig_(sspConfig),
      agentsConfig_(agentsConfig) {};

void SimulationBuilder::autogenerateAltimetry() {
  const bhc::IORI2<true> grid = {kNumAltimetryPts, kNumAltimetryPts};
  bhc::extsetup_altimetry(params_, grid);
  params_.bdinfo->top.dirty = true;
  flatAltimetery3D(params_.bdinfo->top, bathymetryConfig_);
};

void SimulationBuilder::buildBathymetry() {
  const bhc::IORI2<true> grid = {bathymetryConfig_.Grid.nx(),
                                 bathymetryConfig_.Grid.ny()};
  bhc::extsetup_bathymetry(params_, grid, kNumProvince);
  bhc::BdryInfoTopBot<true> &boundary = params_.bdinfo->bot;
  boundary.dirty = true;
  boundary.dirty = bathymetryConfig_.isKm;
  const double kmScaler = bathymetryConfig_.isKm ? 1 / 1000.0 : 1.0;

  boundary.NPts[0] = static_cast<int>(bathymetryConfig_.Grid.nx());
  boundary.NPts[1] = static_cast<int>(bathymetryConfig_.Grid.ny());

  for (size_t iy = 0; iy < bathymetryConfig_.Grid.ny(); ++iy) {
    for (size_t ix = 0; ix < bathymetryConfig_.Grid.ny(); ++ix) {
      auto idx = bathymetryConfig_.Grid.index(ix, iy);
      boundary.bd[idx].x.x = bathymetryConfig_.Grid.xCoords[ix];
      boundary.bd[idx].x.y = bathymetryConfig_.Grid.yCoords[iy];
      boundary.bd[idx].x.z = bathymetryConfig_.Grid.data[idx] * kmScaler;
      // PROVINCE IS 1 INDEXED
      boundary.bd[idx].Province = 1;
    }
  }
  bathymetryBuilt_ = true;
};
void SimulationBuilder::buildSSP() {
  const Grid3D<double> &grid = sspConfig_.Grid;
  bhc::extsetup_ssp_hexahedral(params_, static_cast<int>(grid.nx()),
                               static_cast<int>(grid.ny()),
                               static_cast<int>(grid.nz()));
  params_.ssp->dirty = true;
  params_.ssp->Nx = static_cast<int>(grid.nx());
  params_.ssp->Ny = static_cast<int>(grid.ny());
  params_.ssp->Nz = static_cast<int>(grid.nz());
  params_.ssp->NPts = static_cast<int>(grid.nz());
  params_.ssp->rangeInKm = sspConfig_.isKm;

  const double kmScaler = bathymetryConfig_.isKm ? 1 / 1000.0 : 1.0;

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
        params_.ssp->c[idx] = grid.data[idx];
      }
    }
  }
}

/**
 * @brief Synchronize boundary depth values with SSP depth range
 * @detail The library requires that after writing the SSP, the boundary depths
 * are set to match the SSP depth range:
 * params.Bdry->Top.hs.Depth = ssp->Seg.z[0] and
 * params.Bdry->Bot.hs.Depth = ssp->Seg.z[ssp->NPts-1]
 */
void SimulationBuilder::syncBoundaryAndSSP() {
  params_.Bdry->Top.hs.Depth = params_.ssp->Seg.z[0];
  params_.Bdry->Bot.hs.Depth = params_.ssp->Seg.z[sspConfig_.Grid.nz() - 1];
}

void SimulationBuilder::updateAgents() {
  if (!agentsBuilt_) {
    throw std::runtime_error(
        "Cannot update agents: Agents have not been built yet.");
  }
  size_t nReceivers = agentsConfig_.receivers.size();
  bool isReceiverCountChanged =
      params_.Pos->NRr != static_cast<int32_t>(nReceivers);
  if (isReceiverCountChanged) {
    bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(nReceivers));
    bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(nReceivers));
    bhc::extsetup_rcvrdepths(params_, static_cast<int32_t>(nReceivers));
    params_.Pos->NRr = static_cast<int32_t>(nReceivers);
    params_.Pos->NRz = static_cast<int32_t>(nReceivers);
    params_.Pos->Ntheta = static_cast<int32_t>(nReceivers);
  }

  // no smart checking, everything is overwritten
  params_.Pos->RrInKm = agentsConfig_.isKm;
  const double kmScaler = agentsConfig_.isKm ? 1 / 1000.0 : 1.0;
  params_.Pos->Sx[0] = agentsConfig_.source(0);
  params_.Pos->Sy[0] = agentsConfig_.source(1);
  params_.Pos->Sz[0] = agentsConfig_.source(2) * kmScaler;

  for (size_t i = 0; i < nReceivers; ++i) {
    double delta_x = agentsConfig_.receivers[i](0) - agentsConfig_.source(0);
    double delta_y = agentsConfig_.receivers[i](1) - agentsConfig_.source(1);
    params_.Pos->theta[i] = std::atan2(delta_y, delta_x);
    params_.Pos->Rr[i] = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    params_.Pos->Rz[i] = agentsConfig_.receivers[i](2) * kmScaler;
  }
};

void SimulationBuilder::buildAgents() {
  // Setup Sources but do not assign yet (assigned in update)
  bhc::extsetup_sxsy(params_, kNumSources, kNumSources);
  bhc::extsetup_sz(params_, kNumSources);
  params_.Pos->SxSyInKm = agentsConfig_.isKm;

  // Receivers
  size_t nReceivers = agentsConfig_.receivers.size();
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

void SimulationBuilder::build() {
  buildBathymetry();
  autogenerateAltimetry();
  if (bathymetryBuilt_) {
    buildSSP();
    syncBoundaryAndSSP();
  } else {
    throw std::runtime_error(
        "Cannot build simulation: Bathymetry must be built before SSP.");
  }
  buildAgents();
};

void SimulationBuilder::flatAltimetery3D(bhc::BdryInfoTopBot<true> &boundary,
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

  for (size_t iy = 0; iy < yVals.size(); ++iy) {
    for (size_t ix = 0; ix < xVals.size(); ++ix) {
      boundary.bd[ix * kNumAltimetryPts + iy].x.x = xVals[ix];
      boundary.bd[ix * kNumAltimetryPts + iy].x.y = yVals[iy];
      boundary.bd[ix * kNumAltimetryPts + iy].x.z = 0;
    }
  }
};

} // namespace acoustics