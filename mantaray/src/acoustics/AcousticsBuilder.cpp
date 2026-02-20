//
// Created by tko on 2/2/26.
//

#include "acoustics/pch.h"

#include "acoustics/AcousticsBuilder.h"

namespace acoustics {
AcousticsBuilder::AcousticsBuilder(bhc::bhcParams<true> &params,
                                   BathymetryConfig &bathConfig,
                                   SSPConfig &sspConfig,
                                   AgentsConfig &agentsConfig)
    : params_(params),
      bathymetryConfig_(std::move(bathConfig)),
      sspConfig_(std::move(sspConfig)),
      agentsConfig_(std::move(agentsConfig)){

      };

AgentsConfig &AcousticsBuilder::getAgentsConfig() { return agentsConfig_; };
const SSPConfig &AcousticsBuilder::getSSPConfig() const { return sspConfig_; };

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

void AcousticsBuilder::adjustBeamBox(const Eigen::Vector3d &sourcePos,
                                     const BathymetryConfig &bathymetry,
                                     double &beamX, double &beamY) {
  auto [minValue, maxValue] = bathymetry.Grid.boundingBox();
  double kmScaler = bathymetry.isKm ? 1000.0 : 1.0;
  minValue = minValue * kmScaler;
  maxValue = maxValue * kmScaler;
  // construct the 2 main corners
  auto beamBox = utils::boxFromMidpoint(sourcePos, beamX, beamY);
  bool isMaxValid = utils::eigenFloatSafeComparison(
      maxValue, beamBox.topRight, [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });
  bool isMinValid = utils::eigenFloatSafeComparison(
      beamBox.bottomLeft, minValue, [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });
  // If we are already within the bounding box, we can just skip
  if (isMaxValid && isMinValid) {
    SPDLOG_TRACE("Beam bounding box is valid and within acoustics sim");
    return;
  }
  // This means one of the conditions is evaluated as outside the range.
  // Instead of checking which, we can just compute both offsets and take the
  // bigger of the two needed.

  // Will only be negative if beam box is larger;
  //  (2,2) - (3,1) = (-1, 1)
  auto deltaMax = maxValue - beamBox.topRight;
  // Will only be negative if beam box is smaller
  //  (-2,0) - (-1,-1) = (-1, 1)
  auto deltaMin = beamBox.bottomLeft - minValue;
  double xAdjust = std::min(deltaMax(0), deltaMin(0));
  double yAdjust = std::min(deltaMax(1), deltaMin(1));
  // Making sure we only take negative adjustments
  xAdjust = std::min(xAdjust, 0.0);
  yAdjust = std::min(yAdjust, 0.0);
  SPDLOG_DEBUG("Adjusting beam X size or beam Y size based on invalid box. "
               "Shift is occuring in X: {}, Y: {}",
               xAdjust, yAdjust);
  beamX = beamX + xAdjust;
  beamY = beamY + yAdjust;
  return;
}

void AcousticsBuilder::checkReceiverInBox(const Eigen::Vector3d &sourcePos,
                                          const Eigen::Vector3d &receiverPos,
                                          double beamX, double beamY) {
  auto beamBox = utils::boxFromMidpoint(sourcePos, beamX, beamY);

  bool isMaxValid = utils::eigenFloatSafeComparison(
      beamBox.topRight, receiverPos.head(2), [](const auto &a, const auto &b) {
        return (a.array() >= b.array()).all();
      });
  bool isMinValid =
      utils::eigenFloatSafeComparison(receiverPos.head(2), beamBox.bottomLeft,
                                      [](const auto &a, const auto &b) {
                                        return (a.array() >= b.array()).all();
                                      });
  if (!isMaxValid || !isMinValid) {
    SPDLOG_ERROR("Reciever is outside the beam box, no rays will get to it.");
  }
}

void AcousticsBuilder::constructBeam(double bearingAngle) {
  params_.Angles->alpha.inDegrees = false;
  params_.Angles->beta.inDegrees = false;
  Eigen::Vector3d delta = agentsConfig_.receiver - agentsConfig_.source;
  double horizontalDistance =
      std::sqrt((delta(0) * delta(0)) + (delta(1) * delta(1)));
  double elevationAngle = std::atan2(delta(2), horizontalDistance);
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
  constexpr double boxScale = 1.50;
  beam->rangeInKm = false;
  double kmScaler = bathymetryConfig_.isKm ? 1000.0 : 1.0;
  beam->deltas = delta.norm() * kBeamStepSizeRatio;
  delta = boxScale * delta;
  double deltaX = std::abs(delta(0));
  double deltaY = std::abs(delta(1));
  CHECK((deltaX > 0.0) && (deltaY > 0.0),
        "Delta's need to be positive in bellhop box");

  // Beam box is centered around source coord sys. Reference bellhop docs
  // if needed
  // adjustBeamBox(agentsConfig_.source, bathymetryConfig_, deltaX, deltaY);
  checkReceiverInBox(agentsConfig_.source, agentsConfig_.receiver, deltaX,
                     deltaY);
  beam->Box.x = deltaX;
  beam->Box.y = deltaY;
  SPDLOG_DEBUG("Beam box set to: x: {}, y: {}", deltaX, deltaY);
  double max = *std::max_element(bathymetryConfig_.Grid.data.begin(),
                                 bathymetryConfig_.Grid.data.end());
  // Adding a 10 meter buffer to the beam box to ensure that values can rebound
  // off the bottom
  beam->Box.z = max * kmScaler + 10;
}
std::pair<double, bool>
AcousticsBuilder::isWithinBathymetry(Eigen::Vector3d &position) const {
  double kmScalerBath = bathymetryConfig_.isKm ? 1.0 / 1000.0 : 1.0;
  double bathymetryHeight =
      bathymetryConfig_.Grid.interpolateDataValue(position.x() * kmScalerBath,
                                                  position.y() * kmScalerBath) *
      1 / kmScalerBath;
  if (bathymetryHeight > position.z()) {
    return {bathymetryHeight, true};
  }
  return {bathymetryHeight, false};
}

BoundaryCheck AcousticsBuilder::updateAgents() {
  if (!agentsBuilt_) {
    throw std::runtime_error(
        "Cannot update agents: Agents have not been built yet.");
  }
  bool isReceiverCountIdentical = params_.Pos->NRr == kNumRecievers;
  if (!isReceiverCountIdentical) {
    SPDLOG_DEBUG("Reallocating receiver arrays for updated agents.\n");
    bhc::extsetup_rcvrranges(params_, kNumRecievers);
    bhc::extsetup_rcvrbearings(params_, kNumRecievers);
    bhc::extsetup_rcvrdepths(params_, kNumRecievers);
    params_.Pos->NRr = kNumRecievers;
    params_.Pos->NRz = kNumRecievers;
    params_.Pos->Ntheta = kNumRecievers;
  }

  bool isSourceInBounds =
      utils::positionInBounds(agentsConfig_.source, minCoords_, maxCoords_);
  bool isReceiver =
      utils::positionInBounds(agentsConfig_.receiver, minCoords_, maxCoords_);

  if (!isSourceInBounds || !isReceiver) {
    auto msg = fmt::format(
        "Source or Receiver position is out of bounds of the simulation box. "
        "Source: ({}) , Receiver: ({}) , Min Box: ({}) , Max Box: ({})",
        agentsConfig_.source.transpose(), agentsConfig_.receiver.transpose(),
        minCoords_.transpose(), maxCoords_.transpose());
    SPDLOG_WARN(msg);
    return BoundaryCheck::kEitherOrOutOfBounds;
  }
  auto [bathymetryHeight, isReceiverWithinBath] =
      isWithinBathymetry(agentsConfig_.receiver);
  if (!isReceiverWithinBath) {
    auto msg =
        fmt::format("Current receiver position is below the bathymetry "
                    "and therefore should be marked as dead. Receiver z-height "
                    "is {:4f}, while bathymetry interpolated value is {:4f}",
                    agentsConfig_.receiver.z(), bathymetryHeight);
    SPDLOG_WARN(msg);
    return BoundaryCheck::kReceiverOutofBounds;
  }
  auto result = isWithinBathymetry(agentsConfig_.source);
  bool isSourceWithinBath = result.second;
  bathymetryHeight = result.first;
  if (!isSourceWithinBath) {
    auto msg =
        fmt::format("Current receiver position is below the bathymetry "
                    "and therefore should be marked as dead. Receiver z-height "
                    "is {:4f}, while bathymetry interpolated value is {:4f}",
                    agentsConfig_.source.z(), bathymetryHeight);
    SPDLOG_WARN(msg);
    return BoundaryCheck::kSourceOutofBounds;
  }

  // no smart checking, everything is overwritten
  params_.Pos->RrInKm = false;
  params_.Pos->Sx[0] = agentsConfig_.source(0);
  params_.Pos->Sy[0] = agentsConfig_.source(1);
  params_.Pos->Sz[0] = utils::safeDoubleToFloat(agentsConfig_.source(2));

  auto delta = agentsConfig_.receiver(Eigen::seq(0, 1)) -
               agentsConfig_.source(Eigen::seq(0, 1));
  double bearingAngle = std::atan2(delta(1), delta(0));
  params_.Pos->theta[0] = bearingAngle * kRadians2Degree; // degrees by bellhop!
  params_.Pos->Rr[0] = delta.norm();
  params_.Pos->Rz[0] = utils::safeDoubleToFloat(agentsConfig_.receiver(2));

  constructBeam(bearingAngle);
  return BoundaryCheck::kInBounds;
};

void AcousticsBuilder::buildAgents() {
  // Setup Sources but do not assign yet (assigned in update)
  bhc::extsetup_sxsy(params_, kNumSources, kNumSources);
  bhc::extsetup_sz(params_, kNumSources);
  params_.Pos->SxSyInKm = false;

  // Receivers
  size_t nReceivers = agentsConfig_.receiver.size();
  bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrdepths(params_, static_cast<int32_t>(nReceivers));
  agentsBuilt_ = true;
  // Assigning receiver ranges for update check to prevent reallocation
  params_.Pos->RrInKm = false;
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
    // Here we are assuming bathymetry grid fits within SSP grid
    // Which is reasonable as we check this later in the build process
    double kmScalerBath = bathymetryConfig_.isKm ? 1000.0 : 1.0;
    double kmScalerSSP = sspConfig_.isKm ? 1000.0 : 1.0;
    minCoords_[0] = bathymetryConfig_.Grid.xCoords.front() * kmScalerBath;
    minCoords_[1] = bathymetryConfig_.Grid.yCoords.front() * kmScalerBath;
    minCoords_[2] = sspConfig_.Grid.zCoords.front() * kmScalerSSP;
    maxCoords_[0] = bathymetryConfig_.Grid.xCoords.back() * kmScalerBath;
    maxCoords_[1] = bathymetryConfig_.Grid.yCoords.back() * kmScalerBath;
    maxCoords_[2] = sspConfig_.Grid.zCoords.back() * kmScalerSSP;
  } else {
    // ReSharper disable once CppDFAUnreachableCode
    // This code is here even though unreachable, to protect future
    // refactorers on build order of the sim.
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

  // Grid's have guaranteed monotonic values in coords
  double minX = bathConfig.Grid.xCoords.front();
  double maxX = bathConfig.Grid.xCoords.back();

  double minY = bathConfig.Grid.yCoords.front();
  double maxY = bathConfig.Grid.yCoords.back();
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

BoundaryCheck
AcousticsBuilder::updateReceiver(const Eigen::Vector3d &position) {
  return updateReceiver(position(0), position(1), position(2));
}

BoundaryCheck AcousticsBuilder::updateReceiver(double x, double y, double z) {
  agentsConfig_.receiver(0) = x;
  agentsConfig_.receiver(1) = y;
  agentsConfig_.receiver(2) = z;
  return updateAgents();
}

BoundaryCheck AcousticsBuilder::updateSource(const Eigen::Vector3d &position) {
  return updateSource(position(0), position(1), position(2));
}

BoundaryCheck AcousticsBuilder::updateSource(double x, double y, double z) {
  agentsConfig_.source(0) = x;
  agentsConfig_.source(1) = y;
  agentsConfig_.source(2) = z;
  return updateAgents();
}

} // namespace acoustics
