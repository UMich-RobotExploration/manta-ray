//
// Created by tko on 1/29/26.
//

#include "BeamBuilder.h"

namespace acoustics {

BeamBuilder::BeamBuilder(bhc::bhcParams<true> &params) : params_(params) {}

void BeamBuilder::setupRayBearings(int nBearings, double minAngle,
                                   double maxAngle) {
  bhc::extsetup_raybearings(params_, nBearings);
  SetupVector(params_.Angles->beta.angles, minAngle, maxAngle, nBearings);
}

void BeamBuilder::setupRayElevations(int nElevations, double minAngle,
                                     double maxAngle) {
  bhc::extsetup_rayelevations(params_, nElevations);
  SetupVector(params_.Angles->alpha.angles, minAngle, maxAngle, nElevations);
}

void BeamBuilder::setBeamBox(double x, double y, double z) {
  params_.Beam->Box.x = x;
  params_.Beam->Box.y = y;
  params_.Beam->Box.z = z;
}

void BeamBuilder::setStepSize(double deltas) { params_.Beam->deltas = deltas; }

void BeamBuilder::setAngleUnits(bool inDegrees) {
  params_.Angles->alpha.inDegrees = inDegrees;
  params_.Angles->beta.inDegrees = inDegrees;
}

void BeamBuilder::setRangeUnits(bool inKm) { params_.Beam->rangeInKm = inKm; }

Result BeamBuilder::validate() const {
  Result result;

  // TODO: Implement validation logic
  // - Check positive beam counts
  // - Check valid angle ranges
  // - Check positive beam box dimensions
  // - Check positive step size
  // - Warn if beam box is very large or very small

  return result;
}

} // namespace acoustics
