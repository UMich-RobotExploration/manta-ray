//
// Created by tko on 1/29/26.
//

#include "SourceReceiverBuilder.h"

namespace acoustics {

SourceReceiverBuilder::SourceReceiverBuilder(bhc::bhcParams<true> &params)
    : params_(params) {}

void SourceReceiverBuilder::setupSourceGrid(int nSx, int nSy) {
  bhc::extsetup_sxsy(params_, nSx, nSy);
}

void SourceReceiverBuilder::setupSourceDepths(int nSz) {
  bhc::extsetup_sz(params_, nSz);
}

void SourceReceiverBuilder::setSourcePosition(float x, float y, float z,
                                              int index) {
  params_.Pos->Sx[index] = x;
  params_.Pos->Sy[index] = y;
  params_.Pos->Sz[index] = z;
}

void SourceReceiverBuilder::setupReceiverBearings(int nBearings) {
  bhc::extsetup_rcvrbearings(params_, nBearings);
}

void SourceReceiverBuilder::setupReceiverRanges(int nRanges) {
  bhc::extsetup_rcvrranges(params_, nRanges);
}

void SourceReceiverBuilder::setupReceiverDepths(int nDepths) {
  bhc::extsetup_rcvrdepths(params_, nDepths);
}

void SourceReceiverBuilder::setReceiverBearings(double minBearing,
                                                double maxBearing) {
  int nBearings = params_.Pos->Ntheta;
  SetupVector(params_.Pos->theta, minBearing, maxBearing, nBearings);
}

void SourceReceiverBuilder::setReceiverRanges(double minRange,
                                              double maxRange) {
  int nRanges = params_.Pos->NRr;
  SetupVector(params_.Pos->Rr, minRange, maxRange, nRanges);
}

void SourceReceiverBuilder::setReceiverDepth(float depth, int index) {
  params_.Pos->Rz[index] = depth;
}

void SourceReceiverBuilder::setSourceXYUnits(bool inKm) {
  params_.Pos->SxSyInKm = inKm;
}

void SourceReceiverBuilder::setReceiverRangeUnits(bool inKm) {
  params_.Pos->RrInKm = inKm;
}

ValidationResult SourceReceiverBuilder::validate() const {
  ValidationResult result;

  // TODO: Implement validation logic
  // - Check positive position counts
  // - Check positions within physical bounds
  // - Warn if source/receiver positions are outside domain
  // - Check that positions are set for all allocated indices

  return result;
}

} // namespace acoustics
