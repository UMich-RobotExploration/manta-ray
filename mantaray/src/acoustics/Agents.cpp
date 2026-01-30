//
// Created by tko on 1/30/26.
//

#include "Agents.h"

namespace acoustics {

Agents::Agents(bhc::bhcParams<true> &params) : params_(params) {}

void Agents::initializeSource(double x, double y, double z, bool inKm) {
  bhc::extsetup_sxsy(params_, kNumSources, kNumSources);
  bhc::extsetup_sz(params_, kNumSources);
  params_.Pos->SxSyInKm = inKm;
  if (inKm) {
    z = z / 1000.0;
  }
  params_.Pos->Sx[0] = x;
  params_.Pos->Sy[0] = y;
  params_.Pos->Sy[0] = z;
  initializer_.source = true;
}

Result Agents::initializeReceivers(const std::vector<double> &x,
                                 const std::vector<double> &y,
                                 const std::vector<float> &z, bool inKm) {
  params_.Pos->RrInKm = inKm;
  params_.Pos->SxSyInKm= inKm;
  if (!(x.size() == y.size() == z.size())) {
    result_.addError(ErrorCode::MismatchedDimensions,
                     "Receiver coordinate vectors must be the same size.");
    return result_;
  }
  size_t nReceivers = x.size();

  bhc::extsetup_rcvrranges(params_, nReceivers);
  bhc::extsetup_rcvrbearings(params_, nReceivers);

  // Managing detph setup. Here need to deal with bellhop defaulting to meters
  bhc::extsetup_rcvrdepths(params_, nReceivers);
  setupVector(params_.Pos->Rz, z, nReceivers);


  initializer_.receiver= true;

  return result_;
}

} // namespace acoustics