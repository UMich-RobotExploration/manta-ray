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
  params_.Pos->SxSyInKm = inKm;
  if (!(x.size() == y.size() == z.size())) {
    result_.addError(ErrorCode::MismatchedDimensions,
                     "Receiver coordinate vectors must be the same size.");
    return result_;
  }
  size_t nReceivers = x.size();

  bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(nReceivers));
  for (size_t i = 0; i < nReceivers; ++i) {
    double delta_x = x[i] - params_.Pos->Sx[0];
    double delta_y = y[i] - params_.Pos->Sy[0];
    params_.Pos->theta[i] = std::atan2(delta_y, delta_x);
    params_.Pos->Rr[i] = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
  }

  // Managing detph setup. Here need to deal with bellhop defaulting to meters
  bhc::extsetup_rcvrdepths(params_, nReceivers);
  for (size_t i = 0; i < nReceivers; ++i) {
    if (inKm) {
      params_.Pos->Rz[i] = z[i] / 1000.0f;
    } else {
      params_.Pos->Rz[i] = z[i];
    }
  }

  initializer_.receiver = true;

  return result_;
}

} // namespace acoustics