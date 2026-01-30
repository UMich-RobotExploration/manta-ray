//
// Created by tko on 1/30/26.
//

#include "Agents.h"

namespace acoustics {

Agents::Agents(bhc::bhcParams<true> &params) : params_(params) {}

void Agents::updateSource(double x, double y, double z, Result &result,
                            bool inKm) {
  if (!initializer_.source) {
    result.addError(ErrorCode::MismatchedDimensions,
                    "Source must be initialized before it can be updated.");
    return;
  }
  params_.Pos->SxSyInKm = inKm;
  if (inKm) {
    z = z / 1000.0;
  }
  params_.Pos->Sx[0] = x;
  params_.Pos->Sy[0] = y;
  params_.Pos->Sy[0] = z;
  return;
}

void Agents::initializeSource(double x, double y, double z, bool inKm) {
  bhc::extsetup_sxsy(params_, kNumSources, kNumSources);
  bhc::extsetup_sz(params_, kNumSources);
  initializer_.source = true;
  updateSource(x, y, z, result_, inKm);
}

void Agents::updateReceivers(const std::vector<double> &x,
                               const std::vector<double> &y,
                               const std::vector<float> &z, Result &result,
                               bool inKm) {
  params_.Pos->RrInKm = inKm;
  params_.Pos->SxSyInKm = inKm;
  if (!(x.size() == y.size() || !(y.size() == z.size()))) {
    std::stringstream msg;
    msg << "Receiver coordinate vectors must be the same size.";
    msg << " (x size: " << x.size() << ", y size: " << y.size()
        << ", z size: " << z.size() << ")";
    result_.addError(ErrorCode::MismatchedDimensions, msg.str());
    return;
  }
  size_t nReceivers = x.size();

  // re-initializing if receivers have changed
  if (nReceivers != static_cast<size_t>(params_.Pos->NRr)) {
    bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(nReceivers));
    bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(nReceivers));
    bhc::extsetup_rcvrdepths(params_, nReceivers);
  }

  for (size_t i = 0; i < nReceivers; ++i) {
    double delta_x = x[i] - params_.Pos->Sx[0];
    double delta_y = y[i] - params_.Pos->Sy[0];
    params_.Pos->theta[i] = std::atan2(delta_y, delta_x);
    params_.Pos->Rr[i] = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
  }

  // Managing detph setup. Here need to deal with bellhop defaulting to meters
  for (size_t i = 0; i < nReceivers; ++i) {
    if (inKm) {
      params_.Pos->Rz[i] = z[i] / 1000.0f;
    } else {
      params_.Pos->Rz[i] = z[i];
    }
  }

  initializer_.receiver = true;

  return;
}
Result Agents::initializeReceivers(const std::vector<double> &x,
                                   const std::vector<double> &y,
                                   const std::vector<float> &z, bool inKm) {
  params_.Pos->RrInKm = inKm;
  params_.Pos->SxSyInKm = inKm;
  if (!(x.size() == y.size() || !(y.size() == z.size()))) {
    std::stringstream msg;
    msg << "Receiver coordinate vectors must be the same size.";
    msg << " (x size: " << x.size() << ", y size: " << y.size()
        << ", z size: " << z.size() << ")";
    std::cout << msg.str() << std::endl;
    result_.addError(ErrorCode::MismatchedDimensions, msg.str());
    return result_;
  }
  size_t nReceivers = x.size();
  bhc::extsetup_rcvrranges(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrbearings(params_, static_cast<int32_t>(nReceivers));
  bhc::extsetup_rcvrdepths(params_, nReceivers);

  initializer_.receiver = true;
  updateReceivers(x, y, z, result_, inKm);
  return result_;
}

} // namespace acoustics