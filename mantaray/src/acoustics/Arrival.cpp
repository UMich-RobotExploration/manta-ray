//
// Created by tko on 1/26/26.
//

#include "Arrival.h"

namespace acoustics {

Arrival::Arrival(bhc::bhcParams<true> &in_params, bhc::ArrInfo *arrival_info)
    : bhcParams(in_params), arrivalInfo(arrival_info) {
  // Default constructor implementation
  if (arrivalInfo == nullptr) {
    throw std::invalid_argument(
        "Arrival received null pointer to ArrInfo");
  }
}

} // namespace acoustics
