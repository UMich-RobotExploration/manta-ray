//
// Created by tko on 1/26/26.
//

#include "Arrival.h"

#include "../../deps/bellhopcuda/src/common.hpp"
#include <iostream>

namespace acoustics {

Arrival::Arrival(bhc::bhcParams<true> *in_params, bhc::ArrInfo *arrival_info)
    : bhcParams(in_params), arrivalInfo(arrival_info) {
  if (bhcParams == nullptr) {
    throw std::invalid_argument("Arrival received null pointer to bhcParams");
  }
  if (arrivalInfo == nullptr) {
    throw std::invalid_argument("Arrival received null pointer to ArrInfo");
  }
  if (bhcParams->Pos == nullptr) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals received null pointer to Position");
  }

  // Check critical arrivalInfo members
  if (arrivalInfo->NArr == nullptr) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals received null pointer to NArr");
  }
  if (arrivalInfo->Arr == nullptr) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals received null pointer to Arr");
  }
}
void Arrival::extractEarliestArrivals() {
  /*
   * ArrInfo data structure layout:
   *
   * The arrival information is organized in a nested hierarchy by source and
   * receiver positions:
   *
   *   isz (source depth index)
   *     └─ isx (source x position index)
   *        └─ isy (source y position index)
   *           └─ itheta (receiver bearing/azimuth index)
   *              └─ iz (receiver depth index)
   *                 └─ ir (receiver range index)
   *                    └─ Arrival array: Arr[base * MaxNArr + iArr]
   *
   * For each receiver location (ir, iz, itheta) relative to a source (isx, isy,
   * isz):
   *   - GetFieldAddr() computes a unique base index
   *   - NArr[base] stores the count of arrivals at this location
   *   - Arr[base * MaxNArr + iArr] accesses the iArr-th arrival (0 to
   * NArr[base]-1)
   *
   * Memory layout uses a flattened 1D array where MaxNArr is the maximum
   * arrivals per location.
   */
  const bhc::Position *Pos = bhcParams->Pos;

  // Iterating through sources
  for (int32_t isz = 0; isz < Pos->NSz; ++isz) {
    for (int32_t isx = 0; isx < Pos->NSx; ++isx) {
      for (int32_t isy = 0; isy < Pos->NSy; ++isy) {
        // Now iterating through receiver points
        for (int32_t itheta = 0; itheta < Pos->Ntheta; ++itheta) {
          for (int32_t iz = 0; iz < Pos->NRz_per_range; ++iz) {
            for (int32_t ir = 0; ir < Pos->NRr; ++ir) {
              int32_t base =
                  bhc::GetFieldAddr(isx, isy, isz, itheta, iz, ir, Pos);
              // gives us number of arrays we can iterate over
              int32_t narr = arrivalInfo->NArr[base];
              // Iterating over Individual Ray arrival times
              float minDelay = std::numeric_limits<float>::max();
              if (narr == 0) {
                throw std::runtime_error("No Arrivals for given receiver location");
              }
              for (int32_t iArr = 0; iArr < narr; ++iArr) {
                int32_t arrayIdx = base * arrivalInfo->MaxNArr + iArr;
                bhc::Arrival *arr = &arrivalInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
                if (delay < 0) {
                  throw std::runtime_error(
                      "Negative arrival delay encountered, invalid data");
                }
                if (delay <= minDelay) {
                  minDelay = delay;
                }
              }
              auto rRange = Pos->Rr[ir];
              auto rDepth = Pos->Rz[iz];
              auto rTheta = Pos->theta[itheta];
              std::cout << "Receiver Position - Range: " << rRange
                        << " m, Depth: " << rDepth << " m, Bearing: " << rTheta
                        << " degrees\n";
              std::cout << "Min Delay for Receiver: " << minDelay << "\n";
            }
          }
        }
      }
    }
  }
}

} // namespace acoustics
