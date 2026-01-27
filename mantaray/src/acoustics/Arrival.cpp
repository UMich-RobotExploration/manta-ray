//
// Created by tko on 1/26/26.
//

#include "Arrival.h"

namespace acoustics {
template <typename T> void printVector(const std::vector<T> &vec) {
  std::cout << "[ ";
  for (const auto &element : vec) {
    std::cout << element << " ";
  }
  std::cout << "]" << std::endl;
}

// Calculate the flattened index for the 6D ArrInfo arrays
// Code taken directly from Bellhopcuda source common.hpp as they don't expose
// this WARN: DO NOT CHANGE
inline size_t GetFieldAddr(int32_t isx, int32_t isy, int32_t isz,
                           int32_t itheta, int32_t id, int32_t ir,
                           const bhc::Position *Pos) {
  // clang-format off
  return (((((size_t)isz
      * (size_t)Pos->NSx + (size_t)isx)
      * (size_t)Pos->NSy + (size_t)isy)
      * (size_t)Pos->Ntheta + (size_t)itheta)
      * (size_t)Pos->NRz_per_range + (size_t)id)
      * (size_t)Pos->NRr + (size_t)ir;
  // clang-format on
}

Arrival::Arrival(bhc::bhcParams<true> &in_params,
                 bhc::bhcOutputs<true, true> &outputs)
    : inputs(in_params), outputs(outputs) {
  if (!outputs.arrinfo) {
    throw std::invalid_argument("Arrival received null pointer to ArrInfo");
  }
  this->arrInfo = outputs.arrinfo;
  if (!inputs.Pos) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals received null pointer to Position");
  }

  // Check critical arrivalInfo members
  if (!arrInfo->NArr) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals received null pointer to NArr");
  }
  if (!arrInfo->Arr) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals received null pointer to Arr");
  }
  // guarding for more than one source, don't support this
  if (inputs.Pos->NSx > 1 || inputs.Pos->NSy > 1 || inputs.Pos->NSz > 1) {
    throw std::invalid_argument(
        "Arrival.extractEarliestArrivals only supports single source");
  }
}

void Arrival::printReceiverInfo(const bhc::Position *Pos, int32_t ir,
                                int32_t iz, int32_t itheta) {

  auto rRange = Pos->Rr[ir];
  auto rDepth = Pos->Rz[iz];
  auto rTheta = Pos->theta[itheta];
  std::cout << "Receiver Position - Range: " << rRange
            << " m, Depth: " << rDepth << " m, Bearing: " << rTheta
            << " degrees\n";
}

std::vector<float> Arrival::extractEarliestArrivals() {
  const bhc::Position *Pos = inputs.Pos;

  for (int32_t isz = 0; isz < Pos->NSz; ++isz) {
    for (int32_t isx = 0; isx < Pos->NSx; ++isx) {
      for (int32_t isy = 0; isy < Pos->NSy; ++isy) {
        std::vector<float> arrivalDelays;
        arrivalDelays.reserve(Pos->NRr);

        // Now iterating through receiver points
        for (int32_t itheta = 0; itheta < Pos->Ntheta; ++itheta) {
          for (int32_t iz = 0; iz < Pos->NRz_per_range; ++iz) {
            for (int32_t ir = 0; ir < Pos->NRr; ++ir) {
              size_t base = GetFieldAddr(isx, isy, isz, itheta, iz, ir, Pos);
              // gives us number of rays we can iterate over
              int32_t narr = arrInfo->NArr[base];
              // Iterating over Individual Ray arrival times
              float minDelay = std::numeric_limits<float>::max();

              for (size_t iArr = 0; iArr < static_cast<size_t>(narr); ++iArr) {
                const size_t arrayIdx = base * arrInfo->MaxNArr + iArr;

                bhc::Arrival *arr = &arrInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
                std::cout << "Arrival Delay: " << std::setprecision(10) << delay
                          << "\n";
                std::cout << isz << "," << isx << "," << isy << "," << itheta
                          << "," << iz << "," << ir << "," << iArr << "\n";
                if (delay < 0) {
                  printReceiverInfo(Pos, ir, iz, itheta);
                  std::cout << "Number of arrivals: " << narr << "\n";
                  throw std::runtime_error(
                      "Negative delay encountered in arrival data");
                }
                minDelay = std::min(delay, minDelay);
              }
              if (narr != 0) {
                arrivalDelays.emplace_back(minDelay);
                printVector(arrivalDelays);
                std::cout << "Min Delay for Receiver: " << minDelay << "\n";
              }
            }
          }
        }
      }
    }
  }
  return std::vector<float>{};

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
}

} // namespace acoustics