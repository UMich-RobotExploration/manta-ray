//
// Created by tko on 1/26/26.
//

#include "acoustics/pch.h"

#include "acoustics/Arrival.h"

namespace acoustics {

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

/**
 * @brief Maps 3D receiver indices to flattened vector index.
 * @param ir NRr index
 * @param iz NRz_per_range index
 * @param itheta Ntheta index
 * @return index in flattened vector
 */
size_t Arrival::getIdx(size_t ir, size_t iz, size_t itheta) const {
  return (ir * inputs.Pos->NRz_per_range + iz) * inputs.Pos->Ntheta + itheta;
}

/**
 * @brief Find the fastest arrival time for each reciever position.
 * @details Iterates through all source and receiver positions, checking the
 * arrival. Utilize getIdx to map 3D receiver indices to flattened vector index.
 * # TODO: Need to evaluate amplitude here still?
 *
 * @return Flat3D Vector of earliest arrival times, kNoArrival denotes no
 * arrivals
 */
std::vector<float> Arrival::getEarliestArrivals() {
  const bhc::Position *Pos = inputs.Pos;

  std::vector<float> arrivalDelays;
  arrivalDelays.assign(Pos->NRr * Pos->NRz_per_range * Pos->Ntheta, kNoArrival);
  // std::cout << "Number of receivers: "
  //           << Pos->NRr * Pos->NRz_per_range * Pos->Ntheta << "\n";
  // std::cout << "Number of receiver ranges: " << Pos->NRr
  //           << ", Number of Rz per range: " << Pos->NRz_per_range
  //           << ", Number Theta: " << Pos->Ntheta << "\n";

  CHECK(Pos->NRz_per_range == 1,
        "Z values should be singular per range. A potential issue is that "
        "regular grids ('I') were not used in the Runtype[4]");

  for (int32_t isz = 0; isz < Pos->NSz; ++isz) {
    for (int32_t isx = 0; isx < Pos->NSx; ++isx) {
      for (int32_t isy = 0; isy < Pos->NSy; ++isy) {
        // Now iterating through receiver points
        for (int32_t itheta = 0; itheta < Pos->Ntheta; ++itheta) {
          for (int32_t iz = 0; iz < Pos->NRz_per_range; ++iz) {
            for (int32_t ir = 0; ir < Pos->NRr; ++ir) {
              size_t base = GetFieldAddr(isx, isy, isz, itheta, iz, ir, Pos);
              // gives us number of rays we can iterate over
              int32_t narr = arrInfo->NArr[base];
              // Iterating over Individual Ray arrival times
              float minDelay = std::numeric_limits<float>::max();

              std::cout << "Found " << narr << " arrivals for:" << "\n\t";
              printReceiverInfo(Pos, ir, iz, itheta);
              for (size_t iArr = 0; iArr < static_cast<size_t>(narr); ++iArr) {
                const size_t arrayIdx = base * arrInfo->MaxNArr + iArr;

                bhc::Arrival *arr = &arrInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
                // std::cout << "Arrival Delay: " << std::setprecision(10) <<
                // delay
                // << "\n";
                // float amplitude_dB = 20.0f * std::log10(arr->a);
                // std::cout << "Amplitude: " << arr->a << " , " << amplitude_dB
                // << "\n";
                if (delay < 0) {
                  throw std::runtime_error(
                      "Negative delay encountered in arrival data");
                }
                minDelay = std::min(delay, minDelay);
              }
              if (narr != 0) {
                arrivalDelays[getIdx(ir, iz, itheta)] = minDelay;
              }
            }
          }
        }
      }
    }
  }

  return arrivalDelays;
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

std::vector<float> Arrival::getLargestAmpArrivals() {
  const bhc::Position *Pos = inputs.Pos;

  std::vector<float> arrivalDelays;
  arrivalDelays.assign(Pos->NRr * Pos->NRz_per_range * Pos->Ntheta, kNoArrival);
  // std::cout << "Number of receivers: "
  //           << Pos->NRr * Pos->NRz_per_range * Pos->Ntheta << "\n";
  // std::cout << "Number of receiver ranges: " << Pos->NRr
  //           << ", Number of Rz per range: " << Pos->NRz_per_range
  //           << ", Number Theta: " << Pos->Ntheta << "\n";

  CHECK(Pos->NRz_per_range == 1,
        "Z values should be singular per range. A potential issue is that "
        "regular grids ('I') were not used in the Runtype[4]");

  for (int32_t isz = 0; isz < Pos->NSz; ++isz) {
    for (int32_t isx = 0; isx < Pos->NSx; ++isx) {
      for (int32_t isy = 0; isy < Pos->NSy; ++isy) {
        // Now iterating through receiver points
        for (int32_t itheta = 0; itheta < Pos->Ntheta; ++itheta) {
          for (int32_t iz = 0; iz < Pos->NRz_per_range; ++iz) {
            for (int32_t ir = 0; ir < Pos->NRr; ++ir) {
              size_t base = GetFieldAddr(isx, isy, isz, itheta, iz, ir, Pos);
              // gives us number of rays we can iterate over
              int32_t narr = arrInfo->NArr[base];
              // Iterating over Individual Ray arrival times
              float maxAmp = std::numeric_limits<float>::min();
              float minDelay = -1.0;

              std::cout << "Found " << narr << " arrivals for:" << "\n\t";
              printReceiverInfo(Pos, ir, iz, itheta);
              for (size_t iArr = 0; iArr < static_cast<size_t>(narr); ++iArr) {
                const size_t arrayIdx = base * arrInfo->MaxNArr + iArr;

                bhc::Arrival *arr = &arrInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
                // std::cout << "Arrival Delay: " << std::setprecision(10) <<
                // delay
                // << "\n";
                // float amplitude_dB = 20.0f * std::log10(arr->a);
                // std::cout << "Amplitude: " << arr->a << " , " << amplitude_dB
                // << "\n";
                if (delay < 0) {
                  throw std::runtime_error(
                      "Negative delay encountered in arrival data");
                }
                if ((arr->a - maxAmp) >
                    std::numeric_limits<float>::epsilon() * 100) {
                  maxAmp = arr->a;
                  minDelay = delay;
                }
              }
              if (narr != 0) {
                arrivalDelays[getIdx(ir, iz, itheta)] = minDelay;
              }
            }
          }
        }
      }
    }
  }
  return arrivalDelays;
}

} // namespace acoustics