//
// Created by tko on 1/26/26.
//

#include "acoustics/pch.h"

#include "acoustics/Arrival.h"

namespace acoustics {

/**
 * @brief Calculate the flattened index for the 6D ArrInfo arrays
 * @details Code taken directly from Bellhopcuda source common.hpp as they don't
 * expose this WARN: DO NOT CHANGE. BELLHOPCUDA owns this LAYOUT!!
 */
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

std::string Arrival::printReceiverInfo(const bhc::Position *Pos, int32_t ir,
                                       int32_t iz, int32_t itheta) {

  auto rRange = Pos->Rr[ir];
  auto rDepth = Pos->Rz[iz];
  auto rTheta = Pos->theta[itheta];
  auto output =
      fmt::format("Reciever Position - Range: {} m, Depth: {} m, Bearing: {}",
                  rRange, rDepth, rTheta);
  return output;
}

size_t Arrival::getIdx(size_t ir, size_t iz, size_t itheta) const {
  return (ir * inputs.Pos->NRz_per_range + iz) * inputs.Pos->Ntheta + itheta;
}

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
ArrivalPair Arrival::getFastestArrivals() {
  const bhc::Position *Pos = inputs.Pos;

  CHECK(Pos->NRz_per_range == 1,
        "Z values should be singular per range. A potential issue is that "
        "regular grids ('I') were not used in the Runtype[4]");

  float minDirectDelay = std::numeric_limits<float>::max();
  float minAnyDelay = std::numeric_limits<float>::max();
  bool hasDirectArrival = false;
  bool hasAnyArrival = false;
  int multipathCount = 0;

  for (int32_t isz = 0; isz < Pos->NSz; ++isz) {
    for (int32_t isx = 0; isx < Pos->NSx; ++isx) {
      for (int32_t isy = 0; isy < Pos->NSy; ++isy) {
        for (int32_t itheta = 0; itheta < Pos->Ntheta; ++itheta) {
          for (int32_t iz = 0; iz < Pos->NRz_per_range; ++iz) {
            for (int32_t ir = 0; ir < Pos->NRr; ++ir) {
              size_t base = GetFieldAddr(isx, isy, isz, itheta, iz, ir, Pos);
              int32_t narr = arrInfo->NArr[base];

              for (size_t iArr = 0; iArr < static_cast<size_t>(narr); ++iArr) {
                const size_t arrayIdx = base * arrInfo->MaxNArr + iArr;
                const bhc::Arrival *arr = &arrInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
                if (delay < 0) {
                  SPDLOG_DEBUG("Found {} arrivals for {}", narr,
                               printReceiverInfo(Pos, ir, iz, itheta));
                  throw std::runtime_error(
                      "Negative delay encountered in arrival data");
                }

                // Track fastest arrival regardless of path type
                if (delay < minAnyDelay) {
                  minAnyDelay = delay;
                  hasAnyArrival = true;
                }

                // Track fastest direct-path (zero bounce) arrival
                if (arr->NTopBnc > 0 || arr->NBotBnc > 0) {
                  ++multipathCount;
                  SPDLOG_TRACE("Multipath arrival: delay={:.6f}s, "
                               "topBnc={}, botBnc={}",
                               delay, arr->NTopBnc, arr->NBotBnc);
                } else {
                  // Track fastest direct-path (zero bounce) arrival
                  if (delay < minDirectDelay) {
                    minDirectDelay = delay;
                    hasDirectArrival = true;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  if (!hasDirectArrival && multipathCount > 0) {
    SPDLOG_WARN("No direct-path arrival found, {} multipath arrivals present",
                multipathCount);
  }

  ArrivalPair result;
  result.directPath = hasDirectArrival ? minDirectDelay : kNoArrival;
  result.anyPath = hasAnyArrival ? minAnyDelay : kNoArrival;
  return result;
}

float Arrival::getLargestAmpArrival() {
  const bhc::Position *Pos = inputs.Pos;

  float arrivalDelay = -1;

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

              SPDLOG_DEBUG("Found {} arrivals for {}", narr,
                           printReceiverInfo(Pos, ir, iz, itheta));
              for (size_t iArr = 0; iArr < static_cast<size_t>(narr); ++iArr) {
                const size_t arrayIdx = base * arrInfo->MaxNArr + iArr;

                bhc::Arrival *arr = &arrInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
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
                arrivalDelay = minDelay;
              }
            }
          }
        }
      }
    }
  }
  return arrivalDelay;
}
void Arrival::getAllArrivals(ArrivalInfoDebug &arrivalInfo) {
  const bhc::Position *Pos = inputs.Pos;

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

              std::cout << "Found " << narr << " arrivals for:" << "\n\t";
              SPDLOG_DEBUG("Found {} arrivals for ", narr);
              printReceiverInfo(Pos, ir, iz, itheta);
              arrivalInfo.arrivalTimes.resize(narr, kNoArrival);
              arrivalInfo.arrivalTimesImaginary.resize(narr, kNoArrival);
              arrivalInfo.amplitude.resize(narr, kNoArrival);
              for (size_t iArr = 0; iArr < static_cast<size_t>(narr); ++iArr) {
                const size_t arrayIdx = base * arrInfo->MaxNArr + iArr;

                const bhc::Arrival *arr = &arrInfo->Arr[arrayIdx];
                auto delay = arr->delay.real();
                if (delay < 0) {
                  throw std::runtime_error(
                      "Negative delay encountered in arrival data");
                }
                arrivalInfo.arrivalTimes[iArr] = delay;
                arrivalInfo.arrivalTimesImaginary[iArr] = arr->delay.imag();
                arrivalInfo.amplitude[iArr] = arr->a;
              }
            }
          }
        }
      }
    }
  }
  return;
}
void ArrivalInfoDebug::logArrivalInfo(const std::string &filename) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    throw std::runtime_error("Failed to open file for writing: " +
                             std::string(filename));
  }

  outFile << "ArrivalTime(s),Amplitude,Imaginary,Range(m)\n";
  outFile << std::setprecision(10) << groundTruthArrivalTime << "," << 1.0
          << "," << 0.0 << "," << range << "\n";
  for (size_t i = 0; i < arrivalTimes.size(); ++i) {
    outFile << std::setprecision(20) << arrivalTimes[i] << "," << amplitude[i]
            << "," << arrivalTimesImaginary[i] << ","
            << arrivalTimes[i] * soundSpeed << "\n";
  }
  outFile.close();
}

} // namespace acoustics