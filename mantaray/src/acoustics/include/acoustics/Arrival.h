//
// Created by tko on 1/26/26.
//
#pragma once
#include "acoustics/helpers.h"
#include "checkAssert.h"
#include <bhc/bhc.hpp>
#include <bhc/structs.hpp>
#include <fstream>
#include <iomanip>
#include <vector>

namespace acoustics {
constexpr float kNoArrival = -1.0f;

/**
 * @brief Debugging struct for arrival information
 *
 * @details provides convenient methods to dump to logs and store outputs
 * Not used during simulation running
 */
struct ArrivalInfoDebug {
  std::vector<float> arrivalTimes;
  std::vector<float> arrivalTimesImaginary;
  std::vector<float> amplitude;
  double groundTruthArrivalTime{-1.0};
  double range{-1.0};
  double soundSpeed{-1.0};
  void logArrivalInfo(const std::string &filename);
};

/** @brief Class that extracts arrival information from bellhop output format
 * @details Checks to ensure that appropriate fields exist in bellhop output
 * to prevent segmentation faults through dereferencing of null pointers etc.
 */
class Arrival {
public:
  Arrival(bhc::bhcParams<true> &in_params,
          bhc::bhcOutputs<true, true> &outputs);
  /**
   * @brief Find the fastest arrival time for each reciever position.
   * @details Iterates through all source and receiver positions, checking the
   * arrival. Utilize getIdx to map 3D receiver indices to flattened vector
   * index.
   *
   * @return Flat3D Vector of earliest arrival times, kNoArrival denotes no
   * arrivals
   */
  float getFastestArrival();

  /**
   * @brief Returns largest amplitude arrival (not the shortest flight time)
   */
  float getLargestAmpArrival();
  void getAllArrivals(ArrivalInfoDebug &arrivalInfo);

private:
  const bhc::bhcParams<true> &inputs;
  bhc::bhcOutputs<true, true> &outputs;
  bhc::ArrInfo *arrInfo;

  /**
   * @brief Maps 3D receiver indices to flattened vector index.
   * @param ir NRr index
   * @param iz NRz_per_range index
   * @param itheta Ntheta index
   * @return index in flattened vector
   */
  size_t getIdx(size_t ir, size_t iz, size_t itheta) const;
  static std::string printReceiverInfo(const bhc::Position *Pos, int32_t ir,
                                       int32_t iz, int32_t itheta);
};
} // namespace acoustics
