//
// Created by tko on 1/26/26.
//
#pragma once
#include "acoustics/helpers.h"
#include "checkAssert.h"
#include <bhc/bhc.hpp>
#include <bhc/structs.hpp>
#include <iomanip>
#include <iostream>
#include <vector>

namespace acoustics {
constexpr float kNoArrival = -1.0f;

struct ArrivalInfoDebug {
  std::vector<float> arrivalTimes;
  std::vector<float> arrivalTimesImaginary;
  std::vector<float> amplitude;
  double groundTruthArrivalTime{-1.0};
  double range{-1.0};
  double soundSpeed{-1.0};
  void logArrivalInfo(const std::string &filename);
};

class Arrival {
public:
  Arrival(bhc::bhcParams<true> &in_params,
          bhc::bhcOutputs<true, true> &outputs);
  float getFastestArrival();
  float getLargestAmpArrival();
  void getAllArrivals(ArrivalInfoDebug &arrivalInfo);
  size_t getIdx(size_t ir, size_t iz, size_t itheta) const;

private:
  const bhc::bhcParams<true> &inputs;
  bhc::bhcOutputs<true, true> &outputs;
  bhc::ArrInfo *arrInfo;
  static void printReceiverInfo(const bhc::Position *Pos, int32_t ir,
                                int32_t iz, int32_t itheta);
};
} // namespace acoustics
