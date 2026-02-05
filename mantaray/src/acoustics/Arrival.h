//
// Created by tko on 1/26/26.
//
#pragma once
#include <bhc/bhc.hpp>
#include <bhc/structs.hpp>
#include <iomanip>
#include <iostream>
#include <vector>
#include "checkAssert.h"
#include "helpers.h"


namespace acoustics {
constexpr float kNoArrival = -1.0f;
class Arrival {
public:
  Arrival(bhc::bhcParams<true> &in_params,
          bhc::bhcOutputs<true, true> &outputs);
  std::vector<float> extractEarliestArrivals();
  size_t getIdx(size_t ir, size_t iz, size_t itheta) const;

private:
  const bhc::bhcParams<true> &inputs;
  bhc::bhcOutputs<true, true> &outputs;
  bhc::ArrInfo *arrInfo;
  static void printReceiverInfo(const bhc::Position *Pos, int32_t ir,
                                int32_t iz, int32_t itheta);
};
} // namespace acoustics

