//
// Created by tko on 1/26/26.
//
#pragma once
#include <bhc/bhc.hpp>
#include <bhc/structs.hpp>
#include <vector>


#ifndef MANTARAY_ARRIVAL_H
#define MANTARAY_ARRIVAL_H

namespace acoustics {
  class Arrival {
  public:
    Arrival(bhc::bhcParams<true>& in_params, bhc::ArrInfo* arrival_info);
    std::vector<float> extractEarliestArrivals();
  private:
    const bhc::bhcParams<true>& bhcParams;
    bhc::ArrInfo* arrivalInfo;
    void printReceiverInfo(const bhc::Position *Pos, int32_t ir, int32_t iz,
                           int32_t itheta);

  };
} // namespace acoustics

#endif // MANTARAY_ARRIVAL_H
