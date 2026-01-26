//
// Created by tko on 1/26/26.
//
#pragma once
#include <bhc/bhc.hpp>
#include <bhc/structs.hpp>

#ifndef MANTARAY_ARRIVAL_H
#define MANTARAY_ARRIVAL_H

namespace acoustics {
  class Arrival {
  public:
    Arrival(bhc::bhcParams<true>& in_params, bhc::ArrInfo* arrival_info);
  private:
    const bhc::bhcParams<true>& bhcParams;
    bhc::ArrInfo* arrivalInfo;
  };
} // namespace acoustics

#endif // MANTARAY_ARRIVAL_H
