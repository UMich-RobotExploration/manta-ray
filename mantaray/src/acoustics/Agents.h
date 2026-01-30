//
// Created by tko on 1/30/26.
//
#pragma once
#include <bhc/bhc.hpp>

namespace acoustics {

/// @brief Handles construction of receivers and sources within bellhop
/// @details Will steer beams towards appropriate agents
class Agents {
public:
  explicit Agents(bhc::bhcParams<true> &params);

private:
  bhc::bhcParams<true> &params_;
};
} // namespace acoustics
