//
// Created by tko on 1/28/26.
//
#pragma once
#include <string>

#ifndef MANTARAY_SIMCONTROLS_H
#define MANTARAY_SIMCONTROLS_H

namespace acoustics {
// Provides types for bathymetry interpolation types
constexpr char kBathymetryInterpLinearShort[] = "LS";
constexpr char kBathymetryCurveInterpLinearShort[] = "CS";
constexpr size_t kBathymetryBuffSize = 2;
constexpr size_t kMaxTitle = 80;
} // namespace acoustics
#endif // MANTARAY_SIMCONTROLS_H
