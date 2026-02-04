//
// Created by tko on 1/28/26.
//
#pragma once
#include <string>

#ifndef MANTARAY_SIMCONTROLS_H
#define MANTARAY_SIMCONTROLS_H

namespace acoustics {
constexpr size_t kBathymetryBuffSize = 2;
constexpr size_t kMaxTitle = 80;
// Provides types for bathymetry interpolation types
constexpr char kBathymetryInterpLinearShort[] = "LS";
constexpr char kBathymetryCurveInterpShort[] = "CS";
constexpr int kNumSources = 1;
constexpr int kNumRecievers = 1;
constexpr int kNumBeams = 100;
constexpr double kDegree2Radians = M_PI / 180.0;
constexpr double kRadians2Degree = 1.0 / kDegree2Radians;
constexpr double kBeamSpreadRadians = 15.0 * kDegree2Radians;
constexpr double kBeamStepSizeRatio = 1.0 / 100.0;

enum class BathyInterpolationType {
  kLinear,
  kCurveInterp,
};
} // namespace acoustics
#endif // MANTARAY_SIMCONTROLS_H
