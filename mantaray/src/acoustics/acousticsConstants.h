//
// Created by tko on 1/28/26.
//
#pragma once
#include <string>


namespace acoustics {
constexpr size_t kBathymetryBuffSize = 2;
constexpr size_t kMaxTitle = 80;
// Provides types for bathymetry interpolation types
constexpr char kBathymetryInterpLinearShort[] = "LS";
constexpr char kBathymetryCurveInterpShort[] = "CS";
constexpr int kNumSources = 1;
constexpr int kNumRecievers = 1;
constexpr int kNumBeams = 200;
constexpr double kDegree2Radians = M_PI / 180.0;
constexpr double kRadians2Degree = 1.0 / kDegree2Radians;
constexpr double kBeamSpreadRadians = 45.0 * kDegree2Radians;
constexpr double kBeamStepSizeRatio = 1.0 / 50.0;

enum class BathyInterpolationType {
  kLinear,
  kCurveInterp,
};
} // namespace acoustics
