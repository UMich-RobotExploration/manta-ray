/**
 * @file acousticsConstants.h
 * @brief Defines constants for bellhop and wrapper library
 */
#pragma once

namespace acoustics {
constexpr size_t kBathymetryBuffSize = 2;
constexpr size_t kMaxTitle = 80;
// Provides types for bathymetry interpolation types

// Linear interpolation
constexpr char kBathymetryInterpLinearShort[] = "LS";
// Curve interpolation
constexpr char kBathymetryCurveInterpShort[] = "CS";
// Code only supports 1 source
constexpr int kNumSources = 1;
// Code only supports 1 receiver
constexpr int kNumRecievers = 1;
// Total number of beams requested from Bellhop
constexpr int kNumBeams = 100;
// Conversion constant
constexpr double kDegree2Radians = M_PI / 180.0;
// Conversion constant
constexpr double kRadians2Degree = 1.0 / kDegree2Radians;
// Cone angle that will be swept by bellhop beam
constexpr double kBeamSpreadRadians = 20.0 * kDegree2Radians;
// Ratio of distance between source and receiver each ray will ds step by
constexpr double kBeamStepSizeRatio = 1.0 / 300.0;

enum class BathyInterpolationType {
  kLinear,
  kCurveInterp,
};
} // namespace acoustics
