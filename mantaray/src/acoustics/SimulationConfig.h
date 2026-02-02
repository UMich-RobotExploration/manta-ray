//
// Created by tko on 2/2/26.
//
#pragma once
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "Grid3D.h"

// SimulationConfig.h
struct BathymetryConfig {
  acoustics::Grid2D<double> customGrid;
  acoustics::BathyInterpolationType interpolation =
      acoustics::BathyInterpolationType::kLinear;
  bool isKm{false};
};

struct SSPConfig {
  acoustics::Grid3D<double> customGrid;
  bool isKm{false};
};

struct SimulationConfig {
  // Required
  std::string title;
  std::string runType = "R";

  // Source (required)
  Eigen::Vector3d source;
  bool sourceInKm = false;

  // Receivers (required)
  std::vector<double> receiverX, receiverY;
  std::vector<float> receiverZ;
  bool receiversInKm = false;

  // Required
  BathymetryConfig &bathymetry;
  SSPConfig &ssp;

};