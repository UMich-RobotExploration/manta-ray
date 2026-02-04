//
// Created by tko on 2/2/26.
//
#pragma once
#include "Grid.h"
#include <Eigen/Dense>
#include <vector>
#include "acousticsConstants.h"

namespace acoustics {
// SimulationConfig.h
/**
 * @brief Holds configuration for bathymetry setup
 * @details Assumes a flat top alitmetry, only applies single constant province
 * @param isKm applies to all dimensions of grid (unlike bellhop)
 */
struct BathymetryConfig {
  Grid2D<double> Grid;
  BathyInterpolationType interpolation = BathyInterpolationType::kLinear;
  bool isKm{false};
};

struct SSPConfig {
  Grid3D<double> Grid;
  bool isKm{false};
};

struct AgentsConfig {

  // Source (required)
  Eigen::Vector3d source;
  // Receivers (required)
  Eigen::Vector3d receiver;
  bool isKm = false;
};

}; // namespace acoustics