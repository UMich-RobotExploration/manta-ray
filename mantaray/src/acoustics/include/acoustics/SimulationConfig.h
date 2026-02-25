/**
 * @file SimulationConfig.h
 * @brief All acoustics related configuration structs
 */
#pragma once
#include "acoustics/Grid.h"
#include "acoustics/acousticsConstants.h"
#include <Eigen/Dense>
#include <vector>

namespace acoustics {
/**
 * @brief Holds configuration for bathymetry setup
 * @details Assumes a flat top alitmetry, only applies single constant province
 * @param isKm applies to all dimensions of grid (unlike bellhop)
 */
struct BathymetryConfig {
  Grid2D Grid;
  BathyInterpolationType interpolation = BathyInterpolationType::kLinear;
  bool isKm{false};
};

/**
 * @brief Holds configuration for ssp setup
 * @param isKm applies to all dimensions of grid (unlike bellhop)
 */
struct SSPConfig {
  Grid3D Grid;
  bool isKm{false};
};

/**
 * @brief Source and Receiver configuration
 * @details Must be supplied in meters. This struct is legacy from when multiple
 * receivers and structs were supported.
 */
struct AgentsConfig {
  // Source (required)
  Eigen::Vector3d source;
  // Receivers (required)
  Eigen::Vector3d receiver;
};

}; // namespace acoustics