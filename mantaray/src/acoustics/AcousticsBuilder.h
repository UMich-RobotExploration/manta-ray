//
// Created by tko on 2/2/26.
//

#pragma once
#include "SimulationConfig.h"
#include <algorithm>
#include <array>
#include <bhc/bhc.hpp>
#include <optional>

namespace acoustics {
constexpr int kNumAltimetryPts = 2;
constexpr int kNumProvince = 1;

// TODO: Need to implement validation checks
// - Increasing bearing
// - Increasing range always


class AcousticsBuilder {
public:
  AcousticsBuilder() = delete;
  // No copy (cannot finalize twice)
  AcousticsBuilder(const AcousticsBuilder &) = delete;
  // Copy assignment operator. Utilized when a new object does not
  // need to be created but an existing object needs to be assigned
  AcousticsBuilder &operator=(const AcousticsBuilder &) = delete;

  // Move constructor being deleted to prevent moves
  // && is an r-value aka the Construct(a), the a in here
  AcousticsBuilder(AcousticsBuilder &&other) noexcept = delete;

  // Move assignment operator being delete to prevent moves
  AcousticsBuilder &operator=(AcousticsBuilder &&) = delete;

  /**
   * @brief Simulation builder constructor that takes ownership of ssp, bath,
   * and agents
   * @details Access configurations through getters
   * @param params
   * @param bathConfig
   * @param sspConfig
   * @param agentsConfig
   */
  explicit AcousticsBuilder(bhc::bhcParams<true> &params,
                             BathymetryConfig &bathConfig, SSPConfig &sspConfig,
                             AgentsConfig &agentsConfig);
  void build();
  void updateAgents();
  static void quadraticBathymetry3D(const std::vector<double> &gridX,
                                    const std::vector<double> &gridY,
                                    std::vector<double> &data, double depth);
  AgentsConfig &getAgentsConfig();

private:
  bhc::bhcParams<true> &params_;
  BathymetryConfig bathymetryConfig_;
  SSPConfig sspConfig_;
  AgentsConfig agentsConfig_;
  bool bathymetryBuilt_{false};
  bool agentsBuilt_{false};

  void buildBathymetry();
  void autogenerateAltimetry();
  void buildSSP();
  void syncBoundaryAndSSP();
  void buildAgents();
  static void flatAltimetery3D(bhc::BdryInfoTopBot<true> &boundary,
                               const BathymetryConfig &bathConfig);
};

} // namespace acoustics
