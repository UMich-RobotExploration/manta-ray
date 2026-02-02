//
// Created by tko on 2/2/26.
//

#pragma once
#include "SimulationConfig.h"
#include <algorithm>
#include <array>
#include <bhc/bhc.hpp>

namespace acoustics {
constexpr int kNumAltimetryPts = 2;
constexpr int kNumProvince = 1;

class SimulationBuilder {
public:
  SimulationBuilder() = delete;
  // No copy (cannot finalize twice)
  SimulationBuilder(const SimulationBuilder &) = delete;
  // Copy assignment operator. Utilized when a new object does not
  // need to be created but an existing object needs to be assigned
  SimulationBuilder &operator=(const SimulationBuilder &) = delete;

  // Move constructor being deleted to prevent moves
  // && is an r-value aka the Construct(a), the a in here
  SimulationBuilder(SimulationBuilder &&other) noexcept = delete;

  // Move assignment operator being delete to prevent moves
  SimulationBuilder &operator=(SimulationBuilder &&) = delete;

  explicit SimulationBuilder(bhc::bhcParams<true> &params,
                             const BathymetryConfig &bathConfig,
                             const SSPConfig &sspConfig,
                             AgentsConfig &agentsConfig);
  void build();
  void updateAgents();

private:
  bhc::bhcParams<true> &params_;
  const BathymetryConfig &bathymetryConfig_;
  const SSPConfig &sspConfig_;
  AgentsConfig &agentsConfig_;
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
