//
// Created by tko on 2/2/26.
//

#pragma once
#include "acoustics/SimulationConfig.h"
#include "acoustics/helpers.h"
#include "checkAssert.h"
#include <algorithm>
#include <array>
#include <bhc/bhc.hpp>

namespace acoustics {
constexpr int kNumAltimetryPts = 2;
constexpr int kNumProvince = 1;

// TODO: Need to implement validation checks
// - Beam box is within bounds of sim
// - Beam box has some minimum reasonable size to prevent rays from ending
//  immediately after being launched

// ============================================================================
// Acoustics Builder -
// ============================================================================
/**
 * @brief Builder class for constructing Bellhop acoustic simulations
 * @note All positions and dimensions must use consistent units as specified
 * in the configuration structs (e.g., meters vs kilometers).
 * @par Details:
 * Constructs bathymetry, altimetry, SSP, and agents based on input
 * configurations. Provides methods to update source and receiver positions
 * after initial construction.
 *
 * @par Bellhop integration:
 * Although not my preferred design, I attempt to minimize the potential
 * foot guns we could run into with the bellhop coupling and construction order
 *
 * @par Invariants:
 * -Bathymetry must be completely enclosed by SSP grid
 * -The build order is important and needs to be followed. See the current build
 * class
 */
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

  /**
   * @brief Creates bathymetry, altimetry, SSP, and agent configurations in
   * Bellhop.
   */
  void build();
  static void quadraticBathymetry3D(const std::vector<double> &gridX,
                                    const std::vector<double> &gridY,
                                    std::vector<double> &data, double depth);

  /** @brief Updates source position (MUST USE SAME UNITS AS CONFIG)
   *
   * @details UNITS WARNING AGAIN
   *
   * @throw std::out_of_range if new positions are out of bounds of the
   * simulation box
   */
  void updateSource(double x, double y, double z);

  /* @brief Updates sources
   * @see updateSources(double x, double y, double z)
 */
  void updateSource(const Eigen::Vector3d &position);

  /** @brief Updates receiver position (MUST USE SAME UNITS AS CONFIG)
   *
   * @details UNITS WARNING AGAIN
   *
   * @throw std::out_of_range if new positions are out of bounds of the
   * simulation box
   */
  void updateReceiver(double x, double y, double z);

  /* @brief Updates receivers
   * @see updateReceiver(double x, double y, double z)
 */
  void updateReceiver(const Eigen::Vector3d &position);

  /** @brief Validates that bathymetry is completely enclosed by ssp grid.
   *
   * @param bathGrid
   * @param sspGrid
   */
  void validateSPPandBathymetryBox(const Grid2D &bathGrid,
                                   const Grid3D &sspGrid) const;

  AgentsConfig &getAgentsConfig();
  const SSPConfig &getSSPConfig() const;

private:
  bhc::bhcParams<true> &params_;
  BathymetryConfig bathymetryConfig_;
  SSPConfig sspConfig_;
  AgentsConfig agentsConfig_;

  // INFO: could use std::optional<> here in the future to protect

  // Stores min box coords. Only valid after the simulation is built
  Eigen::Vector3d minCoords_{};
  // Stores max box coords. Only valid after the simulation is built
  Eigen::Vector3d maxCoords_{};

  // // minBoxWidth_ is set during bathymetry build to help ensure box is
  // // not too small
  // double minBoxWidth_{-1.0};
  bool bathymetryBuilt_{false};
  bool agentsBuilt_{false};
  bool beamBuilt_{false};

  /** @brief Constructs bathymetry based on bathymetry config
   *  @details Assumes a 1 province of all points.
   */
  void buildBathymetry();

  /** @brief Autogenerates altimetry based on bathymetry config.
   *  @details Coupled calling convention, bathymetry needs to be defined
   *  Constructs the simplest 4 point flat altimetry
   */
  void autogenerateAltimetry();
  void buildSSP();

  /** @brief Updates source and receiver positions in Bellhop based on input
   * parameters
   * @throw std::runtime_error if agents have not been built yet
   * @throw std::out_of_range if new positions are out of bounds of the
   * simulation box
   */
  void updateAgents();

  /** @brief Synchronize boundary depth values with SSP depth range
   *
   *  @details The library requires that after writing the SSP, the boundary
   * depths are set to match the SSP depth range: params.Bdry->Top.hs.Depth =
   * ssp->Seg.z[0] and params.Bdry->Bot.hs.Depth = ssp->Seg.z[ssp->NPts-1]
   */
  void syncBoundaryAndSSP();

  /**
   * @brief Initializes sources and receivers, then calls update function
   */
  void buildAgents();

  /**
   * @brief Constructs and aims ray's between sources and receivers
   *
   * @details For efficiency this functions assumes that sources and receivers
   * have already been built. So be warned! If sources or receivers are updated
   * after this function is called, the beam configuration will need to be
   * updated by calling this function again with the new bearing angle.
   *   * @param bearingAngle The angle in radians between source and receiver in
   * x-y plane.
   *
   * TODO: Ensure beam box actually is within bounds of SSP.
   */
  void constructBeam(double bearingAngle);
  static void flatAltimetery3D(bhc::BdryInfoTopBot<true> &boundary,
                               const BathymetryConfig &bathConfig);
};

} // namespace acoustics
