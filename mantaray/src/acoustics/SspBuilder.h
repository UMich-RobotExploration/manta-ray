//
// Created by tko on 1/29/26.
//

#pragma once

#include "Result.h"
#include <bhc/bhc.hpp>
#include <vector>

namespace acoustics {

/**
 * @brief Builder class for configuring Sound Speed Profile (SSP)
 *
 * Encapsulates the setup of 3D hexahedral sound speed profiles including
 * coordinate grids, sound speed matrices, and critical synchronization
 * with boundary depth values.
 */
class SspBuilder {
public:
  /**
   * @brief Construct an SspBuilder with reference to Bellhop parameters
   * @param params Reference to bhcParams that will be modified
   */
  explicit SspBuilder(bhc::bhcParams<true> &params);


  // No copy (cannot finalize twice)
  SspBuilder(const SspBuilder &) = delete;
  // Copy assignment operator. Utilized when a new object does not
  // need to be created but an existing object needs to be assigned
  SspBuilder &operator=(const SspBuilder &) = delete;

  // Move constructor being deleted to prevent moves
  // && is an r-value aka the Construct(a), the a in here
  SspBuilder(SspBuilder &&other) noexcept = delete;

  // Move assignment operator being delete to prevent moves
  SspBuilder &operator=(SspBuilder &&) = delete;
  /**
   * @brief Setup hexahedral SSP grid structure
   * @param nx Number of grid points in x direction
   * @param ny Number of grid points in y direction
   * @param nz Number of grid points in z direction
   */
  void setupHexahedral(int nx, int ny, int nz);

  /**
   * @brief Set coordinate grids for SSP
   * @param xCoords X-coordinate values
   * @param yCoords Y-coordinate values
   * @param zCoords Z-coordinate values (depth)
   */
  [[nodiscard]] Result setCoordinateGrid(const std::vector<double> &xCoords,
                         const std::vector<double> &yCoords,
                         const std::vector<double> &zCoords);

  /**
   * @brief Set whether range coordinates are in kilometers
   * @param inKm true if coordinates are in km, false if in meters
   */
  void setRangeUnits(bool inKm);

  /**
   * @brief Synchronize boundary depth values with SSP depth range
   *
   * Critical: Sets params.Bdry->Top.hs.Depth = ssp->Seg.z[0] and
   * params.Bdry->Bot.hs.Depth = ssp->Seg.z[Nz-1]
   */
  void syncBoundaryDepths();

  /**
   * @brief Mark SSP as dirty to trigger recomputation
   */
  void markDirty();

  /**
   * @brief Validate SSP configuration
   * @return ValidationResult containing any errors or warnings
   */
  Result validate() const;

private:
  bhc::bhcParams<true> &params_;
  bool initialized_ = false;
  Result result_ = Result();
};

} // namespace acoustics
