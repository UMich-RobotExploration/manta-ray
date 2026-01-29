//
// Created by tko on 1/29/26.
//

#pragma once

#include "ValidationResult.h"
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
  void setCoordinateGrid(const std::vector<float> &xCoords,
                         const std::vector<float> &yCoords,
                         const std::vector<float> &zCoords);

  /**
   * @brief Set sound speed matrix values
   * @param cMat Sound speed values indexed as (x*Ny+y)*Nz+z
   */
  void setSoundSpeedMatrix(const std::vector<float> &cMat);

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
  ValidationResult validate() const;

private:
  bhc::bhcParams<true> &params_;
};

} // namespace acoustics
