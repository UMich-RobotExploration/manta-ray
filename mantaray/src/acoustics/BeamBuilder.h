//
// Created by tko on 1/29/26.
//

#pragma once

#include "Result.h"
#include "helpers.h"
#include <bhc/bhc.hpp>

namespace acoustics {

/**
 * @brief Builder class for configuring beam/ray tracing parameters
 *
 * Encapsulates the setup of ray bearings, elevations, beam box constraints,
 * and angle units for acoustic ray tracing simulations.
 */
class BeamBuilder {
public:
  /**
   * @brief Construct a BeamBuilder with reference to Bellhop parameters
   * @param params Reference to bhcParams that will be modified
   */
  explicit BeamBuilder(bhc::bhcParams<true> &params);

  /**
   * @brief Setup ray bearings (horizontal angles)
   * @param nBearings Number of bearing angles
   * @param minAngle Minimum bearing angle
   * @param maxAngle Maximum bearing angle
   */
  void setupRayBearings(int nBearings, double minAngle, double maxAngle);

  /**
   * @brief Setup ray elevations (vertical angles)
   * @param nElevations Number of elevation angles
   * @param minAngle Minimum elevation angle
   * @param maxAngle Maximum elevation angle
   */
  void setupRayElevations(int nElevations, double minAngle, double maxAngle);

  /**
   * @brief Set beam box spatial limits
   * @param x Maximum x extent
   * @param y Maximum y extent
   * @param z Maximum z extent (depth)
   */
  void setBeamBox(double x, double y, double z);

  /**
   * @brief Set ray tracing step size
   * @param deltas Step size for ray integration
   */
  void setStepSize(double deltas);

  /**
   * @brief Set whether angles are in degrees or radians
   * @param inDegrees true for degrees, false for radians
   */
  void setAngleUnits(bool inDegrees);

  /**
   * @brief Set whether range coordinates are in kilometers
   * @param inKm true if coordinates are in km, false if in meters
   */
  void setRangeUnits(bool inKm);

  /**
   * @brief Validate beam configuration
   * @return ValidationResult containing any errors or warnings
   */
  Result validate() const;

private:
  bhc::bhcParams<true> &params_;
};

} // namespace acoustics
