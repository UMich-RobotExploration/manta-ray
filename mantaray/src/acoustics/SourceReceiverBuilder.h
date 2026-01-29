//
// Created by tko on 1/29/26.
//

#pragma once

#include "ValidationResult.h"
#include "helpers.h"
#include <bhc/bhc.hpp>

namespace acoustics {

/**
 * @brief Builder class for configuring source and receiver positions
 *
 * Encapsulates the setup of source locations and receiver geometry including
 * disk patterns, ranges, bearings, and depths.
 */
class SourceReceiverBuilder {
public:
  /**
   * @brief Construct a SourceReceiverBuilder with reference to Bellhop
   * parameters
   * @param params Reference to bhcParams that will be modified
   */
  explicit SourceReceiverBuilder(bhc::bhcParams<true> &params);

  /**
   * @brief Setup source position grid
   * @param nSx Number of source x positions
   * @param nSy Number of source y positions
   */
  void setupSourceGrid(int nSx, int nSy);

  /**
   * @brief Setup source depth positions
   * @param nSz Number of source depth positions
   */
  void setupSourceDepths(int nSz);

  /**
   * @brief Set single source position
   * @param x X coordinate
   * @param y Y coordinate
   * @param z Z coordinate (depth)
   * @param index Source index (default 0)
   */
  void setSourcePosition(float x, float y, float z, int index = 0);

  /**
   * @brief Setup receiver bearing angles
   * @param nBearings Number of bearing angles
   */
  void setupReceiverBearings(int nBearings);

  /**
   * @brief Setup receiver ranges
   * @param nRanges Number of range values
   */
  void setupReceiverRanges(int nRanges);

  /**
   * @brief Setup receiver depths
   * @param nDepths Number of depth values
   */
  void setupReceiverDepths(int nDepths);

  /**
   * @brief Set receiver bearing angles
   * @param minBearing Minimum bearing angle
   * @param maxBearing Maximum bearing angle
   */
  void setReceiverBearings(double minBearing, double maxBearing);

  /**
   * @brief Set receiver ranges
   * @param minRange Minimum range
   * @param maxRange Maximum range
   */
  void setReceiverRanges(double minRange, double maxRange);

  /**
   * @brief Set receiver depth value
   * @param depth Depth value
   * @param index Depth index (default 0)
   */
  void setReceiverDepth(float depth, int index = 0);

  /**
   * @brief Set whether source XY coordinates are in kilometers
   * @param inKm true if coordinates are in km, false if in meters
   */
  void setSourceXYUnits(bool inKm);

  /**
   * @brief Set whether receiver range coordinates are in kilometers
   * @param inKm true if coordinates are in km, false if in meters
   */
  void setReceiverRangeUnits(bool inKm);

  /**
   * @brief Validate source/receiver configuration
   * @return ValidationResult containing any errors or warnings
   */
  ValidationResult validate() const;

private:
  bhc::bhcParams<true> &params_;
};

} // namespace acoustics
