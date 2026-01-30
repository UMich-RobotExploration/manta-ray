//
// Created by tko on 1/29/26.
//

#pragma once

#include "Result.h"
#include "helpers.h"
#include <algorithm>
#include <bhc/bhc.hpp>
#include <vector>
#include <stdexcept>

namespace acoustics {

// Provides types for bathymetry interpolation types
constexpr char kBathymetryInterpLinearShort[] = "LS";
constexpr char kBathymetryCurveInterpShort[] = "CS";

enum class BathyInterpolationType {
  kLinear,
  kCurveInterp,
};
/**
 * @brief Builder class for configuring bathymetry and altimetry boundaries
 *
 * Encapsulates the setup of top (altimetry) and bottom (bathymetry) boundaries
 * for 3D acoustic simulations, including grid configuration, depth profiles,
 * interpolation types, and province management.
 */
class BoundaryBuilder {
public:
  /**
   * @brief Construct a BoundaryBuilder with reference to Bellhop parameters
   * @param params Reference to bhcParams that will be modified
   */
  explicit BoundaryBuilder(bhc::bhcParams<true> &params);

  /**
   * @brief Setup bathymetry grid and province structure
   * @param grid Grid dimensions [nx, ny]
   * @param nProvinces Number of bottom provinces for material properties
   */
  void setupBathymetry(const bhc::IORI2<true> &grid, int32_t nProvinces);

  /**
   * @brief Setup altimetry grid structure
   * @param grid Grid dimensions [nx, ny]
   */
  void setupAltimetry(const bhc::IORI2<true> &grid);

  /**
   * @brief Set bottom boundary shape using flat profile
   * @param depth Constant depth value
   * @param gridX X-coordinate grid points
   * @param gridY Y-coordinate grid points
   */
  void setFlatBottom(double depth, const std::vector<double> &gridX,
                     const std::vector<double> &gridY);

  /**
   * @brief Set bottom boundary shape using quadratic profile
   * @param depth Base depth value
   * @param gridX X-coordinate grid points
   * @param gridY Y-coordinate grid points
   */
  void setQuadraticBottom(double depth, const std::vector<double> &gridX,
                          const std::vector<double> &gridY,
                          bool autoGenerateTop);

  /**
   * @brief Set top boundary (surface) shape using flat profile
   * @param depth Constant depth value (typically 0.0 for surface)
   * @param gridX X-coordinate grid points
   * @param gridY Y-coordinate grid points
   */
  void setFlatTop(double depth, const std::vector<double> &gridX,
                  const std::vector<double> &gridY);

  /**
   * @brief Set interpolation type for boundary
   * @param interpType Interpolation enum
   * @param setTop true sets top, false sets bottom
   */
  void setInterpolationType(BathyInterpolationType interpType, bool setTop);

  /**
   * @brief Mark boundaries as dirty to trigger recomputation
   */
  void markDirty(bool setTop);

  /**
   * @brief Validate boundary configuration
   * @return ValidationResult containing any errors or warnings
   */
  Result validate() const;
  void assertBoundariesValid() const;

private:
  bhc::bhcParams<true> &params_;

  /**
   * @brief Helper to populate flat boundary structure
   */
  void flatBoundary3D(bhc::BdryInfoTopBot<true> &boundary, double depth,
                      const std::vector<double> &gridX,
                      const std::vector<double> &gridY);

  /**
   * @brief Helper to populate quadratic boundary structure
   */
  void quadBoundary3D(bhc::BdryInfoTopBot<true> &boundary, double depth,
                      const std::vector<double> &gridX,
                      const std::vector<double> &gridY);

  void assertBoundariesEqual();
  void assertCornerEq(size_t botX, size_t botY, size_t topX, size_t topY);
};

size_t getIndex(const bhc::BdryInfoTopBot<true> &boundary, size_t ix,
                size_t iy);

} // namespace acoustics
