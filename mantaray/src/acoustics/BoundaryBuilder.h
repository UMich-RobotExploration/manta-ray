//
// Created by tko on 1/29/26.
//

#pragma once

#include "Result.h"
#include "helpers.h"
#include <algorithm>
#include <bhc/bhc.hpp>
#include <stdexcept>
#include <vector>

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

  // No copy (cannot finalize twice)
  BoundaryBuilder(const BoundaryBuilder &) = delete;
  // Copy assignment operator. Utilized when a new object does not
  // need to be created but an existing object needs to be assigned
  BoundaryBuilder &operator=(const BoundaryBuilder &) = delete;

  // Move constructor being deleted to prevent moves
  // && is an r-value aka the Construct(a), the a in here
  BoundaryBuilder(BoundaryBuilder &&other) noexcept = delete;

  // Move assignment operator being delete to prevent moves
  BoundaryBuilder &operator=(BoundaryBuilder &&) = delete;

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
  Result validate();

private:
  bhc::bhcParams<true> &params_;
  Result result_ = Result();

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
  /**
   * @brief Validates that the top boundary in all corners is at least equal
   * to the bottom boundary
   * @details Does this by checking by follow rectangular vertex comparisons
   */
  void assertBoundariesValid();
  void assertCornerEq(size_t botX, size_t botY, size_t topX, size_t topY);
  std::pair<bhc::VEC23<true>, bhc::VEC23<true>>
  getCornerValues(size_t botX, size_t botY, size_t topX, size_t topY) const;
};

size_t getIndex(const bhc::BdryInfoTopBot<true> &boundary, size_t ix,
                size_t iy);

} // namespace acoustics
