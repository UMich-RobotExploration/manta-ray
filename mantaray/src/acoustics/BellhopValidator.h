//
// Created by tko on 1/29/26.
//

#pragma once

#include "Result.h"
#include <bhc/bhc.hpp>

namespace acoustics {

/**
 * @brief Validator for cross-component Bellhop parameter constraints
 *
 * Validates interdependencies between boundary, SSP, beam, and source/receiver
 * configurations that cannot be checked by individual builders alone.
 */
class BellhopValidator {
public:
  /**
   * @brief Validate that SSP depth bounds match boundary depth settings
   * @param params Bellhop parameters to validate
   * @return ValidationResult with errors if depths don't align
   *
   * Checks that:
   * - params.Bdry->Top.hs.Depth == params.ssp->Seg.z[0]
   * - params.Bdry->Bot.hs.Depth == params.ssp->Seg.z[Nz-1]
   */
  static Result
  validateSspBoundaryDepthAlignment(const bhc::bhcParams<true> &params);

  /**
   * @brief Validate that beam box stays within boundary and SSP grid extents
   * @param params Bellhop parameters to validate
   * @return ValidationResult with errors if beam box exceeds grids
   *
   * Checks that:
   * - Beam->Box.x <= max boundary X extent
   * - Beam->Box.y <= max boundary Y extent
   * - Beam->Box.z <= max SSP depth
   */
  static Result
  validateBeamBoxWithinBounds(const bhc::bhcParams<true> &params);

  /**
   * @brief Validate that source and receiver positions are within domain
   * @param params Bellhop parameters to validate
   * @return ValidationResult with warnings if positions are outside grids
   *
   * Checks that:
   * - Source positions within boundary extents
   * - Receiver positions reachable within beam box
   */
  static Result
  validateSourceReceiverPositions(const bhc::bhcParams<true> &params);

  /**
   * @brief Run all validation checks
   * @param params Bellhop parameters to validate
   * @return ValidationResult with all errors and warnings combined
   */
  static Result validateAll(const bhc::bhcParams<true> &params);
};

} // namespace acoustics
