//
// Created by tko on 1/29/26.
//

#include "BellhopValidator.h"
#include <cmath>
#include <sstream>

namespace acoustics {

ValidationResult BellhopValidator::validateSspBoundaryDepthAlignment(
    const bhc::bhcParams<true> &params) {
  ValidationResult result;

  // TODO: Implement SSP-Boundary depth alignment validation
  // Check params.Bdry->Top.hs.Depth == params.ssp->Seg.z[0]
  // Check params.Bdry->Bot.hs.Depth == params.ssp->Seg.z[NPts-1]
  // Add errors if misalignment detected with specific values

  return result;
}

ValidationResult BellhopValidator::validateBeamBoxWithinBounds(
    const bhc::bhcParams<true> &params) {
  ValidationResult result;

  // TODO: Implement beam box bounds validation
  // Check Beam->Box.x against boundary X extent
  // Check Beam->Box.y against boundary Y extent
  // Check Beam->Box.z against SSP maximum depth
  // Add errors if beam box exceeds any grid extent

  return result;
}

ValidationResult BellhopValidator::validateSourceReceiverPositions(
    const bhc::bhcParams<true> &params) {
  ValidationResult result;

  // TODO: Implement source/receiver position validation
  // Check source positions within boundary extents
  // Check receiver positions reachable
  // Add warnings if positions are outside typical domain

  return result;
}

ValidationResult
BellhopValidator::validateAll(const bhc::bhcParams<true> &params) {
  ValidationResult result;

  // Merge all validation results
  result.merge(validateSspBoundaryDepthAlignment(params));
  result.merge(validateBeamBoxWithinBounds(params));
  result.merge(validateSourceReceiverPositions(params));

  return result;
}

} // namespace acoustics
