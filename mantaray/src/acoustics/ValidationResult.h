//
// Created by tko on 1/29/26.
//

#pragma once

#include <iostream>
#include <string>
#include <vector>

namespace acoustics {

/**
 * @brief Generic validation result container for tracking errors and warnings
 *
 * This struct provides a consistent way to collect and report validation issues
 * across all builders (BoundaryBuilder, SspBuilder, BeamBuilder,
 * SourceReceiverBuilder) and the BellhopValidator.
 *
 * @example Basic usage:
 * @code
 * ValidationResult result;
 *
 * // Add validation messages
 * if (depth < 0) {
 *   result.addError("Depth must be positive, got: " + std::to_string(depth));
 * }
 * if (gridSize < 10) {
 *   result.addWarning("Grid size " + std::to_string(gridSize) +
 *                     " is small, consider using >= 10 for better accuracy");
 * }
 *
 * // Check validation status
 * if (!result.isValid()) {
 *   result.print();
 *   return 1;
 * }
 * @endcode
 *
 * @example Merging multiple validation results:
 * @code
 * ValidationResult boundaryResult = boundaryBuilder.validate();
 * ValidationResult sspResult = sspBuilder.validate();
 * ValidationResult beamResult = beamBuilder.validate();
 *
 * ValidationResult combined;
 * combined.merge(boundaryResult);
 * combined.merge(sspResult);
 * combined.merge(beamResult);
 *
 * if (combined.hasErrors()) {
 *   std::cerr << "Validation failed:" << std::endl;
 *   combined.print(std::cerr);
 *   throw std::runtime_error("Configuration validation failed");
 * }
 *
 * if (combined.hasWarnings()) {
 *   std::cout << "Warnings detected:" << std::endl;
 *   combined.print(std::cout);
 * }
 * @endcode
 */
struct ValidationResult {
  /// Collection of error messages indicating validation failures
  std::vector<std::string> errors;

  /// Collection of warning messages indicating potential issues
  std::vector<std::string> warnings;

  /**
   * @brief Check if any errors were recorded
   * @return true if errors vector is not empty
   *
   * @example
   * @code
   * if (result.hasErrors()) {
   *   std::cerr << "Cannot proceed due to validation errors" << std::endl;
   * }
   * @endcode
   */
  bool hasErrors() const { return !errors.empty(); }

  /**
   * @brief Check if any warnings were recorded
   * @return true if warnings vector is not empty
   *
   * @example
   * @code
   * if (result.hasWarnings()) {
   *   std::cout << "Note: Configuration has warnings" << std::endl;
   * }
   * @endcode
   */
  bool hasWarnings() const { return !warnings.empty(); }

  /**
   * @brief Check if validation passed (no errors)
   * @return true if errors vector is empty (warnings don't affect validity)
   *
   * @example
   * @code
   * ValidationResult result = validator.validateAll(params);
   * if (result.isValid()) {
   *   bhc::run(params, outputs);
   * }
   * @endcode
   */
  bool isValid() const { return !hasErrors(); }

  /**
   * @brief Add an error message
   * @param msg Error message to add
   *
   * @example
   * @code
   * if (params.Beam->Box.x <= 0) {
   *   result.addError("Beam box X dimension must be positive, got: " +
   *                   std::to_string(params.Beam->Box.x));
   * }
   * @endcode
   */
  void addError(const std::string &msg) { errors.emplace_back(msg); }

  /**
   * @brief Add a warning message
   * @param msg Warning message to add
   *
   * @example
   * @code
   * if (params.ssp->NPts < 5) {
   *   result.addWarning("SSP has only " + std::to_string(params.ssp->NPts) +
   *                     " points, consider using more for smoother profile");
   * }
   * @endcode
   */
  void addWarning(const std::string &msg) { warnings.emplace_back(msg); }

  /**
   * @brief Merge another ValidationResult into this one
   * @param other ValidationResult to merge from
   *
   * Appends all errors and warnings from other into this result.
   *
   * @example
   * @code
   * ValidationResult selfValidation = builder.validate();
   * ValidationResult crossValidation = validator.validateBeamBox(params);
   *
   * ValidationResult total;
   * total.merge(selfValidation);
   * total.merge(crossValidation);
   * @endcode
   */
  void merge(const ValidationResult &other) {
    errors.insert(errors.end(), other.errors.begin(), other.errors.end());
    warnings.insert(warnings.end(), other.warnings.begin(), other.warnings.end());
  }

  /**
   * @brief Clear all errors and warnings
   *
   * @example
   * @code
   * ValidationResult result;
   * result.addError("test");
   * result.clear();
   * assert(result.isValid());
   * @endcode
   */
  void clear() {
    errors.clear();
    warnings.clear();
  }

  /**
   * @brief Print formatted errors and warnings to output stream
   * @param os Output stream to write to (defaults to std::cout)
   *
   * @example
   * @code
   * ValidationResult result = validator.validateAll(params);
   * if (!result.isValid()) {
   *   result.print(std::cerr);  // Print to stderr
   * } else if (result.hasWarnings()) {
   *   result.print();  // Print to stdout
   * }
   * @endcode
   */
  void print(std::ostream &os = std::cout) const {
    if (hasErrors()) {
      os << "=== ERRORS ===" << std::endl;
      for (size_t i = 0; i < errors.size(); ++i) {
        os << "  [ERROR " << (i + 1) << "] " << errors[i] << std::endl;
      }
    }

    if (hasWarnings()) {
      os << "=== WARNINGS ===" << std::endl;
      for (size_t i = 0; i < warnings.size(); ++i) {
        os << "  [WARNING " << (i + 1) << "] " << warnings[i] << std::endl;
      }
    }

    if (!hasErrors() && !hasWarnings()) {
      os << "=== VALIDATION PASSED ===" << std::endl;
    }
  }
};

} // namespace acoustics
