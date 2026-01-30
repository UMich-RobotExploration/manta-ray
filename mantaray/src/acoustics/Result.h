#pragma once

#include <iostream>
#include <string>
#include <vector>

namespace acoustics {

enum class ErrorCode { MismatchedDimensions, UninitializedBellhop, NullPtr, Unknown };

enum class WarningCode { NotImplementedCheck, SSPOutOfBounds, Unknown };

struct Result {
  std::vector<ErrorCode> errorCodes;
  std::vector<WarningCode> warningCodes;
  std::vector<std::string> errorsDetails;
  std::vector<std::string> warningsDetails;

  [[nodiscard]] bool hasErrors() const { return !errorCodes.empty(); }

  [[nodiscard]] bool hasWarnings() const { return !warningCodes.empty(); }

  [[nodiscard]] bool ok() const { return !hasErrors(); }
  [[nodiscard]] bool err() const { return hasErrors(); }

  void addError(ErrorCode code, const std::string &details = "") {
    errorCodes.emplace_back(code);
    errorsDetails.emplace_back(details);
  }

  void addWarning(WarningCode code, const std::string &details = "") {
    warningCodes.emplace_back(code);
    warningsDetails.emplace_back(details);
  }

  void merge(const Result &other) {
    errorCodes.insert(errorCodes.end(), other.errorCodes.begin(),
                      other.errorCodes.end());
    warningCodes.insert(warningCodes.end(), other.warningCodes.begin(),
                        other.warningCodes.end());
    errorsDetails.insert(errorsDetails.end(), other.errorsDetails.begin(),
                         other.errorsDetails.end());
    warningsDetails.insert(warningsDetails.end(), other.warningsDetails.begin(),
                           other.warningsDetails.end());
  }

  void clear() {
    errorCodes.clear();
    warningCodes.clear();
    errorsDetails.clear();
    warningsDetails.clear();
  }

  std::ostream &print(std::ostream &os = std::cout) const {
    if (hasErrors()) {
      os << "=== ERRORS ===" << "\n";
      for (size_t i = 0; i < errorsDetails.size(); ++i) {
        os << "  [ERROR " << (i + 1) << "] "
           << formatError(errorCodes[i], errorsDetails[i]) << "\n";
      }
    }

    if (hasWarnings()) {
      os << "=== WARNINGS ===" << "\n";
      for (size_t i = 0; i < warningsDetails.size(); ++i) {
        os << "  [WARNING " << (i + 1) << "] "
           << formatWarning(warningCodes[i], warningsDetails[i]) << "\n";
      }
    }

    if (!hasErrors() && !hasWarnings()) {
      os << "=== VALIDATION PASSED ===" << "\n";
    }
    return os;
  }

private:
  static std::string formatError(ErrorCode code, const std::string &details) {
    std::string msg = errorCodeToString(code);
    if (!details.empty()) {
      msg += ": " + details;
    }
    return msg;
  }

  static std::string formatWarning(WarningCode code,
                                   const std::string &details) {
    std::string msg = warningCodeToString(code);
    if (!details.empty()) {
      msg += ": " + details;
    }
    return msg;
  }

  static std::string errorCodeToString(ErrorCode code) {
    switch (code) {
    case ErrorCode::MismatchedDimensions:
      return "Mismatched dimensions:";
    case ErrorCode::UninitializedBellhop:
      return "Uninitialized bellhop:";
    case ErrorCode::NullPtr:
      return "NullPtr where should not be null:";
    default:
      return "Unknown error";
    }
  }

  static std::string warningCodeToString(WarningCode code) {
    switch (code) {
    case WarningCode::NotImplementedCheck:
      return "Validation check not implemented:";
    case WarningCode::SSPOutOfBounds:
      return "SSP Out of Bounds:";
    default:
      return "Unknown warning";
    }
  }
};

} // namespace acoustics
