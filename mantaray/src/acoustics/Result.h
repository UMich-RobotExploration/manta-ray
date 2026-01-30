#pragma once

#include <iostream>
#include <string>
#include <vector>

namespace acoustics {

enum class ErrorCode {
  MismatchedDimensions,
  Unknown
};

enum class WarningCode {
  NotImplementedCheck,
  Unknown
};

struct Result {
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  [[nodiscard]] bool hasErrors() const { return !errors.empty(); }

  [[nodiscard]] bool hasWarnings() const { return !warnings.empty(); }

  [[nodiscard]] bool isValid() const { return !hasErrors(); }

  void addError(ErrorCode code, const std::string &details = "") {
    errors.emplace_back(formatError(code, details));
  }

  void addWarning(WarningCode code, const std::string &details = "") {
    warnings.emplace_back(formatWarning(code, details));
  }

  void merge(const Result &other) {
    errors.insert(errors.end(), other.errors.begin(), other.errors.end());
    warnings.insert(warnings.end(), other.warnings.begin(), other.warnings.end());
  }

  void clear() {
    errors.clear();
    warnings.clear();
  }

  std::ostream& print(std::ostream &os = std::cout) const {
    if (hasErrors()) {
      os << "=== ERRORS ===" << "\n";
      for (size_t i = 0; i < errors.size(); ++i) {
        os << "  [ERROR " << (i + 1) << "] " << errors[i] << "\n";
      }
    }

    if (hasWarnings()) {
      os << "=== WARNINGS ===" << "\n";
      for (size_t i = 0; i < warnings.size(); ++i) {
        os << "  [WARNING " << (i + 1) << "] " << warnings[i] << "\n";
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

  static std::string formatWarning(WarningCode code, const std::string &details) {
    std::string msg = warningCodeToString(code);
    if (!details.empty()) {
      msg += ": " + details;
    }
    return msg;
  }

  static std::string errorCodeToString(ErrorCode code) {
    switch (code) {
      case ErrorCode::MismatchedDimensions:
        return "Mismatched boundary dimensions:";
      default:
        return "Unknown error";
    }
  }

  static std::string warningCodeToString(WarningCode code) {
    switch (code) {
      case WarningCode::NotImplementedCheck:
        return "Validation check not implemented:";
      default:
        return "Unknown warning";
    }
  }
};

} // namespace acoustics
