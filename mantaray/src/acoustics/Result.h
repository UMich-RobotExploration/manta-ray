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
      default:
        return "Unknown error";
    }
  }

  static std::string warningCodeToString(WarningCode code) {
    switch (code) {
      default:
        return "Unknown warning";
    }
  }
};

} // namespace acoustics
