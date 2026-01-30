//
// Created by tko on 1/30/26.
//
#pragma once
#include "Result.h"
#include "helpers.h"
#include <bhc/bhc.hpp>
#include <vector>

namespace acoustics {

constexpr int kNumSources = 1;
struct agentInitializer {
  bool source{false};
  bool receiver{false};
};

/// @brief Handles construction of receivers and sources within bellhop
/// @details Will steer beams towards appropriate agents
class Agents {
public:
  explicit Agents(bhc::bhcParams<true> &params);

  // No copy
  Agents(const Agents &) = delete;

  // Copy assignment operator. Utilized when a new object does not
  // need to be created but an existing object needs to be assigned
  Agents &operator=(const Agents &) = delete;

  // Move constructor being deleted to prevent moves
  // && is an r-value aka the Construct(a), the a in here
  Agents(Agents &&other) noexcept = delete;

  // Move assignment operator being delete to prevent moves
  Agents &operator=(Agents &&) = delete;

  /**
   * @brief Initialize a single source (multiple sources not supported)
   * @param inKm applies to all unlike bellhop
   */
  void initializeSource(double x, double y, double z, bool inKm = false);
  /**
   * @brief Updates a single source (multiple sources not supported)
   * @details Conducts a check to ensure it was initialized first
   * @param inKm applies to all unlike bellhop
   */
  [[nodiscard]] Result updateSource(double x, double y, double z, Result &result,
                    bool inKm = false);

  /**
   * @brief Initialize receivers
   * @param inKm applies to all unlike bellhop
   * @param z Z coordinate bizarrely needs to be float
   */
  [[nodiscard]] Result initializeReceivers(const std::vector<double> &x,
                             const std::vector<double> &y,
                             const std::vector<float> &z, bool inKm = false);

  [[nodiscard]] Result updateReceivers(const std::vector<double> &x,
                         const std::vector<double> &y,
                         const std::vector<float> &z, Result &result,
                         bool inKm = false);

private:
  bhc::bhcParams<true> &params_;
  agentInitializer initializer_ = agentInitializer();
  Result result_ = Result();
};

} // namespace acoustics
