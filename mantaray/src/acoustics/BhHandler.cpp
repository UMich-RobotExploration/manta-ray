//
// Created by tko on 1/27/26.
//

#include "BhHandler.h"

template<bool O3D, bool R3D>
class BellhopContext {
public:
  BellhopContext(const bhc::bhcInit &init) {
    // Call setup
    if (!bhc::setup<O3D, R3D>(init, params_, outputs_)) {
      throw std::runtime_error("bellhop setup failed");
    }
  }

  ~BellhopContext() {
    // Always finalize on destruction
    bhc::finalize<O3D, R3D>(params_, outputs_);
  }

  // No copy (cannot finalize twice)
  BellhopContext(const BellhopContext&) = delete;
  BellhopContext& operator=(const BellhopContext&) = delete;

  BellhopContext(BellhopContext&& other) noexcept
      : params_(std::move(other.params_)),
        outputs_(std::move(other.outputs_)) {
    // other.params_ / other.outputs_ are now dangling? but we disable copy so only one finalizer runs
  }

  BellhopContext& operator=(BellhopContext&&) = delete;

  // Provide accessors so library calls can use them
  bhc::bhcParams<O3D>& params() noexcept { return params_; }
  bhc::bhcOutputs<O3D, R3D>& outputs() noexcept { return outputs_; }

private:
  bhc::bhcParams<O3D> params_;
  bhc::bhcOutputs<O3D, R3D> outputs_;
};