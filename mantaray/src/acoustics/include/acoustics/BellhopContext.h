#pragma once
#include <bhc/bhc.hpp>

/** @brief Namespace handling all acoustics and bellhop code
 */
namespace acoustics {

/**
 * @brief Class for ensure Bellhop memory gets cleaned up
 * @tparam O3D ocean 3D
 * @tparam R3D rays 3D
 */
template <bool O3D, bool R3D> class BhContext {
public:
  // Using explicit to prevent implicit conversions later on
  /**
   * @param init bhc initialization parameters
   */
  explicit BhContext(const bhc::bhcInit &init) {
    // Call setup
    if (!bhc::setup<O3D, R3D>(init, params_, outputs_)) {
      throw std::runtime_error("bellhop setup failed");
    }
  }

  // deleting default constructor
  BhContext() = delete;

  ~BhContext() {
    // Always finalize on destruction
    bhc::finalize<O3D, R3D>(params_, outputs_);
  }

  // No copy (cannot finalize twice)
  BhContext(const BhContext &) = delete;
  // Copy assignment operator. Utilized when a new object does not
  // need to be created but an existing object needs to be assigned
  BhContext &operator=(const BhContext &) = delete;

  // Move constructor being deleted to prevent moves
  // && is an r-value aka the Construct(a), the a in here
  BhContext(BhContext &&other) noexcept = delete;

  // Move assignment operator being delete to prevent moves
  BhContext &operator=(BhContext &&) = delete;

  // Provide accessors so library calls can use them
  bhc::bhcParams<O3D> &params() noexcept { return params_; }
  bhc::bhcOutputs<O3D, R3D> &outputs() noexcept { return outputs_; }

private:
  bhc::bhcParams<O3D> params_;
  bhc::bhcOutputs<O3D, R3D> outputs_;
};
} // namespace acoustics
