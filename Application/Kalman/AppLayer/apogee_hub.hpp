#pragma once

#include "Application/Kalman/AppLayer/apogee_algorithm.hpp"

#include <array>
#include <memory>

namespace app {

static constexpr size_t kMaxApogeeAlgorithms = 4;

struct AlgorithmResult {
  const char* name = nullptr;
  ApogeeVerdict verdict{};
  bool is_primary = false;
};

struct ApogeeDecision {
  bool triggered = false;
  uint8_t source_index = 0;
  ApogeeVerdict primary_verdict{};
  std::array<AlgorithmResult, kMaxApogeeAlgorithms> all_results{};
  size_t result_count = 0;
};

class ApogeeHub {
 public:
  ApogeeHub() = default;

  int addAlgorithm(std::unique_ptr<IApogeeAlgorithm> algo);
  void setPrimary(size_t index);
  size_t primaryIndex() const { return primary_index_; }

  void reset();
  void arm(uint32_t liftoff_ms);
  bool armed() const { return armed_; }

  ApogeeDecision update(uint32_t now_ms, const ApogeeInput& input);

  size_t algorithmCount() const { return count_; }
  IApogeeAlgorithm* algorithm(size_t i) {
    return i < count_ ? algos_[i].get() : nullptr;
  }
  const IApogeeAlgorithm* algorithm(size_t i) const {
    return i < count_ ? algos_[i].get() : nullptr;
  }

 private:
  std::array<std::unique_ptr<IApogeeAlgorithm>, kMaxApogeeAlgorithms> algos_{};
  size_t count_ = 0;
  size_t primary_index_ = 0;
  bool armed_ = false;
  bool latched_ = false;
};

}  // namespace app
