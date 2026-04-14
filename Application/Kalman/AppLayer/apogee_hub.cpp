#include "Application/Kalman/AppLayer/apogee_hub.hpp"

namespace app {

int ApogeeHub::addAlgorithm(std::unique_ptr<IApogeeAlgorithm> algo) {
  if (!algo || count_ >= kMaxApogeeAlgorithms) {
    return -1;
  }
  algos_[count_] = std::move(algo);
  return static_cast<int>(count_++);
}

void ApogeeHub::setPrimary(size_t index) {
  if (index < count_) {
    primary_index_ = index;
  }
}

void ApogeeHub::reset() {
  for (size_t i = 0; i < count_; ++i) {
    if (algos_[i]) {
      algos_[i]->reset();
    }
  }
  armed_ = false;
  latched_ = false;
}

void ApogeeHub::arm(uint32_t liftoff_ms) {
  for (size_t i = 0; i < count_; ++i) {
    if (algos_[i]) {
      algos_[i]->arm(liftoff_ms);
    }
  }
  armed_ = true;
  latched_ = false;
}

ApogeeDecision ApogeeHub::update(uint32_t now_ms, const ApogeeInput& input) {
  ApogeeDecision decision{};

  if (!armed_ || latched_) {
    return decision;
  }

  for (size_t i = 0; i < count_; ++i) {
    if (!algos_[i]) continue;

    const ApogeeVerdict verdict = algos_[i]->update(now_ms, input);

    decision.all_results[decision.result_count] =
        AlgorithmResult{algos_[i]->name(), verdict, (i == primary_index_)};
    ++decision.result_count;

    if (i == primary_index_) {
      decision.primary_verdict = verdict;
      decision.source_index = static_cast<uint8_t>(i);
      if (verdict.result == ApogeeVerdict::Result::Detected) {
        decision.triggered = true;
        latched_ = true;
      }
    }
  }

  return decision;
}

}  // namespace app
