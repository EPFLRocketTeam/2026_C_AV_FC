#include "Application/Kalman/kalman_health.hpp"

KalmanHealthStore& KalmanHealthStore::instance() {
  static KalmanHealthStore store;
  return store;
}

void KalmanHealthStore::reset() {
  snapshot_ = KalmanHealthSnapshot{};
}

void KalmanHealthStore::set(const KalmanHealthSnapshot& snapshot) {
  snapshot_ = snapshot;
}

KalmanHealthSnapshot KalmanHealthStore::get() const {
  return snapshot_;
}
