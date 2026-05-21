#pragma once

#include <cstdint>

struct KalmanHealthSnapshot {
  bool diverged = false;
  bool altitude_valid = false;
  bool velocity_valid = false;
  uint32_t imu_samples_consumed = 0;
  uint32_t baro_updates = 0;
  uint32_t gps_updates = 0;
  uint32_t imu_ring_hwm[4] = {0u, 0u, 0u, 0u};
  uint32_t last_kalman_loop_us = 0;
  uint32_t max_kalman_loop_us = 0;
  uint32_t last_main_loop_iteration_us = 0;
  uint32_t max_main_loop_iteration_us = 0;
  uint32_t yieldable_imu_drops = 0;
};

class KalmanHealthStore {
 public:
  static KalmanHealthStore& instance();

  void reset();
  void set(const KalmanHealthSnapshot& snapshot);
  KalmanHealthSnapshot get() const;

 private:
  KalmanHealthSnapshot snapshot_{};
};
