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
  // New diagnostic fields
  uint32_t catchup_yield_count = 0;    // Times catchUp was called (from estimator)
  uint32_t catchup_budget_yields = 0;  // Times catchUp hit budget limit (from ESKF stats)
  uint32_t total_events_processed = 0; // Total ESKF events processed (cumulative)
  uint32_t kalman_behind_us = 0;       // ESKF time lag behind wall clock (us)
  uint32_t baro_corrections = 0;       // Baro corrections applied (cumulative)
  uint32_t imu_fifo_max_samples = 0;   // Max FIFO depth seen this interval (any sensor)
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
