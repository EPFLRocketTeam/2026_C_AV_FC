#pragma once

#include <stddef.h>
#include <cstdint>

namespace app {

class DescentNavFilter {
 public:
  struct Config {
    // Process noise parameters
    float accel_process_sigma = 3.0f;          // m/s^2 equivalent process sigma
    float baro_bias_rw_sigma = 0.08f;          // m/sqrt(s)
    float horizontal_velocity_decay_per_s = 0.45f;

    // Robust update controls
    float gate_sigma = 5.0f;
    float hard_gate_sigma = 400.0f;
    float huber_k = 2.5f;
    float min_variance = 1e-4f;

    // Per-update correction clamps
    float max_position_correction_m = 35.0f;
    float max_velocity_correction_mps = 20.0f;
    float max_baro_bias_correction_m = 8.0f;

    // Initial covariance defaults
    float init_position_var = 400.0f;
    float init_velocity_var = 100.0f;
    float init_baro_bias_var = 25.0f;
  };

  struct InitData {
    uint64_t timestamp_us = 0;
    float position_ned[3] = {0.0f, 0.0f, 0.0f};
    float velocity_ned[3] = {0.0f, 0.0f, 0.0f};
    float baro_bias_m = 0.0f;

    float position_var_ned[3] = {400.0f, 400.0f, 400.0f};
    float velocity_var_ned[3] = {100.0f, 100.0f, 100.0f};
    float baro_bias_var = 25.0f;
  };

  struct State {
    bool active = false;
    uint64_t timestamp_us = 0;

    float position_ned[3] = {0.0f, 0.0f, 0.0f};
    float velocity_ned[3] = {0.0f, 0.0f, 0.0f};
    float baro_bias_m = 0.0f;

    float position_var_ned[3] = {0.0f, 0.0f, 0.0f};
    float velocity_var_ned[3] = {0.0f, 0.0f, 0.0f};
    float baro_bias_var = 0.0f;
  };

  struct Stats {
    uint32_t gnss_pos_updates = 0;
    uint32_t gnss_pos_rejects = 0;
    uint32_t gnss_pos_clamps = 0;

    uint32_t gnss_vel_updates = 0;
    uint32_t gnss_vel_rejects = 0;
    uint32_t gnss_vel_clamps = 0;

    uint32_t baro_updates = 0;
    uint32_t baro_rejects = 0;
    uint32_t baro_clamps = 0;
    uint32_t baro_hold_skips = 0;
  };

  DescentNavFilter() = default;

  void configure(const Config& cfg);
  void reset();
  void enter(const InitData& init);

  // Hard reset to a GNSS state (used for first-fix and long-gap reacquire).
  // Preserves baro measurement-model continuity by shifting b_baro with delta p_d.
  void hardResetToGnss(uint64_t timestamp_us,
                       const float position_ned[3],
                       const float position_var_ned[3],
                       const float velocity_ned[3],
                       const float velocity_var_ned[3],
                       bool reset_velocity = false);

  bool isActive() const { return active_; }
  uint64_t timestampUs() const { return timestamp_us_; }

  void setBaroHoldUntilUs(uint64_t timestamp_us) { baro_hold_until_us_ = timestamp_us; }

  void predict(uint64_t timestamp_us);

  bool fuseGnssPosition(uint64_t timestamp_us,
                        const float position_ned[3],
                        const float variance_ned[3]);

  bool fuseGnssVelocity(uint64_t timestamp_us,
                        const float velocity_ned[3],
                        const float variance_ned[3]);

  bool fuseBaroDown(uint64_t timestamp_us,
                    float down_position_from_baro,
                    float variance);

  State state() const;
  const Stats& stats() const { return stats_; }

 private:
  bool updateAxisPos(size_t axis, float measurement, float variance);
  bool updateAxisVel(size_t axis, float measurement, float variance);
  bool updateDownWithBaro(float measurement, float variance);

  static float clampAbs(float value, float limit, bool& clamped);

  Config cfg_{};
  Stats stats_{};

  bool active_ = false;
  uint64_t timestamp_us_ = 0;
  uint64_t baro_hold_until_us_ = 0;

  // Horizontal axes use independent constant-velocity 2-state models: [p, v]
  float x2_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  float P2_[2][2][2] = {{{0.0f, 0.0f}, {0.0f, 0.0f}},
                        {{0.0f, 0.0f}, {0.0f, 0.0f}}};

  // Down axis includes baro bias state: [p_d, v_d, b_baro]
  float x3_[3] = {0.0f, 0.0f, 0.0f};
  float P3_[3][3] = {{0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f},
                     {0.0f, 0.0f, 0.0f}};
};

}  // namespace app
