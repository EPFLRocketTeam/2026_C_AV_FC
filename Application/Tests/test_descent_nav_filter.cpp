#include "Application/Kalman/AppLayer/descent_nav_filter.hpp"

#include <gtest/gtest.h>

using app::DescentNavFilter;

TEST(KalmanDescentParity, PredictConstantVelocityMotion) {
  DescentNavFilter filter;
  filter.reset();

  DescentNavFilter::InitData init{};
  init.timestamp_us = 0;
  init.position_ned[0] = 0.0f;
  init.position_ned[1] = 0.0f;
  init.position_ned[2] = 100.0f;
  init.velocity_ned[0] = 5.0f;
  init.velocity_ned[1] = -2.0f;
  init.velocity_ned[2] = 10.0f;
  filter.enter(init);

  filter.predict(1000000ULL);
  const auto s = filter.state();

  EXPECT_TRUE(s.active);
  EXPECT_NEAR(s.position_ned[0], 5.0f, 0.15f);
  EXPECT_NEAR(s.position_ned[1], -2.0f, 0.15f);
  EXPECT_NEAR(s.position_ned[2], 110.0f, 0.2f);
  EXPECT_NEAR(s.velocity_ned[0], 3.188f, 0.05f);
  EXPECT_NEAR(s.velocity_ned[1], -1.275f, 0.05f);
  EXPECT_NEAR(s.velocity_ned[2], 10.0f, 0.05f);
}

TEST(KalmanDescentParity, GnssPositionUpdatePullsState) {
  DescentNavFilter filter;
  filter.reset();

  DescentNavFilter::InitData init{};
  init.timestamp_us = 0;
  init.position_ned[0] = 100.0f;
  init.velocity_ned[0] = 0.0f;
  init.position_var_ned[0] = 400.0f;
  init.position_var_ned[1] = 400.0f;
  init.position_var_ned[2] = 400.0f;
  filter.enter(init);

  const float z_pos[3] = {0.0f, 0.0f, 0.0f};
  const float R_pos[3] = {1.0f, 1.0f, 1.0f};
  const bool accepted = filter.fuseGnssPosition(100000ULL, z_pos, R_pos);

  const auto s = filter.state();
  EXPECT_TRUE(accepted);
  EXPECT_LT(s.position_ned[0], 100.0f);
  EXPECT_GE(filter.stats().gnss_pos_updates, 1u);
}

TEST(KalmanDescentParity, LargeOutlierRejected) {
  DescentNavFilter filter;
  filter.reset();

  DescentNavFilter::Config cfg{};
  cfg.gate_sigma = 4.0f;
  cfg.hard_gate_sigma = 4.0f;
  filter.configure(cfg);

  DescentNavFilter::InitData init{};
  init.timestamp_us = 0;
  init.position_var_ned[0] = 1.0f;
  init.velocity_var_ned[0] = 1.0f;
  filter.enter(init);

  const float z_pos[3] = {5000.0f, 5000.0f, 5000.0f};
  const float R_pos[3] = {1.0f, 1.0f, 1.0f};
  const bool accepted = filter.fuseGnssPosition(10000ULL, z_pos, R_pos);

  const auto s = filter.state();
  EXPECT_FALSE(accepted);
  EXPECT_NEAR(s.position_ned[0], 0.0f, 0.01f);
  EXPECT_GE(filter.stats().gnss_pos_rejects, 1u);
}

TEST(KalmanDescentParity, BaroBiasObservable) {
  DescentNavFilter filter;
  filter.reset();

  DescentNavFilter::InitData init{};
  init.timestamp_us = 0;
  init.position_ned[2] = 100.0f;
  init.velocity_ned[2] = 0.0f;
  init.baro_bias_m = 0.0f;
  init.position_var_ned[2] = 100.0f;
  init.velocity_var_ned[2] = 25.0f;
  init.baro_bias_var = 100.0f;
  filter.enter(init);

  for (int i = 0; i < 20; ++i) {
    const uint64_t ts = 100000ULL * static_cast<uint64_t>(i + 1);
    filter.predict(ts);
    (void)filter.fuseBaroDown(ts, 110.0f, 1.0f);
  }

  const auto s = filter.state();
  const float reconstructed = s.position_ned[2] + s.baro_bias_m;
  EXPECT_NEAR(reconstructed, 110.0f, 2.0f);
  EXPECT_GT(s.baro_bias_m, 0.0f);
}

TEST(KalmanDescentParity, BaroHoldWindowSkipsCorrection) {
  DescentNavFilter filter;
  filter.reset();

  DescentNavFilter::InitData init{};
  init.timestamp_us = 0;
  init.position_ned[2] = 50.0f;
  init.velocity_ned[2] = 0.0f;
  filter.enter(init);
  filter.setBaroHoldUntilUs(2000000ULL);

  const bool accepted = filter.fuseBaroDown(1000000ULL, 150.0f, 1.0f);
  const auto s = filter.state();

  EXPECT_FALSE(accepted);
  EXPECT_NEAR(s.position_ned[2], 50.0f, 0.2f);
  EXPECT_GE(filter.stats().baro_hold_skips, 1u);
}
