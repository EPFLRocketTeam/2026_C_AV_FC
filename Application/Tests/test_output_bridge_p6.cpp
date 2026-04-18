#include "Application/Kalman/AppLayer/output_bridge.hpp"

#include <gtest/gtest.h>

TEST(P6OutputBridge, MapsEstimatorOutputToNavigationWithSignConvention) {
  app::EstimatorOutput out{};
  out.position_ned[0] = 10.0f;
  out.position_ned[1] = -5.0f;
  out.position_ned[2] = 123.0f;
  out.velocity_ned[0] = 1.0f;
  out.velocity_ned[1] = 2.0f;
  out.velocity_ned[2] = 3.0f;
  out.quaternion[0] = 1.0f;
  out.quaternion[1] = 0.0f;
  out.quaternion[2] = 0.0f;
  out.quaternion[3] = 0.0f;
  out.altitude_m = -123.0f;

  flight_computer::bmp3_data baro{};
  baro.pressure = 101325.0;
  baro.temperature = 20.0;

  flight_computer::Vector3 accel{};
  accel.x = 0.1;
  accel.y = 0.2;
  accel.z = 0.3;

  const auto nav = app::mapEstimatorToNavigation(out, baro, accel);

  EXPECT_NEAR(nav.position_kalman.x, 10.0, 1e-9);
  EXPECT_NEAR(nav.position_kalman.y, -5.0, 1e-9);
  EXPECT_NEAR(nav.position_kalman.z, 123.0, 1e-9);
  EXPECT_NEAR(nav.speed.z, 3.0, 1e-9);
  EXPECT_NEAR(nav.altitude, -123.0, 1e-9);
  EXPECT_NEAR(nav.course, 0.0, 1e-6);
  EXPECT_NEAR(nav.accel.x, 0.1, 1e-9);
  EXPECT_NEAR(nav.attitude.x, 0.0, 1e-6);
  EXPECT_NEAR(nav.attitude.y, 0.0, 1e-6);
  EXPECT_NEAR(nav.attitude.z, 0.0, 1e-6);
}

TEST(P6OutputBridge, BuildsApogeeInputContract) {
  app::EstimatorOutput out{};
  out.altitude_m = 800.0f;
  out.velocity_ned[2] = 12.5f;
  out.shadow_velocity_down_mps = 11.0f;
  out.eskf_valid = true;
  out.shadow_valid = true;
  out.body_accel_x_mps2 = -4.0f;

  const auto input = app::buildApogeeInput(
      out,
      false,
      flight_computer::State::ASCENT,
      1000,
      1600);

  EXPECT_NEAR(input.altitude_m, 800.0f, 1e-6);
  EXPECT_NEAR(input.eskf_velocity_down_mps, 12.5f, 1e-6);
  EXPECT_NEAR(input.shadow_velocity_down_mps, 11.0f, 1e-6);
  EXPECT_TRUE(input.eskf_valid);
  EXPECT_TRUE(input.shadow_valid);
  EXPECT_FALSE(input.eskf_diverged);
  EXPECT_TRUE(input.is_coast_phase);
  EXPECT_EQ(input.time_since_liftoff_ms, 600u);
  EXPECT_NEAR(input.body_accel_x_mps2, -4.0f, 1e-6);
}
