#include "Application/Kalman/kalman_health.hpp"

#include <gtest/gtest.h>

TEST(KalmanHealthStore, ResetSetGet) {
  auto& store = KalmanHealthStore::instance();
  store.reset();

  auto snap = store.get();
  EXPECT_FALSE(snap.diverged);
  EXPECT_EQ(snap.imu_samples_consumed, 0u);
  EXPECT_EQ(snap.imu_ring_hwm[0], 0u);
  EXPECT_EQ(snap.imu_ring_hwm[1], 0u);
  EXPECT_EQ(snap.imu_ring_hwm[2], 0u);
  EXPECT_EQ(snap.last_kalman_loop_us, 0u);
  EXPECT_EQ(snap.max_kalman_loop_us, 0u);
  EXPECT_EQ(snap.last_main_loop_iteration_us, 0u);
  EXPECT_EQ(snap.max_main_loop_iteration_us, 0u);
  EXPECT_EQ(snap.yieldable_imu_drops, 0u);

  KalmanHealthSnapshot update{};
  update.diverged = true;
  update.altitude_valid = true;
  update.velocity_valid = true;
  update.imu_samples_consumed = 77;
  update.baro_updates = 5;
  update.gps_updates = 3;
  update.imu_ring_hwm[0] = 11;
  update.imu_ring_hwm[1] = 7;
  update.imu_ring_hwm[2] = 9;
  update.last_kalman_loop_us = 2100;
  update.max_kalman_loop_us = 2400;
  update.last_main_loop_iteration_us = 2600;
  update.max_main_loop_iteration_us = 3000;
  update.yieldable_imu_drops = 2;

  store.set(update);
  snap = store.get();

  EXPECT_TRUE(snap.diverged);
  EXPECT_TRUE(snap.altitude_valid);
  EXPECT_TRUE(snap.velocity_valid);
  EXPECT_EQ(snap.imu_samples_consumed, 77u);
  EXPECT_EQ(snap.baro_updates, 5u);
  EXPECT_EQ(snap.gps_updates, 3u);
  EXPECT_EQ(snap.imu_ring_hwm[0], 11u);
  EXPECT_EQ(snap.imu_ring_hwm[1], 7u);
  EXPECT_EQ(snap.imu_ring_hwm[2], 9u);
  EXPECT_EQ(snap.last_kalman_loop_us, 2100u);
  EXPECT_EQ(snap.max_kalman_loop_us, 2400u);
  EXPECT_EQ(snap.last_main_loop_iteration_us, 2600u);
  EXPECT_EQ(snap.max_main_loop_iteration_us, 3000u);
  EXPECT_EQ(snap.yieldable_imu_drops, 2u);
}
