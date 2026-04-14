#include "Application/Kalman/kalman_health.hpp"

#include <gtest/gtest.h>

TEST(KalmanHealthStore, ResetSetGet) {
  auto& store = KalmanHealthStore::instance();
  store.reset();

  auto snap = store.get();
  EXPECT_FALSE(snap.diverged);
  EXPECT_EQ(snap.imu_samples_consumed, 0u);

  KalmanHealthSnapshot update{};
  update.diverged = true;
  update.altitude_valid = true;
  update.velocity_valid = true;
  update.imu_samples_consumed = 77;
  update.baro_updates = 5;
  update.gps_updates = 3;

  store.set(update);
  snap = store.get();

  EXPECT_TRUE(snap.diverged);
  EXPECT_TRUE(snap.altitude_valid);
  EXPECT_TRUE(snap.velocity_valid);
  EXPECT_EQ(snap.imu_samples_consumed, 77u);
  EXPECT_EQ(snap.baro_updates, 5u);
  EXPECT_EQ(snap.gps_updates, 3u);
}
