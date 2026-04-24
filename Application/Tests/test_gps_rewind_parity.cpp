#include "Application/Kalman/kalman/eskf_tuning_config.hpp"
#include "Application/Kalman/kalman/eskf_yieldable.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace eskf;

// Use UINT32_MAX as budget when tests need catchUp to run to completion
// without yielding. Under Valgrind, wall-clock budgets are easily exceeded.
constexpr uint32_t kNoBudget = UINT32_MAX;

namespace {

State createDefaultState() {
  State s;
  s.setIdentity();
  return s;
}

ImuFrame createImuFrame(uint64_t ts,
                        eskf_scalar ax = 0,
                        eskf_scalar az = -9.80665) {
  ImuFrame imu{};
  imu.accel[0] = ax;
  imu.accel[1] = 0;
  imu.accel[2] = az;
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 0;
  imu.timestamp_us = ts;
  return imu;
}

void initFilterForContracts(EskfYieldable& filter,
                            const State& initial,
                            int32_t gps_delay_us = 0,
                            uint32_t liftoff_rejection_us = 0) {
  TuningConfig tuning = getDefaultTuningConfig();
  tuning.gps_delay_us = gps_delay_us;
  tuning.liftoff_rejection_us = liftoff_rejection_us;
  filter.init(tuning);
  filter.initialize(initial, InitialCovariance::defaults(), ProcessNoise::defaults());

  YieldableConfig cfg;
  cfg.gpsDelayUs = gps_delay_us;
  filter.configure(cfg);
}

void wakeFilterFromHibernation(EskfYieldable& filter,
                               uint64_t liftoff_us = 0,
                               uint64_t rewind_to_ts = 0) {
  LiftoffInitData init_data{};
  init_data.q[0] = 1;
  init_data.q[1] = 0;
  init_data.q[2] = 0;
  init_data.q[3] = 0;
  init_data.b_gyro[0] = 0;
  init_data.b_gyro[1] = 0;
  init_data.b_gyro[2] = 0;
  init_data.heading_variance = 0.01;
  init_data.heading_initialized = false;
  init_data.liftoff_us = liftoff_us;
  init_data.ground_altitude_m = 0;
  init_data.ground_reference_valid = false;

  filter.injectLiftoffSnap(init_data, rewind_to_ts);
  filter.catchUp(rewind_to_ts, kNoBudget);
}

}  // namespace

TEST(KalmanGpsRewindParity, InitializeResetsBuffers) {
  EskfYieldable filter;

  const State initial = createDefaultState();
  filter.initialize(initial, InitialCovariance::defaults(), ProcessNoise::defaults());

  EXPECT_EQ(filter.imuBufferCount(), 0u);
  EXPECT_EQ(filter.baroBufferCount(), 0u);
  EXPECT_EQ(filter.eventBufferCount(), 0u);
  EXPECT_EQ(filter.kalmanTimestamp(), 0u);
}

TEST(KalmanGpsRewindParity, CatchUpProcessesBufferedImu) {
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  filter.initialize(initial, InitialCovariance::defaults(), ProcessNoise::defaults());

  for (int i = 0; i < 10; ++i) {
    const ImuFrame imu = createImuFrame((i + 1) * 1000, 1.0);
    filter.pushImu(imu, 0.001);
  }

#if ESKF_USE_GPS_REWIND
  EXPECT_EQ(filter.kalmanTimestamp(), 0u);
#endif

  const bool caught_up = filter.catchUp(10000, kNoBudget);
  EXPECT_TRUE(caught_up);
  EXPECT_EQ(filter.kalmanTimestamp(), 10000u);
  EXPECT_GT(filter.state().v[0], 0.005);
}

TEST(KalmanGpsRewindParity, BaroAsyncReserveAndSet) {
  EskfYieldable filter;
  const State initial = createDefaultState();
  filter.initialize(initial, InitialCovariance::defaults(), ProcessNoise::defaults());

  const size_t slot = filter.reserveBaro(5000);
#if ESKF_USE_GPS_REWIND
  EXPECT_EQ(filter.baroBufferCount(), 1u);
#endif
  filter.setBaroMeasurement(slot, 100.0, 1.0);

  const ImuFrame imu = createImuFrame(5000);
  filter.pushImu(imu, 0.001);
  (void)filter.catchUp(5000, kNoBudget);
}

TEST(KalmanGpsRewindParity, BaroOnTimeReplayUsesDirectCorrection) {
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  State s = filter.state();
  s.p[2] = 200.0;
  filter.core().setState(s);
  filter.triggerBaro(5000);

  s = filter.state();
  s.p[2] = 0.0;
  filter.core().setState(s);

  filter.completeBaro(0.0);
  filter.catchUp(5000, kNoBudget);

  EXPECT_LT(std::abs(filter.state().p[2]), 10.0);
}

TEST(KalmanGpsRewindParity, BaroLateCompletionUsesInnovationTransport) {
  EskfYieldable filter_a;
  EskfYieldable filter_b;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter_a, initial, 0, 0);
  initFilterForContracts(filter_b, initial, 0, 0);
  wakeFilterFromHibernation(filter_a, 0, 0);
  wakeFilterFromHibernation(filter_b, 0, 0);

  for (int i = 0; i < 10; ++i) {
    const ImuFrame imu = createImuFrame((i + 1) * 1000ULL);
    filter_a.pushImu(imu, 0.001);
    filter_b.pushImu(imu, 0.001);
  }
  filter_a.catchUp(10000, kNoBudget);
  filter_b.catchUp(10000, kNoBudget);

  filter_a.core().resetPositionCovariance(1000.0);
  filter_b.core().resetPositionCovariance(1000.0);

  State sa = filter_a.state();
  sa.p[2] = 200.0;
  filter_a.core().setState(sa);
  const size_t slot_a = filter_a.reserveBaro(5000);

  State sb = filter_b.state();
  sb.p[2] = -200.0;
  filter_b.core().setState(sb);
  const size_t slot_b = filter_b.reserveBaro(5000);

  sa = filter_a.state();
  sa.p[2] = 0.0;
  filter_a.core().setState(sa);
  sb = filter_b.state();
  sb.p[2] = 0.0;
  filter_b.core().setState(sb);

  filter_a.setBaroMeasurement(slot_a, 0.0, 0.01);
  filter_b.setBaroMeasurement(slot_b, 0.0, 0.01);
  filter_a.catchUp(11000, kNoBudget);
  filter_b.catchUp(11000, kNoBudget);

  const eskf_scalar delta_pz = std::abs(filter_a.state().p[2] - filter_b.state().p[2]);
  EXPECT_GT(delta_pz, 10.0);
}

TEST(KalmanGpsRewindParity, IsBehindStatus) {
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  filter.initialize(initial, InitialCovariance::defaults(), ProcessNoise::defaults());

  EXPECT_TRUE(filter.isBehind(1000));

  const ImuFrame imu = createImuFrame(1000);
  filter.pushImu(imu, 0.001);
  filter.catchUp(1000, kNoBudget);

  EXPECT_FALSE(filter.isBehind(1000));
}
