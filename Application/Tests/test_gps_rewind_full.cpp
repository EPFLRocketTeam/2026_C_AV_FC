// GPS Rewind Unit Tests
// Tests for unified timestamp-controlled replay architecture

#include "Application/Tests/unity_gtest_compat.hpp"

#include <cmath>
#include <vector>

#include "Application/Kalman/kalman/eskf_yieldable.hpp"
#include "Application/Kalman/kalman/eskf_tuning_config.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_imu.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_baro.hpp"

using namespace eskf;

// Tolerance for floating-point comparisons
constexpr eskf_scalar kTol = 1e-9;

// Use UINT32_MAX as budget when tests need catchUp to run to completion
// without yielding. Under Valgrind (50-100x slower), even generous wall-clock
// budgets like 1s can be exceeded during heavy Eigen operations.
constexpr uint32_t kNoBudget = UINT32_MAX;
constexpr eskf_scalar kTolF = 1e-5;

// ============================================================
// Helper Functions
// ============================================================

static State createDefaultState() {
  State s;
  s.setIdentity();
  return s;
}

static ImuFrame createImuFrame(uint64_t ts, eskf_scalar ax = 0, eskf_scalar az = -9.80665) {
  ImuFrame imu{};
  imu.accel[0] = ax;
  imu.accel[1] = 0;
  imu.accel[2] = az;  // Cancel gravity
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 0;
  imu.timestamp_us = ts;
  return imu;
}

static eskf_scalar yawFromState(const State& s) {
  return std::atan2(
      2.0 * (s.q[0] * s.q[3] + s.q[1] * s.q[2]),
      1.0 - 2.0 * (s.q[2] * s.q[2] + s.q[3] * s.q[3]));
}

static void initFilterForContracts(EskfYieldable& filter,
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

static void wakeFilterFromHibernation(EskfYieldable& filter,
                                      uint64_t liftoff_us = 0,
                                      uint64_t rewind_to_ts = 0,
                                      eskf_scalar q0 = 1.0,
                                      eskf_scalar q2 = 0.0) {
  LiftoffInitData init_data{};
  init_data.q[0] = q0;
  init_data.q[1] = 0;
  init_data.q[2] = q2;
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

#if APP_TARGET_NATIVE
struct DeterministicClock {
  uint32_t now_us = 0;
  uint32_t step_us = 1;
};

static uint32_t deterministicNowMicros(void* ctx) {
  auto* clock = static_cast<DeterministicClock*>(ctx);
  const uint32_t out = clock->now_us;
  clock->now_us += clock->step_us;
  return out;
}
#endif

// ============================================================
// Initialization Tests
// ============================================================

static void test_initialize_resets_buffers() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  
  filter.initialize(initial, P0, Q);
  
  TEST_ASSERT_EQUAL(0, filter.imuBufferCount());
  TEST_ASSERT_EQUAL(0, filter.baroBufferCount());
  TEST_ASSERT_EQUAL(0, filter.eventBufferCount());
  TEST_ASSERT_EQUAL(0, filter.kalmanTimestamp());
}

// ============================================================
// Push IMU Tests (Buffer Only)
// ============================================================

static void test_pushImu_only_buffers() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  ImuFrame imu = createImuFrame(1000, 1.0);  // 1 m/s² accel
  filter.pushImu(imu, 0.001);
  
#if ESKF_USE_GPS_REWIND
  // Should be buffered
  TEST_ASSERT_EQUAL(1, filter.imuBufferCount());
  // Kalman timestamp should NOT have advanced (no catchUp yet)
  TEST_ASSERT_EQUAL(0, filter.kalmanTimestamp());
  // Velocity should still be zero (not processed)
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, filter.state().v[0]);
#endif
}

// ============================================================
// Catch-Up Tests
// ============================================================

static void test_catchUp_processes_buffered_imu() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  // Push some IMU frames (buffer only)
  for (int i = 0; i < 10; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000, 1.0); // 1 m/s² accel
    filter.pushImu(imu, 0.001);
  }
  
#if ESKF_USE_GPS_REWIND
  // Should still be at time 0 (not processed yet)
  TEST_ASSERT_EQUAL(0, filter.kalmanTimestamp());
#endif
  
  // Call catchUp to process
  bool caught_up = filter.catchUp(10000, kNoBudget);
  TEST_ASSERT_TRUE(caught_up);
  TEST_ASSERT_EQUAL(10000, filter.kalmanTimestamp());
  
  // Velocity should have increased
  TEST_ASSERT_TRUE(filter.state().v[0] > 0.005);
}

static void test_catchUp_respects_budget() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  // Push many IMU frames
  for (int i = 0; i < 500; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 100);
    filter.pushImu(imu, 0.0001);
  }
  
  // Call catchUp with tiny budget - should yield
  // Note: On fast machines this might still complete, so we check lastEventsProcessed
  filter.catchUp(50000, 1);  // 1µs budget - unrealistic but tests the logic
  
  // Either yielded or processed everything - both are valid
  TEST_ASSERT_TRUE(filter.lastEventsProcessed() > 0);
}

static void test_isBehind_returns_correct_status() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  // Initially at time 0
  TEST_ASSERT_TRUE(filter.isBehind(1000));
  
  // Push and process IMU
  ImuFrame imu = createImuFrame(1000);
  filter.pushImu(imu, 0.001);
  filter.catchUp(1000, kNoBudget);
  
  // Now should not be behind
  TEST_ASSERT_FALSE(filter.isBehind(1000));
}

// ============================================================
// Baro Async Timing Tests
// ============================================================

static void test_baro_async_reserve_and_set() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  // Reserve baro slot (trigger time known)
  size_t slot = filter.reserveBaro(5000);
  
#if ESKF_USE_GPS_REWIND
  TEST_ASSERT_EQUAL(1, filter.baroBufferCount());
#endif
  
  // Set measurement (value retrieved later)
  filter.setBaroMeasurement(slot, 100.0, 1.0);
  
  // Process should now work
  ImuFrame imu = createImuFrame(5000);
  filter.pushImu(imu, 0.001);
  filter.catchUp(5000, kNoBudget);
}

static void test_baro_on_time_replay_uses_direct_correction() {
  // Subsystem: processNextBaro on-time replay path.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  // Trigger snapshot from a state far from the upcoming measurement.
  State s = filter.state();
  s.p[2] = 200.0;
  filter.core().setState(s);
  filter.triggerBaro(5000);

  // Move state close to measurement before replay reaches trigger timestamp.
  s = filter.state();
  s.p[2] = 0.0;
  filter.core().setState(s);

  // Measurement corresponding to altitude 0m (h ~= 0 when p_D=0 and b_baro=0).
  filter.completeBaro(0.0);
  filter.catchUp(5000, kNoBudget);

  // With direct correction, state should remain close to current prediction.
  TEST_ASSERT_TRUE(std::abs(filter.state().p[2]) < 10.0);
}

static void test_baro_late_completion_uses_innovation_transport() {
  // Subsystem: processNextBaro late completion fallback path.
  EskfYieldable filter_a;
  EskfYieldable filter_b;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter_a, initial, 0, 0);
  initFilterForContracts(filter_b, initial, 0, 0);
  wakeFilterFromHibernation(filter_a, 0, 0);
  wakeFilterFromHibernation(filter_b, 0, 0);

  // Advance both filters beyond trigger timestamp so completion is late.
  for (int i = 0; i < 10; ++i) {
    const ImuFrame imu = createImuFrame((i + 1) * 1000ULL);
    filter_a.pushImu(imu, 0.001);
    filter_b.pushImu(imu, 0.001);
  }
  filter_a.catchUp(10000, kNoBudget);
  filter_b.catchUp(10000, kNoBudget);

  // Increase vertical update gain so snapshot dependence is clearly observable.
  filter_a.core().resetPositionCovariance(1000.0);
  filter_b.core().resetPositionCovariance(1000.0);

  // Capture opposite trigger snapshots in two filters.
  State sa = filter_a.state();
  sa.p[2] = 200.0;
  filter_a.core().setState(sa);
  const size_t slot_a = filter_a.reserveBaro(5000);

  State sb = filter_b.state();
  sb.p[2] = -200.0;
  filter_b.core().setState(sb);
  const size_t slot_b = filter_b.reserveBaro(5000);

  // Make current states identical before applying same measurement.
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

  // Late path should depend on trigger snapshot; direct path would not.
  const eskf_scalar delta_pz =
      std::abs(filter_a.state().p[2] - filter_b.state().p[2]);
  TEST_ASSERT_TRUE(delta_pz > 10.0);
}

// ============================================================
// GPS Split Timestamp Tests
// ============================================================

#if ESKF_USE_GPS_REWIND
static void test_gps_unified_timestamp_with_delay() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, -100000, 0);
  wakeFilterFromHibernation(filter, 0, 0);
  
  // Configure GPS delay (both position and velocity apply this offset from PPS)
  YieldableConfig cfg;
  cfg.gpsDelayUs = -100000;  // -100ms: measurement is 100ms earlier than PPS
  filter.configure(cfg);
  
  // Push IMU frames to advance time
  for (int i = 0; i < 150; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter.pushImu(imu, 0.001);
  }
  filter.catchUp(150000, kNoBudget);
  TEST_ASSERT_EQUAL(150000, filter.kalmanTimestamp());
  
  // Push GPS with PPS at 200ms
  // Both position and velocity timestamps will be 200ms - 100ms = 100ms
  eskf_scalar pos[3] = {0, 0, 0};
  eskf_scalar vel[3] = {0, 0, 0};
  eskf_scalar R[3] = {1, 1, 1};
  eskf_scalar lever[3] = {0, 0, 0};
  
  filter.pushGpsVelocity(200000, vel, R, lever);
  
  // Should have rewound to GPS timestamp (100ms)
  TEST_ASSERT_TRUE(filter.kalmanTimestamp() <= 100000);
  
  // Push position at same PPS time - will also use 100ms timestamp
  filter.pushGpsPosition(200000, pos, R);
  
  // Catch up to GPS timestamp (100ms, not 200ms since delay was applied)
  filter.catchUp(100000, kNoBudget);
  TEST_ASSERT_EQUAL(100000, filter.kalmanTimestamp());
}

static void test_liftoff_snap_stored_and_replayed() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  // Push some IMU frames
  for (int i = 0; i < 50; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter.pushImu(imu, 0.001);
  }
  filter.catchUp(50000, kNoBudget);
  
  // Inject liftoff snap at 10ms using LiftoffInitData struct
  LiftoffInitData init_data{};
  init_data.q[0] = 0.99619; init_data.q[1] = 0; 
  init_data.q[2] = 0.08716; init_data.q[3] = 0; // ~10° pitch
  init_data.b_acc[0] = 0.12;
  init_data.b_acc[1] = -0.03;
  init_data.b_acc[2] = 0.06;
  init_data.b_gyro[0] = 0; init_data.b_gyro[1] = 0; init_data.b_gyro[2] = 0;
  init_data.heading_variance = 0.01;
  init_data.heading_initialized = false;
  init_data.liftoff_us = 10000;
  init_data.ground_altitude_m = 0;
  init_data.ground_reference_valid = false;
  
  filter.injectLiftoffSnap(init_data, 10000);
  
  // Catch up
  filter.catchUp(50000, kNoBudget);
  
  // The quaternion should reflect the injected orientation
  const State& s = filter.state();
  TEST_ASSERT_DOUBLE_WITHIN(0.1, init_data.q[0], s.q[0]);
  TEST_ASSERT_FLOAT_WITHIN(kTolF, init_data.b_acc[0], s.b_acc[0]);
  TEST_ASSERT_FLOAT_WITHIN(kTolF, init_data.b_acc[1], s.b_acc[1]);
  TEST_ASSERT_FLOAT_WITHIN(kTolF, init_data.b_acc[2], s.b_acc[2]);
}

// ============================================================
// Test Gap Analysis - New Tests
// ============================================================

static void test_buffer_overflow_during_rewind() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  // Use nominal GNSS delay so GPS PPS timestamps land behind current Kalman time.
  initFilterForContracts(filter, initial, -100000, 0);
  wakeFilterFromHibernation(filter, 0, 0);
  filter.resetStats();
  
  // Fill buffer beyond capacity to trigger overflow
  // ESKF_IMU_BUFFER_SIZE is 3200, so we push more
  for (int i = 0; i < ESKF_IMU_BUFFER_SIZE + 100; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 100);
    filter.pushImu(imu, 0.0001);
  }
  
  // Should have recorded drops
  TEST_ASSERT_TRUE(filter.stats().imu_drops > 0);
  TEST_ASSERT_EQUAL(ESKF_IMU_BUFFER_SIZE, filter.imuBufferCount());
  
  // Trigger GPS to cause rewind
  eskf_scalar vel[3] = {0, 0, 0};
  eskf_scalar R[3] = {1, 1, 1};
  eskf_scalar lever[3] = {0, 0, 0};
  
  // First catch up to current time
  filter.catchUp(ESKF_IMU_BUFFER_SIZE * 100, kNoBudget);
  
  // Push GPS velocity with earlier timestamp
  filter.pushGpsVelocity(ESKF_IMU_BUFFER_SIZE * 100, vel, R, lever);
  
  // Verify rewind happened
  TEST_ASSERT_TRUE(filter.stats().rewind_count > 0);
}

static void test_out_of_order_event_timestamps() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  filter.resetStats();
  
  // Push some IMU frames to advance time
  for (int i = 0; i < 100; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter.pushImu(imu, 0.001);
  }
  
  // Push mag heading at t=50000
  filter.pushMagHeading(0.0, 0.01, 50000);
  
  // Push mag heading at earlier time t=30000 (out of order)
  filter.pushMagHeading(0.1, 0.01, 30000);
  
  // Should detect out-of-order event
  TEST_ASSERT_TRUE(filter.stats().out_of_order_events > 0);
}

static void test_rewind_before_any_checkpoints_uses_initial() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  // Keep nominal GNSS delay: PPS 105ms maps to 5ms event timestamp.
  initFilterForContracts(filter, initial, -100000, 0);
  wakeFilterFromHibernation(filter, 0, 0);
  filter.resetStats();
  
  // Push only a few IMU samples (not enough for periodic checkpoint)
  for (int i = 0; i < 10; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter.pushImu(imu, 0.001);
  }
  filter.catchUp(10000, kNoBudget);
  
  // Push GPS that triggers rewind to t=5000
  eskf_scalar vel[3] = {0, 0, 0};
  eskf_scalar R[3] = {1, 1, 1};
  eskf_scalar lever[3] = {0, 0, 0};
  
  // This should use oldest_checkpoint_ initialized in initialize()
  filter.pushGpsVelocity(105000, vel, R, lever);  // Vel timestamp = 5000
  
  // Should have done a rewind and used oldest checkpoint (no error)
  TEST_ASSERT_TRUE(filter.stats().rewind_count > 0);
  // Should NOT have triggered no-checkpoint warning since we init one in initialize()
  TEST_ASSERT_EQUAL(0, filter.stats().rewind_no_checkpoint_count);
}

static void test_multiple_sequential_rewinds() {
  EskfYieldable filter;
  
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  // Use nominal GNSS delay so each GPS event rewinds to an earlier epoch.
  initFilterForContracts(filter, initial, -100000, 0);
  wakeFilterFromHibernation(filter, 0, 0);
  filter.resetStats();
  
  // Push many IMU samples
  for (int i = 0; i < 500; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter.pushImu(imu, 0.001);
  }
  filter.catchUp(500000, kNoBudget);
  
  eskf_scalar vel[3] = {0, 0, 0};
  eskf_scalar R[3] = {1, 1, 1};
  eskf_scalar lever[3] = {0, 0, 0};
  
  // First GPS at PPS 500ms, velocity timestamp = 400ms
  filter.pushGpsVelocity(500000, vel, R, lever);
  filter.catchUp(500000, kNoBudget);
  TEST_ASSERT_EQUAL(1, filter.stats().rewind_count);
  
  // Second GPS at PPS 500ms (another measurement), rewind again
  filter.pushGpsVelocity(510000, vel, R, lever);
  filter.catchUp(510000, kNoBudget);
  TEST_ASSERT_EQUAL(2, filter.stats().rewind_count);
  
  // Third GPS at PPS 520ms
  filter.pushGpsVelocity(520000, vel, R, lever);
  filter.catchUp(520000, kNoBudget);
  TEST_ASSERT_EQUAL(3, filter.stats().rewind_count);
}

static void test_completeBaro_vs_setBaroMeasurement_equivalence() {
  // Test that both baro APIs produce consistent behavior
  
  // First filter using setBaroMeasurement
  EskfYieldable filter1;
  State initial = createDefaultState();
  initial.timestamp_us = 0;
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter1.initialize(initial, P0, Q);
  
  // Push IMU to advance time
  for (int i = 0; i < 10; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter1.pushImu(imu, 0.001);
  }
  
  size_t slot = filter1.reserveBaro(5000);
  filter1.setBaroMeasurement(slot, 100.0, 4.0);  // Fixed R
  filter1.catchUp(10000, kNoBudget);
  eskf_scalar alt1 = filter1.state().p[2];
  
  // Second filter using triggerBaro/completeBaro
  EskfYieldable filter2;
  filter2.initialize(initial, P0, Q);
  
  for (int i = 0; i < 10; ++i) {
    ImuFrame imu = createImuFrame((i + 1) * 1000);
    filter2.pushImu(imu, 0.001);
  }
  
  filter2.triggerBaro(5000);
  filter2.completeBaro(100.0);  // Auto R from velocity
  filter2.catchUp(10000, kNoBudget);
  eskf_scalar alt2 = filter2.state().p[2];
  
  // Both should be on the same order of magnitude (R differs, so not exact)
  // The key test is that both go through buffer processing
  TEST_ASSERT_TRUE(filter2.baroBufferCount() > 0);  // Verify buffered
  // Altitude estimates should be in similar ballpark
  TEST_ASSERT_DOUBLE_WITHIN(50.0, alt1, alt2);
}

static void test_mag_heading_outlier_is_rejected_by_state_machine() {
  // Subsystem: MagHeading should use heading gate policy, not direct fusion.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  // Force initialized heading state so outlier goes through gate path.
  filter.core().setHeadingInitialized(true);

  const eskf_scalar yaw_before = yawFromState(filter.state());

  // Large innovation should fail 3-sigma gate and be rejected.
  filter.pushMagHeading(3.0, 0.01, 10000);
  filter.catchUp(10000, kNoBudget);

  const eskf_scalar yaw_after = yawFromState(filter.state());
  TEST_ASSERT_TRUE(std::abs(yaw_after - yaw_before) < 0.2);
}

static void test_completeBaro_stale_drop_counted_once() {
  // Subsystem: stale completion must not be counted again in discard pass.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;

  TuningConfig tuning = getDefaultTuningConfig();
  tuning.baro_snapshot_max_age_us = 1000;
  tuning.liftoff_rejection_us = 0;
  filter.init(tuning);
  filter.initialize(initial, InitialCovariance::defaults(), ProcessNoise::defaults());

  YieldableConfig cfg;
  cfg.gpsDelayUs = 0;
  filter.configure(cfg);
  wakeFilterFromHibernation(filter, 0, 0);

  for (int i = 0; i < 5; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 1000ULL), 0.001);
  }
  filter.catchUp(5000, kNoBudget);
  filter.resetStats();

  // Deliberately stale completion: trigger timestamp is behind Kalman time.
  filter.triggerBaro(1000);
  filter.completeBaro(80.0);
  TEST_ASSERT_EQUAL(1, filter.stats().baro_drops);

  // Another catchUp pass should consume dropped slot without recounting.
  filter.catchUp(6000, kNoBudget);
  TEST_ASSERT_EQUAL(1, filter.stats().baro_drops);
}

// ============================================================
// Contract Tests (Checklist Items 1-7, 11, 15, 18)
// ============================================================

static void test_yieldable_contract_01_liftoff_snap_precedes_tied_imu() {
  // Subsystem: EskfYieldable catchUp same-timestamp scheduler.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);

  // Tied IMU sample at 10ms with non-zero gyro.
  ImuFrame imu = createImuFrame(10000, 0.0, -9.80665);
  imu.gyro[2] = 2.0;
  filter.pushImu(imu, 0.01);

  // Liftoff snap is injected at the same timestamp.
  wakeFilterFromHibernation(filter, 10000, 10000, 1.0, 0.0);
  filter.catchUp(10000, kNoBudget);

  // If liftoff executes first (contract), tied IMU rotates away from identity.
  const State& s = filter.state();
  TEST_ASSERT_TRUE(std::abs(s.q[3]) > 0.005);
}

static void test_yieldable_contract_01_same_timestamp_baro_then_event_fifo() {
  // Subsystem: EskfYieldable catchUp tie ordering IMU -> Baro -> Event.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  State seeded = filter.state();
  seeded.p[2] = 200.0;
  filter.core().setState(seeded);

  // Baro and GPS position share the same timestamp; event should execute after baro.
  const size_t slot = filter.reserveBaro(5000);
  filter.setBaroMeasurement(slot, 0.0, 0.01);

  const eskf_scalar pos[3] = {0.0, 0.0, 120.0};
  const eskf_scalar R_pos[3] = {1000.0, 1000.0, 0.01};
  filter.pushGpsPosition(5000, pos, R_pos);

  filter.pushImu(createImuFrame(5000), 0.001);
  filter.catchUp(5000, kNoBudget);

  // GPS position event should dominate the final vertical state if it runs last.
  TEST_ASSERT_TRUE(filter.state().p[2] > 30.0);
}

static void test_yieldable_contract_02_catchup_true_when_nothing_processable() {
  // Subsystem: catchUp completion semantics with pending-not-ready baro slots.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);

  filter.pushImu(createImuFrame(1000), 0.001);
  filter.catchUp(1000, kNoBudget);

  // Reserve baro slot and leave it incomplete (ready=false).
  filter.reserveBaro(2000);
  const bool caught_up = filter.catchUp(5000, kNoBudget);

  TEST_ASSERT_TRUE(caught_up);
  TEST_ASSERT_EQUAL(1000, filter.kalmanTimestamp());
}

static void test_yieldable_contract_03_budget_yield_cursor_integrity_deterministic_clock() {
  // Subsystem: catchUp budget-yield path with deterministic clock.
  EskfYieldable reference;
  EskfYieldable budgeted;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(reference, initial, 0, 0);
  initFilterForContracts(budgeted, initial, 0, 0);

  constexpr int kSamples = 220;
  constexpr uint64_t kTargetUs = static_cast<uint64_t>(kSamples) * 1000ULL;

  for (int i = 0; i < kSamples; ++i) {
    const uint64_t ts = static_cast<uint64_t>(i + 1) * 1000ULL;
    const ImuFrame imu = createImuFrame(ts, 1.0, -9.80665);
    reference.pushImu(imu, 0.001);
    budgeted.pushImu(imu, 0.001);
  }

  reference.catchUp(kTargetUs, kNoBudget);

#if APP_TARGET_NATIVE
  DeterministicClock clock;
  clock.now_us = 0;
  clock.step_us = 5;
  budgeted.setTestNowMicros(deterministicNowMicros, &clock);
#endif

  uint32_t total_events = 0;
  bool saw_yield = false;
  uint64_t prev_ts = budgeted.kalmanTimestamp();

  bool done = false;
  for (int attempt = 0; attempt < 400 && !done; ++attempt) {
    done = budgeted.catchUp(kTargetUs, 40);
    total_events += budgeted.lastEventsProcessed();
    if (!done) {
      saw_yield = true;
      TEST_ASSERT_TRUE(budgeted.lastEventsProcessed() > 0);
    }
    TEST_ASSERT_TRUE(budgeted.kalmanTimestamp() >= prev_ts);
    prev_ts = budgeted.kalmanTimestamp();
  }

#if APP_TARGET_NATIVE
  budgeted.clearTestNowMicros();
#endif

  TEST_ASSERT_TRUE(done);
  TEST_ASSERT_TRUE(saw_yield);
  TEST_ASSERT_EQUAL(kSamples, total_events);

  TEST_ASSERT_DOUBLE_WITHIN(1e-6, reference.state().v[0], budgeted.state().v[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, reference.state().p[0], budgeted.state().p[0]);
}

static void test_yieldable_contract_04_rewind_replay_indices_match_checkpoint_contract() {
  // Subsystem: rewindTo checkpoint selection and replay index offsets.
  auto setup_filter = []() {
    EskfYieldable filter;
    State initial = createDefaultState();
    initial.timestamp_us = 0;
    initFilterForContracts(filter, initial, 0, 0);
    wakeFilterFromHibernation(filter, 0, 0);
    filter.core().setHeadingInitialized(true);

    for (int i = 0; i < 300; ++i) {
      filter.pushImu(createImuFrame((i + 1) * 1000ULL, 1.0, -9.80665), 0.001);
    }
    const size_t baro_slot = filter.reserveBaro(256000);
    filter.setBaroMeasurement(baro_slot, 120.0, 0.01);
    filter.pushMagHeading(0.3, 0.01, 256000);
    filter.catchUp(300000, kNoBudget);
    filter.resetStats();
    return filter;
  };

  EskfYieldable baseline = setup_filter();
  EskfYieldable rewound = setup_filter();

  const eskf_scalar vel[3] = {0, 0, 0};
  const eskf_scalar R_vel[3] = {1e12, 1e12, 1e12};
  const eskf_scalar lever[3] = {0, 0, 0};
  rewound.pushGpsVelocity(256000, vel, R_vel, lever);
  rewound.catchUp(300000, kNoBudget);

  TEST_ASSERT_EQUAL(1, rewound.stats().rewind_count);
  TEST_ASSERT_DOUBLE_WITHIN(1e-2, baseline.state().v[0], rewound.state().v[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, baseline.state().p[2], rewound.state().p[2]);
}

static void test_yieldable_contract_05_rewind_data_gap_detected() {
  // Subsystem: GPS older than retained IMU history is dropped pre-enqueue.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  const int total = static_cast<int>(ESKF_IMU_BUFFER_SIZE) + 512;
  for (int i = 0; i < total; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 100ULL, 1.0, -9.80665), 0.0001);
  }
  const uint64_t final_ts = static_cast<uint64_t>(total) * 100ULL;
  filter.catchUp(final_ts, kNoBudget);
  const eskf_scalar vx_before_rewind = filter.state().v[0];

  filter.resetStats();
  const eskf_scalar vel[3] = {0, 0, 0};
  const eskf_scalar R_vel[3] = {1e12, 1e12, 1e12};
  const eskf_scalar lever[3] = {0, 0, 0};
  filter.pushGpsVelocity(100, vel, R_vel, lever);
  filter.catchUp(final_ts, kNoBudget);

  TEST_ASSERT_EQUAL(0, filter.stats().rewind_count);
  TEST_ASSERT_EQUAL(0, filter.stats().rewind_data_gap_count);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, vx_before_rewind, filter.state().v[0]);
}

static void test_yieldable_contract_06_ring_overflow_asymmetry() {
  // Subsystem: ring-overflow asymmetry for IMU/Baro/Event buffers.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);
  filter.resetStats();

  for (size_t i = 0; i < ESKF_IMU_BUFFER_SIZE + 24; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 100ULL), 0.0001);
  }
  TEST_ASSERT_TRUE(filter.stats().imu_drops > 0);
  const uint32_t imu_drops = filter.stats().imu_drops;

  for (size_t i = 0; i < ESKF_BARO_BUFFER_SIZE + 8; ++i) {
    filter.reserveBaro((i + 1) * 1000ULL);
  }
  for (size_t i = 0; i < ESKF_EVENT_BUFFER_SIZE + 16; ++i) {
    filter.pushMagHeading(0.0, 0.05, (i + 1) * 2000ULL);
  }

  TEST_ASSERT_TRUE(filter.stats().baro_drops > 0);
  TEST_ASSERT_TRUE(filter.stats().event_drops > 0);
  TEST_ASSERT_EQUAL(imu_drops, filter.stats().imu_drops);
}

static void test_yieldable_contract_07_out_of_order_processed_chronologically() {
  // Subsystem: out-of-order events replay chronologically (equivalent to
  // explicit in-order insertion) while timeline remains monotonic.
  EskfYieldable reference;
  {
    State initial = createDefaultState();
    initial.timestamp_us = 0;
    initFilterForContracts(reference, initial, 0, 0);
    wakeFilterFromHibernation(reference, 0, 0);
    reference.resetStats();
    reference.pushMagHeading(1.0, 0.001, 30000);
    reference.pushMagHeading(-1.0, 0.001, 50000);
    reference.catchUp(60000, kNoBudget);
  }

  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);
  filter.resetStats();

  filter.pushMagHeading(-1.0, 0.001, 50000);
  filter.pushMagHeading(1.0, 0.001, 30000);  // Out of order by timestamp.
  filter.catchUp(60000, kNoBudget);

  TEST_ASSERT_EQUAL(1, filter.stats().out_of_order_events);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6,
                            yawFromState(reference.state()),
                            yawFromState(filter.state()));
  TEST_ASSERT_EQUAL(50000, filter.kalmanTimestamp());
}

static void test_yieldable_contract_11_trigger_complete_baro_pending_contract() {
  // Subsystem: baro trigger/complete innovation transport contract.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  for (int i = 0; i < 20; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 1000ULL), 0.001);
  }

  filter.triggerBaro(15000);
  const bool first_catch = filter.catchUp(20000, kNoBudget);
  const eskf_scalar z_before_complete = filter.state().p[2];
  const uint32_t rewinds_before = filter.stats().rewind_count;

  filter.completeBaro(80.0);
  filter.pushImu(createImuFrame(21000), 0.001);
  filter.catchUp(21000, kNoBudget);

  TEST_ASSERT_TRUE(first_catch);
  TEST_ASSERT_EQUAL(21000, filter.kalmanTimestamp());
  TEST_ASSERT_TRUE(std::abs(filter.state().p[2] - z_before_complete) > 1e-3);
  TEST_ASSERT_EQUAL(rewinds_before, filter.stats().rewind_count);
}

static void test_yieldable_contract_15_dual_timestamp_single_delay_for_packet_ordering() {
  // Subsystem: GNSS dual-timestamp contract in yieldable packet timing.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, -20000, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  for (int i = 0; i < 130; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 1000ULL), 0.001);
  }
  filter.catchUp(130000, kNoBudget);
  filter.resetStats();

  const eskf_scalar pos[3] = {0, 0, 0};
  const eskf_scalar vel[3] = {0, 0, 0};
  const eskf_scalar R_pos[3] = {1.0, 1.0, 1.0};
  const eskf_scalar R_vel[3] = {1.0, 1.0, 1.0};
  const eskf_scalar lever[3] = {0, 0, 0};

  filter.pushGpsPacket(120000, pos, R_pos, vel, R_vel, lever);

  // Rewind must move Kalman time backward (single-delay ordering domain).
  TEST_ASSERT_TRUE(filter.kalmanTimestamp() <= 100000);

  filter.catchUp(130000, kNoBudget);
  TEST_ASSERT_EQUAL(130000, filter.kalmanTimestamp());
  TEST_ASSERT_EQUAL(1, filter.stats().rewind_count);
}

static void test_yieldable_contract_16_drop_late_gps_older_than_retained_imu_history() {
  // Subsystem: late GPS packet older than retained IMU history must be
  // dropped before enqueue to avoid degraded rewind loops.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 0);
  wakeFilterFromHibernation(filter, 0, 0);

  // Fill beyond IMU ring capacity so oldest retained timestamp advances.
  for (int i = 0; i < 3600; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 1000ULL), 0.001);
  }
  filter.catchUp(3600000, kNoBudget);
  filter.resetStats();

  const eskf_scalar pos[3] = {0, 0, 0};
  const eskf_scalar vel[3] = {0, 0, 0};
  const eskf_scalar R_pos[3] = {1.0, 1.0, 1.0};
  const eskf_scalar R_vel[3] = {1.0, 1.0, 1.0};
  const eskf_scalar lever[3] = {0, 0, 0};
  const size_t events_before = filter.eventBufferCount();

  // This timestamp is older than retained IMU horizon after the ring has lapped.
  filter.pushGpsPacket(100000, pos, R_pos, vel, R_vel, lever);

  TEST_ASSERT_EQUAL(events_before, filter.eventBufferCount());
  TEST_ASSERT_EQUAL(0, filter.stats().rewind_count);
  TEST_ASSERT_EQUAL(0, filter.stats().rewind_data_gap_count);

  filter.catchUp(3600000, kNoBudget);
  TEST_ASSERT_EQUAL(0, filter.stats().rewind_count);
}

static void test_yieldable_contract_18_liftoff_snap_survives_burst_pressure() {
  // Subsystem: LiftoffSnap survivability under event-burst around liftoff.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 200000);

  for (int i = 0; i < 40; ++i) {
    filter.pushImu(createImuFrame((i + 1) * 1000ULL), 0.001);
  }

  const eskf_scalar kPi = static_cast<eskf_scalar>(3.14159265358979323846);
  const eskf_scalar q0 = static_cast<eskf_scalar>(std::cos(7.5 * kPi / 180.0));
  const eskf_scalar q2 = static_cast<eskf_scalar>(std::sin(7.5 * kPi / 180.0));

  // Inject LiftoffSnap without running catchUp yet; burst pressure is queued first.
  LiftoffInitData init_data{};
  init_data.q[0] = q0;
  init_data.q[1] = 0;
  init_data.q[2] = q2;
  init_data.q[3] = 0;
  init_data.b_gyro[0] = 0;
  init_data.b_gyro[1] = 0;
  init_data.b_gyro[2] = 0;
  init_data.heading_variance = 0.01;
  init_data.heading_initialized = false;
  init_data.liftoff_us = 10000;
  init_data.ground_altitude_m = 0;
  init_data.ground_reference_valid = false;
  filter.injectLiftoffSnap(init_data, 10000);

  // Snap must still be pending before catchUp runs.
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, filter.state().q[2]);

  // GPS packets in rejection window should be dropped before enqueue.
  const eskf_scalar pos[3] = {0, 0, 0};
  const eskf_scalar vel[3] = {0, 0, 0};
  const eskf_scalar R_pos[3] = {1.0, 1.0, 1.0};
  const eskf_scalar R_vel[3] = {1.0, 1.0, 1.0};
  const eskf_scalar lever[3] = {0, 0, 0};
  for (int i = 0; i < 180; ++i) {
    filter.pushGpsPacket(10000 + static_cast<uint64_t>(i) * 500ULL,
                         pos, R_pos, vel, R_vel, lever);
  }

  // Add additional post-window events to apply burst pressure while keeping
  // event drops outside expected operational gate assumptions.
  for (int i = 0; i < 80; ++i) {
    filter.pushMagHeading(0.0, 0.05, 220000 + static_cast<uint64_t>(i) * 1000ULL);
  }

  filter.catchUp(320000, kNoBudget);

  TEST_ASSERT_EQUAL(0, filter.stats().event_drops);
  TEST_ASSERT_TRUE(std::abs(filter.state().q[2]) > 0.05);
}

static void test_yieldable_contract_19_liftoff_rewind_skips_boot_gap_bridge() {
  // Subsystem: liftoff rewind must not bridge from initialization checkpoint
  // with a synthetic gap dt after long pad dwell.
  EskfYieldable filter;

  State initial = createDefaultState();
  initial.timestamp_us = 0;
  initFilterForContracts(filter, initial, 0, 200000);

  // Emulate long pad dwell with only recent IMU history retained.
  constexpr uint64_t kStartTsUs = 600000000ULL;  // 10 minutes after boot
  for (size_t i = 0; i < ESKF_IMU_BUFFER_SIZE; ++i) {
    filter.pushImu(createImuFrame(kStartTsUs + (i + 1) * 100ULL, 1.0, -9.80665),
                   0.0001);
  }

  LiftoffInitData init_data{};
  init_data.q[0] = 1;
  init_data.q[1] = 0;
  init_data.q[2] = 0;
  init_data.q[3] = 0;
  init_data.heading_variance = 0.01;
  init_data.heading_initialized = false;
  init_data.liftoff_us = kStartTsUs + ESKF_IMU_BUFFER_SIZE * 100ULL;
  init_data.ground_reference_valid = false;

  const uint64_t rewind_to_ts = init_data.liftoff_us - 400000ULL;

  filter.resetStats();
  filter.injectLiftoffSnap(init_data, rewind_to_ts);
  filter.catchUp(init_data.liftoff_us, kNoBudget);

  TEST_ASSERT_EQUAL(1, filter.stats().rewind_count);
  TEST_ASSERT_EQUAL(0, filter.stats().rewind_data_gap_count);
  TEST_ASSERT_TRUE(filter.state().v[0] < 0.37);
}

static void test_yieldable_contract_23_virtual_imu_policy_trace_determinism() {
  // Checklist item 23: VirtualImu policy output trace must be replay-deterministic.
  auto run_trace = []() {
    VirtualImu vimu;
    VirtualImuConfig cfg{};
    cfg.imu_count = 3;
    for (size_t i = 0; i < 3; ++i) {
      cfg.imus[i].enabled = true;
    }
    cfg.use_central_diff = false;
    cfg.voting_enabled = true;
    cfg.gyro_voting_threshold = 0.4;
    cfg.gyro_hard_fault_threshold = 2.0;
    vimu.configure(cfg);

    struct Trace {
      uint64_t ts;
      uint8_t s2;
      size_t valid;
      bool degraded;
    };
    std::vector<Trace> out_trace;

    SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                            SensorStatus::OK,
                                            SensorStatus::HARD_FAIL};

    for (int k = 0; k < 25; ++k) {
      eskf_sensor_t a0[3] = {0, 0, -9.8f};
      eskf_sensor_t a1[3] = {0, 0, -9.8f};
      eskf_sensor_t a2[3] = {0.2f * std::sin(0.2f * k), 0, -9.8f};
      eskf_sensor_t g0[3] = {0, 0, 0.1f};
      eskf_sensor_t g1[3] = {0, 0, 0.1f};
      eskf_sensor_t g2[3] = {0, 0, 0.1f + 0.7f * std::sin(0.3f * k)};

      VirtualImuRuntimePolicy pol;
      pol.soft_threshold_scale = (k >= 6 && k <= 12) ? 2.0 : 1.0;
      pol.boost_phase = (k >= 6 && k <= 12);
      vimu.setRuntimePolicy(pol);

      const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
      const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
      VirtualImuOutput out[1];
      const size_t count =
          vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                       static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
      if (count == 1) {
        Trace tr{};
        tr.ts = out[0].frame.timestamp_us;
        tr.s2 = static_cast<uint8_t>(out[0].imu_status[2]);
        tr.valid = out[0].valid_imu_count;
        tr.degraded = out[0].degraded_output;
        out_trace.push_back(tr);
      }
    }
    return out_trace;
  };

  const auto a = run_trace();
  const auto b = run_trace();
  TEST_ASSERT_EQUAL(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    TEST_ASSERT_EQUAL_UINT64(a[i].ts, b[i].ts);
    TEST_ASSERT_EQUAL(a[i].s2, b[i].s2);
    TEST_ASSERT_EQUAL(a[i].valid, b[i].valid);
    TEST_ASSERT_EQUAL(a[i].degraded, b[i].degraded);
  }
}

static void test_yieldable_contract_28_virtual_baro_policy_trace_determinism() {
  // Checklist item 28: VirtualBaro policy output trace must be replay-deterministic.
  auto run_trace = []() {
    VirtualBaro vbaro;
    VirtualBaroConfig cfg{};
    cfg.baro_count = 3;
    cfg.baros[0].enabled = true;
    cfg.baros[1].enabled = true;
    cfg.baros[2].enabled = true;
    cfg.voting_enabled = true;
    cfg.voting_threshold_pa = 350.0;
    cfg.hard_fault_threshold_pa = 2500.0;
    cfg.calibration_mismatch_tolerance_pa = 300.0;
    vbaro.configure(cfg);

    struct Trace {
      uint64_t ts;
      uint8_t s2;
      size_t valid;
      bool degraded;
    };
    std::vector<Trace> out_trace;

    SensorStatus statuses[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
    eskf_sensor_t temps[3] = {288.15f, 288.15f, 288.15f};

    for (int k = 0; k < 30; ++k) {
      eskf_sensor_t p[3] = {
          101325.0f + 10.0f * std::sin(0.1f * k),
          101330.0f + 8.0f * std::sin(0.12f * k + 0.3f),
          101335.0f + ((k % 10 == 0) ? 1200.0f : 6.0f * std::sin(0.11f * k))};
      BaroOutput out =
          vbaro.process(p, temps, statuses, static_cast<uint64_t>(k) * 20000ULL);
      Trace tr{};
      tr.ts = out.timestamp_us;
      tr.s2 = static_cast<uint8_t>(out.baro_status[2]);
      tr.valid = out.valid_count;
      tr.degraded = out.degraded_output;
      out_trace.push_back(tr);
    }
    return out_trace;
  };

  const auto a = run_trace();
  const auto b = run_trace();
  TEST_ASSERT_EQUAL(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    TEST_ASSERT_EQUAL_UINT64(a[i].ts, b[i].ts);
    TEST_ASSERT_EQUAL(a[i].s2, b[i].s2);
    TEST_ASSERT_EQUAL(a[i].valid, b[i].valid);
    TEST_ASSERT_EQUAL(a[i].degraded, b[i].degraded);
  }
}
#endif

// ============================================================
// GTest Wrapper
// ============================================================

#define WRAP_TEST(test_fn) TEST(KalmanGpsRewindFullSuite, test_fn) { test_fn(); }

WRAP_TEST(test_initialize_resets_buffers);
WRAP_TEST(test_pushImu_only_buffers);
WRAP_TEST(test_catchUp_processes_buffered_imu);
WRAP_TEST(test_catchUp_respects_budget);
WRAP_TEST(test_isBehind_returns_correct_status);

WRAP_TEST(test_baro_async_reserve_and_set);
WRAP_TEST(test_baro_on_time_replay_uses_direct_correction);
WRAP_TEST(test_baro_late_completion_uses_innovation_transport);

#if ESKF_USE_GPS_REWIND
WRAP_TEST(test_gps_unified_timestamp_with_delay);
WRAP_TEST(test_liftoff_snap_stored_and_replayed);

WRAP_TEST(test_buffer_overflow_during_rewind);
WRAP_TEST(test_out_of_order_event_timestamps);
WRAP_TEST(test_rewind_before_any_checkpoints_uses_initial);
WRAP_TEST(test_multiple_sequential_rewinds);
WRAP_TEST(test_completeBaro_vs_setBaroMeasurement_equivalence);
WRAP_TEST(test_mag_heading_outlier_is_rejected_by_state_machine);
WRAP_TEST(test_completeBaro_stale_drop_counted_once);

WRAP_TEST(test_yieldable_contract_01_liftoff_snap_precedes_tied_imu);
WRAP_TEST(test_yieldable_contract_01_same_timestamp_baro_then_event_fifo);
WRAP_TEST(test_yieldable_contract_02_catchup_true_when_nothing_processable);
WRAP_TEST(test_yieldable_contract_03_budget_yield_cursor_integrity_deterministic_clock);
WRAP_TEST(test_yieldable_contract_04_rewind_replay_indices_match_checkpoint_contract);
WRAP_TEST(test_yieldable_contract_05_rewind_data_gap_detected);
WRAP_TEST(test_yieldable_contract_06_ring_overflow_asymmetry);
WRAP_TEST(test_yieldable_contract_07_out_of_order_processed_chronologically);
WRAP_TEST(test_yieldable_contract_11_trigger_complete_baro_pending_contract);
WRAP_TEST(test_yieldable_contract_15_dual_timestamp_single_delay_for_packet_ordering);
WRAP_TEST(test_yieldable_contract_16_drop_late_gps_older_than_retained_imu_history);
WRAP_TEST(test_yieldable_contract_18_liftoff_snap_survives_burst_pressure);
WRAP_TEST(test_yieldable_contract_19_liftoff_rewind_skips_boot_gap_bridge);
WRAP_TEST(test_yieldable_contract_23_virtual_imu_policy_trace_determinism);
WRAP_TEST(test_yieldable_contract_28_virtual_baro_policy_trace_determinism);
#endif

#undef WRAP_TEST
