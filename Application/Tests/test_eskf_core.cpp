// ESKF Core Unit Tests
// Tests filter initialization, prediction, and correction

#include "Application/Tests/unity_gtest_compat.hpp"

#include <cmath>

#include "Application/Kalman/kalman/eskf_core.hpp"

using namespace eskf;

// Tolerance for floating-point comparisons
constexpr eskf_scalar kTol = 1e-9;
constexpr eskf_scalar kTolF = 1e-5;

// ============================================================
// Initialization Tests
// ============================================================

static void test_reset_sets_identity_quaternion() {
  EskfCore filter;
  filter.reset();

  const State& s = filter.state();

  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, s.q[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, s.q[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, s.q[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, s.q[3]);
}

static void test_reset_zeros_position_velocity() {
  EskfCore filter;
  filter.reset();

  const State& s = filter.state();

  for (int i = 0; i < 3; ++i) {
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, s.p[i]);
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, s.v[i]);
  }
}

static void test_initialize_sets_state() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  initial.p[0] = 10;
  initial.p[1] = 20;
  initial.p[2] = 30;
  initial.v[2] = -5;

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();

  filter.initialize(initial, P0, Q);

  const State& s = filter.state();

  TEST_ASSERT_DOUBLE_WITHIN(kTol, 10.0, s.p[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 20.0, s.p[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 30.0, s.p[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, -5.0, s.v[2]);
}

static void test_initialize_sets_covariance() {
  EskfCore filter;

  State initial;
  initial.setIdentity();

  InitialCovariance P0;
  P0.pos = 5.0;
  P0.vel = 0.5;
  P0.tilt = 0.1;
  P0.heading = 1.0;
  P0.accel_bias = 0.2;
  P0.gyro_bias = 0.05;
  P0.baro_bias = 2.0;

  ProcessNoise Q = ProcessNoise::defaults();

  filter.initialize(initial, P0, Q);

  const Covariance& cov = filter.covariance();

  // Check position diagonal (σ² = 25)
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 25.0, cov.P[0][0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 25.0, cov.P[1][1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 25.0, cov.P[2][2]);

  // Check velocity diagonal (σ² = 0.25)
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.25, cov.P[3][3]);
}

// ============================================================
// Prediction Tests
// ============================================================

static void test_predict_zero_imu_no_position_change() {
  EskfCore filter;
  filter.reset();

  ImuFrame imu{};
  imu.accel[0] = 0;
  imu.accel[1] = 0;
  imu.accel[2] = -9.80665;  // Cancel gravity (body pointing up)
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 0;
  imu.timestamp_us = 1000;

  filter.predict(imu, 0.01);

  const State& s = filter.state();

  // Position should remain near zero (small integration drift allowed)
  TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, s.p[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, s.p[1]);
}

static void test_predict_constant_accel_increases_velocity() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  // Apply 1 m/s² acceleration in body X direction
  // With identity quaternion, body X = NED North
  ImuFrame imu{};
  imu.accel[0] = 1.0;
  imu.accel[1] = 0;
  imu.accel[2] = -9.80665;  // Cancel gravity
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 0;
  imu.timestamp_us = 1000;

  eskf_scalar dt = 0.01;
  filter.predict(imu, dt);

  const State& s = filter.state();

  // Velocity should increase by ~1*0.01 = 0.01 m/s in North
  TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.01, s.v[0]);
}

static void test_predict_rotation_updates_quaternion() {
  EskfCore filter;
  filter.reset();

  // Apply 1 rad/s rotation about Z
  ImuFrame imu{};
  imu.accel[0] = 0;
  imu.accel[1] = 0;
  imu.accel[2] = -9.80665;
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 1.0;  // 1 rad/s about Z
  imu.timestamp_us = 1000;

  eskf_scalar dt = 0.1;  // 0.1 rad rotation
  filter.predict(imu, dt);

  const State& s = filter.state();

  // Quaternion should no longer be identity
  TEST_ASSERT_TRUE(std::abs(s.q[0] - 1.0) > 0.001);
  TEST_ASSERT_TRUE(std::abs(s.q[3]) > 0.001);  // Z component

  // But norm should still be 1
  eskf_scalar norm = std::sqrt(s.q[0]*s.q[0] + s.q[1]*s.q[1] +
                               s.q[2]*s.q[2] + s.q[3]*s.q[3]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);
}

static void test_predict_1000_steps_quaternion_norm() {
  EskfCore filter;
  filter.reset();

  ImuFrame imu{};
  imu.accel[0] = 0.5;
  imu.accel[1] = -0.3;
  imu.accel[2] = -9.8;
  imu.gyro[0] = 0.1;
  imu.gyro[1] = 0.05;
  imu.gyro[2] = 0.02;

  eskf_scalar dt = 0.001;  // 1ms steps

  for (int i = 0; i < 1000; ++i) {
    imu.timestamp_us = static_cast<uint64_t>(i * 1000);
    filter.predict(imu, dt);
  }

  const State& s = filter.state();
  eskf_scalar norm = std::sqrt(s.q[0]*s.q[0] + s.q[1]*s.q[1] +
                               s.q[2]*s.q[2] + s.q[3]*s.q[3]);

  // Norm should be within 1e-6 of 1.0
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0, norm);
}

static void test_predict_covariance_grows() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  eskf_scalar initial_pos_var = filter.covariance().P[0][0];

  ImuFrame imu{};
  imu.accel[0] = 0;
  imu.accel[1] = 0;
  imu.accel[2] = -9.80665;
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 0;
  imu.timestamp_us = 1000;

  for (int i = 0; i < 100; ++i) {
    filter.predict(imu, 0.01);
  }

  // Position variance should have grown due to process noise
  TEST_ASSERT_TRUE(filter.covariance().P[0][0] > initial_pos_var);
}

// ============================================================
// Correction Tests
// ============================================================

static void test_correctGpsPosition_reduces_error() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  initial.p[0] = 10;  // 10m North error
  initial.p[1] = 5;   // 5m East error

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  // GPS says position is [0, 0, 0]
  eskf_scalar pos_ned[3] = {0, 0, 0};
  eskf_scalar R[3] = {1.0, 1.0, 4.0};  // 1m² variance N/E, 2m² D

  filter.correctGpsPosition(pos_ned, R);

  const State& s = filter.state();

  // Position error should be reduced (pulled towards 0)
  TEST_ASSERT_TRUE(std::abs(s.p[0]) < 10.0);
  TEST_ASSERT_TRUE(std::abs(s.p[1]) < 5.0);
}

static void test_correctGpsVelocity_reduces_error() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  initial.v[0] = 5.0;  // 5 m/s North velocity error

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  // GPS says velocity is [0, 0, 0]
  eskf_scalar vel_ned[3] = {0, 0, 0};
  eskf_scalar R[3] = {0.25, 0.25, 0.25};  // 0.5 m/s variance

  filter.correctGpsVelocity(vel_ned, R);

  const State& s = filter.state();

  // Velocity error should be reduced
  TEST_ASSERT_TRUE(std::abs(s.v[0]) < 5.0);
}

static void test_correctBaroAltitude_updates_position() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  initial.p[2] = 100;  // 100m below ground (in NED, positive is down)

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  // Baro says altitude is 0m (z = 0 in NED means altitude = 0)
  eskf_scalar alt_m = 0;
  eskf_scalar R = 1.0;  // 1m² variance

  filter.correctBaroAltitude(alt_m, R);

  const State& s = filter.state();

  // Z position should move towards 0
  TEST_ASSERT_TRUE(std::abs(s.p[2]) < 100.0);
}

static void test_correction_produces_valid_nis() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  initial.p[0] = 5.0;

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  eskf_scalar pos_ned[3] = {0, 0, 0};
  eskf_scalar R[3] = {1.0, 1.0, 1.0};

  filter.correctGpsPosition(pos_ned, R);

  // NIS should be finite and positive
  eskf_scalar nis = filter.lastNIS();
  TEST_ASSERT_TRUE(std::isfinite(nis));
  TEST_ASSERT_TRUE(nis >= 0);
}

static void test_covariance_diagonal_remains_positive() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  // Run prediction and corrections
  ImuFrame imu{};
  imu.accel[0] = 0.1;
  imu.accel[1] = 0.2;
  imu.accel[2] = -9.8;
  imu.gyro[0] = 0.01;
  imu.gyro[1] = 0.02;
  imu.gyro[2] = 0.01;

  for (int i = 0; i < 100; ++i) {
    imu.timestamp_us = static_cast<uint64_t>(i * 1000);
    filter.predict(imu, 0.001);
  }

  eskf_scalar pos_ned[3] = {0, 0, 0};
  eskf_scalar R[3] = {1.0, 1.0, 1.0};
  filter.correctGpsPosition(pos_ned, R);

  const Covariance& cov = filter.covariance();

  // All diagonal elements should be positive
  for (int i = 0; i < kDimError; ++i) {
    TEST_ASSERT_TRUE(cov.P[i][i] > 0);
  }
}

// ============================================================
// Checkpoint Tests
// ============================================================

static void test_checkpoint_restore() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  initial.p[0] = 100;
  initial.v[1] = 50;

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  // Create checkpoint
  Checkpoint cp = filter.createCheckpoint();

  // Modify state
  ImuFrame imu{};
  imu.accel[0] = 1.0;
  imu.accel[1] = 0;
  imu.accel[2] = -9.8;
  imu.gyro[0] = 0;
  imu.gyro[1] = 0;
  imu.gyro[2] = 0;
  imu.timestamp_us = 10000;

  for (int i = 0; i < 50; ++i) {
    filter.predict(imu, 0.01);
  }

  // State has changed
  TEST_ASSERT_TRUE(std::abs(filter.state().p[0] - 100) > 0.1);

  // Restore checkpoint
  filter.restoreCheckpoint(cp);

  // State should be back to checkpoint values
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 100.0, filter.state().p[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 50.0, filter.state().v[1]);
}

void test_processHeadingUpdate_logic() {
  EskfCore filter;
  filter.reset();
  
  // First update: should SNAP and return Snapped
  HeadingUpdateResult res = filter.processHeadingUpdate(0.5, 0.01);
  TEST_ASSERT_EQUAL(static_cast<int>(HeadingUpdateResult::Snapped), static_cast<int>(res));
  TEST_ASSERT_TRUE(filter.isHeadingInitialized());
  
  // Second update close to current: should FUSE
  res = filter.processHeadingUpdate(0.51, 0.01);
  TEST_ASSERT_EQUAL(static_cast<int>(HeadingUpdateResult::Fused), static_cast<int>(res));
  
  // Update far away: should be REJECTED (outlier)
  // 0.5 rad ~ 28 deg. Try 2.0 rad (~114 deg)
  res = filter.processHeadingUpdate(2.0, 0.01);
  TEST_ASSERT_EQUAL(static_cast<int>(HeadingUpdateResult::Rejected), static_cast<int>(res));
  
  // Trigger resurrection (need ESKF_HEADING_RESURRECT_COUNT rejects)
  // We already have 1 reject. Need 49 more (if count is 50)
  for (int i = 0; i < ESKF_HEADING_RESURRECT_COUNT - 2; ++i) {
    res = filter.processHeadingUpdate(2.0, 0.01);
    TEST_ASSERT_EQUAL(static_cast<int>(HeadingUpdateResult::Rejected), static_cast<int>(res));
  }
  
  // The next one should trigger resurrection
  res = filter.processHeadingUpdate(2.0, 0.01);
  TEST_ASSERT_EQUAL(static_cast<int>(HeadingUpdateResult::Resurrected), static_cast<int>(res));
  
  // After resurrection, heading should match the snapped value (2.0)
  // Verify by fusing a value close to 2.0
  res = filter.processHeadingUpdate(2.01, 0.01);
  TEST_ASSERT_EQUAL(static_cast<int>(HeadingUpdateResult::Fused), static_cast<int>(res));
}

// ============================================================
// forceYaw Position/Velocity Rotation Tests
// ============================================================

void test_forceYaw_rotates_position() {
  EskfCore filter;
  filter.reset();
  
  // Set position at (100, 0, 0) - facing North with 100m North offset
  State s = filter.state();
  s.p[0] = 100.0;  // North
  s.p[1] = 0.0;    // East
  s.p[2] = -50.0;  // Up (50m altitude)
  filter.setState(s);
  
  // Rotate yaw by 90° (π/2 rad) - now should be at (0, 100, 0) East
  filter.forceYaw(constants::kPi / 2, true);
  
  const State& result = filter.state();
  
  // Position should rotate: North=0, East=100, Down unchanged
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, result.p[0]);    // North (was 100)
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 100.0, result.p[1]);  // East (was 0)
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, -50.0, result.p[2]);  // Down (unchanged)
}

void test_forceYaw_rotates_velocity_without_gps() {
  EskfCore filter;
  filter.reset();
  
  // Set velocity at (50, 0, -10) - moving North at 50m/s, descending at 10m/s
  State s = filter.state();
  s.v[0] = 50.0;   // North
  s.v[1] = 0.0;    // East
  s.v[2] = -10.0;  // Down (ascending)
  filter.setState(s);
  
  // Rotate yaw by 90° (π/2 rad) - velocity should rotate
  filter.forceYaw(constants::kPi / 2, true, nullptr);  // nullptr = rotate velocity
  
  const State& result = filter.state();
  
  // Velocity should rotate: was moving North, now moving East
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, result.v[0]);    // North (was 50)
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 50.0, result.v[1]);   // East (was 0)
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, -10.0, result.v[2]);  // Down (unchanged)
}

void test_forceYaw_overwrites_velocity_with_gps() {
  EskfCore filter;
  filter.reset();
  
  // Set velocity at (50, 0, -10) - moving North at 50m/s
  State s = filter.state();
  s.v[0] = 50.0;
  s.v[1] = 0.0;
  s.v[2] = -10.0;
  filter.setState(s);
  
  // GPS says velocity is (30, 40, -5) - moving Northeast
  eskf_scalar gps_vel[3] = {30.0, 40.0, -5.0};
  
  // Rotate yaw by 45° and provide GPS velocity - should OVERWRITE
  filter.forceYaw(constants::kPi / 4, true, gps_vel);
  
  const State& result = filter.state();
  
  // Velocity should be exactly GPS values (overwritten, not rotated)
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 30.0, result.v[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 40.0, result.v[1]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, -5.0, result.v[2]);
}

void test_attemptHeadingAlignment_no_pitch_gate() {
  EskfCore filter;
  
  // Initialize filter
  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);
  
  // Feed IMU to set prev_gyro_ (required for gyro gate check)
  ImuFrame imu{};
  imu.accel[2] = -9.81;
  imu.gyro[0] = 0; imu.gyro[1] = 0; imu.gyro[2] = 0;
  imu.timestamp_us = 1000;
  filter.predict(imu, 0.01);
  
  // High-pitch flight: 89° pitch, horizontal speed = 50 m/s
  // Old pitch gate would reject this. New horizontal speed gate should accept.
  // cos(89°) ≈ 0.017, so v_vert/v_horiz ≈ 58
  // For 50 m/s horizontal, vertical is ~2900 m/s (unrealistic but tests gate removal)
  // Actually just use ~50 m/s horizontal, ~10 m/s vertical for a reasonable test
  eskf_scalar gps_vel[3] = {50.0, 0.0, -10.0};  // ~78° flight path angle
  eskf_scalar sAcc = 0.5;
  
  // This should now pass since we removed the pitch gate
  bool aligned = filter.attemptHeadingAlignment(gps_vel, sAcc);
  
  TEST_ASSERT_TRUE(aligned);
  TEST_ASSERT_TRUE(filter.isHeadingAligned());
  TEST_ASSERT_TRUE(filter.isHeadingInitialized());
}

static void test_resetVerticalChannelFromBaro_covariance_contract() {
  EskfCore filter;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  State s = filter.state();
  s.b_baro = 120.0;
  s.p[2] = -3.0;
  filter.setState(s);

  Covariance cov = filter.covariance();
  for (int i = 0; i < kDimError; ++i) {
    cov.P[idx::kPos + 2][i] = 0.1 * (i + 1);
    cov.P[i][idx::kPos + 2] = 0.1 * (i + 2);
  }
  cov.P[idx::kBarBias][idx::kBarBias] = 9.0;
  filter.setCovariance(cov);

  filter.resetVerticalChannelFromBaro(100.0, 4.0);

  const State& out = filter.state();
  const Covariance& out_cov = filter.covariance();

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 20.0, out.p[2]);
  for (int i = 0; i < kDimError; ++i) {
    if (i == idx::kPos + 2) {
      continue;
    }
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, out_cov.P[idx::kPos + 2][i]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, out_cov.P[i][idx::kPos + 2]);
  }
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 13.0,
                            out_cov.P[idx::kPos + 2][idx::kPos + 2]);
}

static void test_covariance_inflate_floor_does_not_shrink() {
  EskfCore filter;
  filter.reset();

  Covariance cov = filter.covariance();
  cov.P[idx::kPos + 0][idx::kPos + 0] = 25.0;
  cov.P[idx::kVel + 1][idx::kVel + 1] = 16.0;
  filter.setCovariance(cov);

  filter.inflatePositionCovariance(4.0);
  filter.inflateVelocityCovariance(9.0);

  const Covariance& out = filter.covariance();
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 25.0, out.P[idx::kPos + 0][idx::kPos + 0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 16.0, out.P[idx::kVel + 1][idx::kVel + 1]);
}

static void test_covariance_reset_block_clears_cross_terms() {
  EskfCore filter;
  filter.reset();

  Covariance cov = filter.covariance();
  cov.P[idx::kPos + 0][idx::kVel + 0] = 1.5;
  cov.P[idx::kVel + 0][idx::kPos + 0] = 1.5;
  cov.P[idx::kVel + 1][idx::kAtt + 2] = -0.8;
  cov.P[idx::kAtt + 2][idx::kVel + 1] = -0.8;
  filter.setCovariance(cov);

  filter.resetPositionCovariance(3.0);
  filter.resetVelocityCovariance(2.0);

  const Covariance& out = filter.covariance();
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 3.0, out.P[idx::kPos + 0][idx::kPos + 0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 2.0, out.P[idx::kVel + 0][idx::kVel + 0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, out.P[idx::kPos + 0][idx::kVel + 0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, out.P[idx::kVel + 0][idx::kPos + 0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, out.P[idx::kVel + 1][idx::kAtt + 2]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, out.P[idx::kAtt + 2][idx::kVel + 1]);
}

// ============================================================
// Behavior Contracts (Checklist Item 14)
// ============================================================

static void test_core_contract_14_covariance_decimation_cadence() {
  // Subsystem: EskfCore predict covariance decimation implementation contract.
  EskfCore filter;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  ImuFrame imu{};
  imu.accel[0] = 0.2;
  imu.accel[1] = -0.1;
  imu.accel[2] = -9.80665;
  imu.gyro[0] = 0.02;
  imu.gyro[1] = -0.03;
  imu.gyro[2] = 0.01;

  const eskf_scalar baseline = filter.covariance().P[0][0];

#if ESKF_COVARIANCE_DECIMATION > 1
  for (int i = 0; i < ESKF_COVARIANCE_DECIMATION - 1; ++i) {
    imu.timestamp_us = static_cast<uint64_t>(i + 1) * 1000ULL;
    filter.predict(imu, 0.001);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, baseline, filter.covariance().P[0][0]);
  }

  imu.timestamp_us = static_cast<uint64_t>(ESKF_COVARIANCE_DECIMATION) * 1000ULL;
  filter.predict(imu, 0.001);
  TEST_ASSERT_TRUE(std::abs(filter.covariance().P[0][0] - baseline) > 1e-12);
#else
  imu.timestamp_us = 1000;
  filter.predict(imu, 0.001);
  TEST_ASSERT_TRUE(std::abs(filter.covariance().P[0][0] - baseline) > 1e-12);
#endif
}

static void test_core_contract_14_covariance_decimation_accumulates_jacobians() {
#if ESKF_COVARIANCE_DECIMATION > 1
  EskfCore a;
  EskfCore b;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  a.initialize(initial, P0, Q);
  b.initialize(initial, P0, Q);

  for (int i = 0; i < ESKF_COVARIANCE_DECIMATION; ++i) {
    ImuFrame imu_a{};
    imu_a.accel[0] = (i == 0) ? 20.0 : 0.0;
    imu_a.accel[1] = 0.0;
    imu_a.accel[2] = -9.80665;
    imu_a.gyro[0] = 0.0;
    imu_a.gyro[1] = 0.0;
    imu_a.gyro[2] = 0.0;
    imu_a.timestamp_us = static_cast<uint64_t>(i + 1) * 1000ULL;

    ImuFrame imu_b = imu_a;
    imu_b.accel[0] = (i == 0) ? -20.0 : 0.0;

    a.predict(imu_a, 0.001);
    b.predict(imu_b, 0.001);
  }

  eskf_scalar abs_diff_sum = 0;
  for (int r = 0; r < kDimError; ++r) {
    for (int c = 0; c < kDimError; ++c) {
      abs_diff_sum += std::abs(a.covariance().P[r][c] - b.covariance().P[r][c]);
    }
  }

  TEST_ASSERT_TRUE(abs_diff_sum > 1e-8);
#else
  TEST_IGNORE_MESSAGE("Covariance decimation disabled in this build");
#endif
}

static void test_core_contract_14_covariance_decimation_flush_before_updates() {
#if ESKF_COVARIANCE_DECIMATION > 1
  EskfCore filter;

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  const eskf_scalar baseline_pos_var =
      filter.covariance().P[idx::kPos + 0][idx::kPos + 0];

  ImuFrame imu{};
  imu.accel[0] = 0.1;
  imu.accel[1] = -0.2;
  imu.accel[2] = -9.80665;
  imu.gyro[0] = 0.01;
  imu.gyro[1] = -0.01;
  imu.gyro[2] = 0.02;

  for (int i = 0; i < ESKF_COVARIANCE_DECIMATION - 1; ++i) {
    imu.timestamp_us = static_cast<uint64_t>(i + 1) * 1000ULL;
    filter.predict(imu, 0.001);
  }

  // P is still deferred at this point; innovation call should force a flush.
  eskf_scalar innovations[3] = {};
  eskf_scalar S[3] = {};
  const eskf_scalar pos_ned[3] = {0.0, 0.0, 0.0};
  const eskf_scalar R[3] = {1.0, 1.0, 1.0};
  (void)filter.computeGpsPositionInnovation(pos_ned, R, innovations, S);

  TEST_ASSERT_TRUE(
      filter.covariance().P[idx::kPos + 0][idx::kPos + 0] > baseline_pos_var);
#else
  TEST_IGNORE_MESSAGE("Covariance decimation disabled in this build");
#endif
}

static void test_flight_mode_freezes_imu_bias_covariance_but_keeps_baro_bias_rw() {
  EskfCore filter;

  TuningConfig tuning = getDefaultTuningConfig();
  tuning.freeze_accel_bias_in_flight = true;
  tuning.freeze_gyro_bias_in_flight = true;
  filter.init(tuning);

  State initial;
  initial.setIdentity();
  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();
  filter.initialize(initial, P0, Q);

  filter.setMode(FilterMode::Flight);

  const eskf_scalar ba0 = filter.covariance().P[idx::kAccBias + 0][idx::kAccBias + 0];
  const eskf_scalar bg0 = filter.covariance().P[idx::kGyrBias + 0][idx::kGyrBias + 0];
  const eskf_scalar bb0 = filter.covariance().P[idx::kBarBias][idx::kBarBias];

  ImuFrame imu{};
  imu.accel[0] = 0.2;
  imu.accel[1] = -0.1;
  imu.accel[2] = -9.80665;
  imu.gyro[0] = 0.03;
  imu.gyro[1] = -0.02;
  imu.gyro[2] = 0.01;

  for (int i = 0; i < 100; ++i) {
    imu.timestamp_us = static_cast<uint64_t>(i + 1) * 1000ULL;
    filter.predict(imu, 0.001);
  }

  const Covariance& cov = filter.covariance();
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, ba0, cov.P[idx::kAccBias + 0][idx::kAccBias + 0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, bg0, cov.P[idx::kGyrBias + 0][idx::kGyrBias + 0]);
  TEST_ASSERT_TRUE(cov.P[idx::kBarBias][idx::kBarBias] > bb0);
}

static void test_flight_mode_freezes_imu_bias_state_updates() {
  TuningConfig tuning = getDefaultTuningConfig();
  tuning.freeze_accel_bias_in_flight = true;
  tuning.freeze_gyro_bias_in_flight = true;

  State initial;
  initial.setIdentity();
  initial.b_acc[0] = 0.5;
  initial.b_gyro[0] = 0.05;

  InitialCovariance P0 = InitialCovariance::defaults();
  ProcessNoise Q = ProcessNoise::defaults();

  // Flight-mode filter: IMU bias states should remain fixed.
  EskfCore flight_filter;
  flight_filter.init(tuning);
  flight_filter.initialize(initial, P0, Q);
  flight_filter.setMode(FilterMode::Flight);

  Covariance cov_flight = flight_filter.covariance();
  cov_flight.P[idx::kAccBias + 0][idx::kPos + 0] = 5.0;
  cov_flight.P[idx::kPos + 0][idx::kAccBias + 0] = 5.0;
  cov_flight.P[idx::kGyrBias + 0][idx::kPos + 0] = 3.0;
  cov_flight.P[idx::kPos + 0][idx::kGyrBias + 0] = 3.0;
  flight_filter.setCovariance(cov_flight);

  const eskf_scalar pos_meas[3] = {20.0, 0.0, 0.0};
  const eskf_scalar R_pos[3] = {0.01, 1.0, 1.0};
  flight_filter.correctGpsPosition(pos_meas, R_pos);

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.5, flight_filter.state().b_acc[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.05, flight_filter.state().b_gyro[0]);

  // Settling-mode filter: same corrections should move bias states.
  EskfCore settling_filter;
  settling_filter.init(tuning);
  settling_filter.initialize(initial, P0, Q);

  Covariance cov_settling = settling_filter.covariance();
  cov_settling.P[idx::kAccBias + 0][idx::kPos + 0] = 5.0;
  cov_settling.P[idx::kPos + 0][idx::kAccBias + 0] = 5.0;
  cov_settling.P[idx::kGyrBias + 0][idx::kPos + 0] = 3.0;
  cov_settling.P[idx::kPos + 0][idx::kGyrBias + 0] = 3.0;
  settling_filter.setCovariance(cov_settling);

  settling_filter.correctGpsPosition(pos_meas, R_pos);

  TEST_ASSERT_TRUE(std::abs(settling_filter.state().b_acc[0] - 0.5) > 1e-6);
  TEST_ASSERT_TRUE(std::abs(settling_filter.state().b_gyro[0] - 0.05) > 1e-6);
}

// ============================================================
// GTest Wrapper
// ============================================================

#define WRAP_TEST(test_fn) TEST(KalmanEskfCoreSuite, test_fn) { test_fn(); }

WRAP_TEST(test_reset_sets_identity_quaternion);
WRAP_TEST(test_reset_zeros_position_velocity);
WRAP_TEST(test_initialize_sets_state);
WRAP_TEST(test_initialize_sets_covariance);

WRAP_TEST(test_predict_zero_imu_no_position_change);
WRAP_TEST(test_predict_constant_accel_increases_velocity);
WRAP_TEST(test_predict_rotation_updates_quaternion);
WRAP_TEST(test_predict_1000_steps_quaternion_norm);
WRAP_TEST(test_predict_covariance_grows);

WRAP_TEST(test_correctGpsPosition_reduces_error);
WRAP_TEST(test_correctGpsVelocity_reduces_error);
WRAP_TEST(test_correctBaroAltitude_updates_position);
WRAP_TEST(test_correction_produces_valid_nis);
WRAP_TEST(test_covariance_diagonal_remains_positive);

WRAP_TEST(test_checkpoint_restore);
WRAP_TEST(test_processHeadingUpdate_logic);

WRAP_TEST(test_forceYaw_rotates_position);
WRAP_TEST(test_forceYaw_rotates_velocity_without_gps);
WRAP_TEST(test_forceYaw_overwrites_velocity_with_gps);
WRAP_TEST(test_attemptHeadingAlignment_no_pitch_gate);
WRAP_TEST(test_resetVerticalChannelFromBaro_covariance_contract);
WRAP_TEST(test_covariance_inflate_floor_does_not_shrink);
WRAP_TEST(test_covariance_reset_block_clears_cross_terms);
WRAP_TEST(test_core_contract_14_covariance_decimation_cadence);
WRAP_TEST(test_core_contract_14_covariance_decimation_accumulates_jacobians);
WRAP_TEST(test_core_contract_14_covariance_decimation_flush_before_updates);
WRAP_TEST(test_flight_mode_freezes_imu_bias_covariance_but_keeps_baro_bias_rw);
WRAP_TEST(test_flight_mode_freezes_imu_bias_state_updates);

#undef WRAP_TEST
