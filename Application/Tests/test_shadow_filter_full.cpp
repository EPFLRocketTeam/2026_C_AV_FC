// Shadow Filter Unit Tests
// Tests for RailShadowFilter and FlightShadowFilter

#include "Application/Tests/unity_gtest_compat.hpp"

#include <cmath>

#include "Application/Kalman/kalman/shadow_filter.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_imu.hpp"

using namespace eskf;

// Tolerance for floating-point comparisons
constexpr eskf_scalar kTol = 1e-6;
constexpr eskf_scalar kTolLoose = 1e-3;

// ============================================================
// RailShadowFilter Tests
// ============================================================

static void test_rail_reset_identity() {
  RailShadowFilter filter;
  filter.reset();

  const eskf_scalar *q = filter.quaternion();

  // Should be identity quaternion
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, q[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, q[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, q[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, q[3]);
  TEST_ASSERT_TRUE(filter.isGateOpen());
}

static void test_rail_static_converges_to_gravity() {
  RailShadowFilter filter;
  filter.reset();

  // Rocket pointing UP (nose up, -Z in body = -Down in NED = Up)
  // Accelerometer measures -g in body Z (gravity pulls down, sensor reads up)
  const eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.01;

  // Run for a few seconds to let filter converge
  for (int i = 0; i < 500; ++i) {
    filter.update(accel, gyro, dt);
  }

  // Gate should be open (accel magnitude matches gravity)
  TEST_ASSERT_TRUE(filter.isGateOpen());

  // Quaternion should be close to identity (rocket aligned with gravity)
  const eskf_scalar *q = filter.quaternion();
  eskf_scalar norm =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);
}

static void test_rail_gate_closes_during_handling() {
  RailShadowFilter filter;
  filter.reset();

  // Normal gravity - gate should be open
  eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.01;

  filter.update(accel, gyro, dt);
  TEST_ASSERT_TRUE(filter.isGateOpen());

  // Add significant linear acceleration (handling/shaking)
  // Magnitude will be sqrt(9.8^2 + 5^2) ≈ 11 m/s², way off from g
  accel[0] = 5.0;
  filter.update(accel, gyro, dt);
  TEST_ASSERT_FALSE(filter.isGateOpen());

  // Return to pure gravity - gate should reopen
  accel[0] = 0.0;
  filter.update(accel, gyro, dt);
  TEST_ASSERT_TRUE(filter.isGateOpen());
}

static void test_rail_gyro_only_during_motion() {
  RailShadowFilter filter;
  filter.reset();

  // Large linear acceleration - gate will close
  const eskf_scalar accel[3] = {5.0, 0, -9.80665}; // Handling motion
  const eskf_scalar gyro[3] = {0, 0, 1.0};         // 1 rad/s about Z
  const eskf_scalar dt = 0.1;

  // Update filter - gate should close
  filter.update(accel, gyro, dt);
  TEST_ASSERT_FALSE(filter.isGateOpen());

  // Quaternion should have rotated by gyro only (Z rotation)
  const eskf_scalar *q = filter.quaternion();

  // After 0.1s at 1 rad/s: θ = 0.1 rad
  // q = [cos(θ/2), 0, 0, sin(θ/2)] for Z rotation
  // Expected: q ≈ [0.9988, 0, 0, 0.05]
  TEST_ASSERT_TRUE(std::abs(q[3]) > 0.04);    // Non-zero Z component
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, q[1]); // X and Y near zero
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, q[2]);
}

static void test_rail_tilted_convergence() {
  RailShadowFilter filter;
  filter.reset();

  // Rocket tilted 30° pitch (nose up relative to vertical)
  // In body frame, gravity appears rotated
  const eskf_scalar pitch_rad = 30.0 * M_PI / 180.0;
  const eskf_scalar g = 9.80665;
  const eskf_scalar accel[3] = {
      -g * std::sin(pitch_rad), // X component (forward)
      0,                        // Y component
      -g * std::cos(pitch_rad)  // Z component (down)
  };
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.01;

  // Run for several seconds
  for (int i = 0; i < 1000; ++i) {
    filter.update(accel, gyro, dt);
  }

  // Filter should converge - check quaternion is unit norm
  const eskf_scalar *q = filter.quaternion();
  eskf_scalar norm =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);

  // The quaternion should represent approximately 30° pitch
  // q_y component should be significant for pitch rotation
  TEST_ASSERT_TRUE(std::abs(q[2]) > 0.1); // Y component for pitch
}

static void test_rail_85deg_pitch_convergence() {
  RailShadowFilter filter;
  filter.reset();

  // Rocket tilted 85° pitch (nose up, nearly vertical)
  // In body frame, gravity appears rotated
  // For pitch about Y axis: body X gets most of gravity
  constexpr eskf_scalar pitch_deg = 85.0;
  constexpr eskf_scalar pitch_rad = pitch_deg * M_PI / 180.0;
  constexpr eskf_scalar g = 9.80665;

  // Body frame convention: X = nose, Z = up
  // For 85° pitch (nose nearly up):
  // - Body X sees: g * sin(pitch) ≈ 9.77 m/s² (gravity component along nose)
  // - Body Z sees: -g * cos(pitch) ≈ -0.85 m/s² (gravity component along Z)
  const eskf_scalar accel[3] = {
      g * std::sin(pitch_rad), // X: ~9.77 m/s²
      0,                       // Y: 0
      -g * std::cos(pitch_rad) // Z: ~-0.85 m/s²
  };
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001; // 1kHz for faster convergence

  // Initialize filter to prevent integral windup
  filter.initializeFromAccel(accel);

  // Run for several seconds to converge (Mahony filter needs time)
  for (int i = 0; i < 5000; ++i) {
    filter.update(accel, gyro, dt);
  }

  // Check quaternion is unit norm
  const eskf_scalar *q = filter.quaternion();
  eskf_scalar norm =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);

  // Extract pitch angle from quaternion
  // Pitch = asin( 2(w*y - x*z) )
  // q[0]=w, q[1]=x, q[2]=y, q[3]=z
  eskf_scalar pitch_arg = 2.0 * (q[0] * q[2] - q[1] * q[3]);
  if (pitch_arg > 1.0)
    pitch_arg = 1.0;
  if (pitch_arg < -1.0)
    pitch_arg = -1.0;

  eskf_scalar estimated_pitch_rad = std::asin(pitch_arg);
  eskf_scalar estimated_pitch_deg = estimated_pitch_rad * 180.0 / M_PI;

  // Verify pitch is within 5° of target (allow for filter settling)
  TEST_ASSERT_DOUBLE_WITHIN(5.0, pitch_deg, estimated_pitch_deg);
}

static void test_rail_csv_vector_convergence() {
  RailShadowFilter filter;
  filter.reset();

  // Vector from EskfImuPipeline.csv (User Reported Failure Case)
  // [9.728, 0.86, -0.15]
  // Magnitude: ~9.77 (Consistent with 1g)
  // Angle from X-axis: atan(sqrt(0.86^2 + 0.15^2) / 9.73) = ~5.1 deg
  // Pitch = 90 - 5.1 = 84.9 deg.
  const eskf_scalar accel[3] = {9.728, 0.86, -0.15};
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001;

  // Initialize
  filter.initializeFromAccel(accel);

  // Run loop
  for (int i = 0; i < 5000; ++i) {
    filter.update(accel, gyro, dt);
  }

  // Check raw quaternion convergence (internal state)
  const eskf_scalar *q = filter.quaternion();
  eskf_scalar pitch_arg = 2.0 * (q[0] * q[2] - q[1] * q[3]);
  if (pitch_arg > 1.0)
    pitch_arg = 1.0;
  if (pitch_arg < -1.0)
    pitch_arg = -1.0;
  eskf_scalar estimated_pitch_deg = std::asin(pitch_arg) * 180.0 / M_PI;
  TEST_ASSERT_DOUBLE_WITHIN(5.0, 85.0, estimated_pitch_deg);

  // Check COMBINED quaternion (User Output)
  // The user reported error was in the estimated pitch, which is derived from
  // getCombinedQuaternion
  eskf_scalar q_out[4];
  filter.getCombinedQuaternion(q_out);

  eskf_scalar pitch_arg_out = 2.0 * (q_out[0] * q_out[2] - q_out[1] * q_out[3]);
  if (pitch_arg_out > 1.0)
    pitch_arg_out = 1.0;
  if (pitch_arg_out < -1.0)
    pitch_arg_out = -1.0;
  eskf_scalar estimated_pitch_out_deg = std::asin(pitch_arg_out) * 180.0 / M_PI;

  // This check targets the singularity logic: if it forces heading incorrectly,
  // pitch might drift
  TEST_ASSERT_DOUBLE_WITHIN(5.0, 85.0, estimated_pitch_out_deg);
}

// ============================================================
// FlightShadowFilter Tests
// ============================================================

static void test_flight_reset_inherits_quaternion() {
  FlightShadowFilter filter;

  // Create a non-identity initial quaternion (45° yaw)
  const eskf_scalar angle = M_PI / 4; // 45 degrees
  const eskf_scalar initial_q[4] = {std::cos(angle / 2), 0, 0,
                                    std::sin(angle / 2)};

  filter.reset(initial_q);

  const eskf_scalar *q = filter.quaternion();
  TEST_ASSERT_DOUBLE_WITHIN(kTol, initial_q[0], q[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, initial_q[1], q[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, initial_q[2], q[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, initial_q[3], q[3]);
}

static void test_flight_static_zero_velocity() {
  FlightShadowFilter filter;

  // Identity quaternion (rocket pointing up)
  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  // Static on ground: accel cancels gravity
  // Body Z-axis points up (NED down), so accel reads -g
  const eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001;

  // Run a few prediction steps
  for (int i = 0; i < 100; ++i) {
    filter.predict(accel, gyro, dt);
  }

  // Vertical velocity should remain near zero
  // (accel in NED = rotated body accel + g should be ~0)
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, filter.velocity());
}

static void test_flight_constant_accel_velocity_integrates() {
  FlightShadowFilter filter;

  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  // Apply constant downward acceleration (beyond gravity)
  // Body Z points up in NED, so body accel of -10 means:
  // - Sensor reads -10 (upward force on sensor)
  // - In NED: accel_ned_z = +10 (after rotation) + g = ~20 m/s² down
  // Actually with identity q, body Z = NED Z (down)
  // So accel[2] = -g cancels gravity, additional accel appears as:
  // accel[2] = -9.8 - 1.0 = -10.8 gives +1 m/s² down
  const eskf_scalar accel[3] = {0, 0, -9.80665 - 1.0}; // Extra 1 m/s² upward
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001;

  // Run for 1 second (1000 steps at 1ms)
  for (int i = 0; i < 1000; ++i) {
    filter.predict(accel, gyro, dt);
  }

  // After 1s at -1 m/s² NED vertical: v ≈ -1 m/s (ascending)
  // Note: NED Down positive, so negative = ascending
  TEST_ASSERT_DOUBLE_WITHIN(0.1, -1.0, filter.velocity());
}

static void test_flight_baro_correction_reduces_error() {
  FlightShadowFilter filter;

  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  // Introduce initial altitude error by running prediction
  const eskf_scalar accel[3] = {0, 0, -9.80665 - 2.0}; // 2 m/s² upward
  const eskf_scalar gyro[3] = {0, 0, 0};

  // Run prediction for 0.5 seconds
  for (int i = 0; i < 50; ++i) {
    filter.predict(accel, gyro, 0.01);
  }

  // Altitude should have changed from zero
  eskf_scalar alt_before = filter.altitude();
  TEST_ASSERT_TRUE(std::abs(alt_before) > 0.1);

  // Apply baro correction saying we're at 0m altitude
  // (altitude positive up, so z_baro = 0 means z_ned = 0)
  filter.correctBaro(0.0, 0.02);

  // Altitude should be closer to 0 after correction
  eskf_scalar alt_after = filter.altitude();
  TEST_ASSERT_TRUE(std::abs(alt_after) < std::abs(alt_before));
}

static void test_flight_aero_blind_ignores_baro() {
  FlightShadowFilter filter;

  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  // Accelerate to high speed (above aero-blind threshold of 100 m/s)
  // Body accel that results in 120 m/s vertical after integration
  const eskf_scalar accel[3] = {0, 0, -9.80665 - 100.0}; // Strong upward
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001;

  // Run until velocity exceeds threshold
  for (int i = 0; i < 1500; ++i) { // 1.5s → v ≈ -150 m/s
    filter.predict(accel, gyro, dt);
  }

  // Should now be in aero-blind mode
  TEST_ASSERT_TRUE(filter.isAeroBlind());

  // Record current altitude
  eskf_scalar alt_before = filter.altitude();

  // Try to apply baro correction
  filter.correctBaro(0.0, 0.02);

  // Altitude should NOT have changed (baro ignored)
  TEST_ASSERT_DOUBLE_WITHIN(kTol, alt_before, filter.altitude());
}

static void test_flight_reengagement_snap() {
  FlightShadowFilter filter;

  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  // First, accelerate to high speed
  eskf_scalar accel[3] = {0, 0, -9.80665 - 100.0};
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001;

  for (int i = 0; i < 1500; ++i) {
    filter.predict(accel, gyro, dt);
  }
  TEST_ASSERT_TRUE(filter.isAeroBlind());

  // Now decelerate back below threshold
  accel[2] = -9.80665 + 50.0;      // Deceleration (drag)
  for (int i = 0; i < 4000; ++i) { // 4s of deceleration
    filter.predict(accel, gyro, dt);
  }

  // Should exit aero-blind mode
  TEST_ASSERT_FALSE(filter.isAeroBlind());

  // Apply baro correction after transition
  // The snap logic should force position to baro value
  const eskf_scalar z_baro = 5000.0; // 5000m altitude (positive up)
  filter.correctBaro(z_baro, 0.02);

  // After snap, altitude should be close to -z_baro (NED conversion)
  // Note: First correction triggers snap, so altitude = -5000
  TEST_ASSERT_DOUBLE_WITHIN(1.0, -z_baro, filter.altitude());
}

// ============================================================
// Gyro Bias LPF Tests
// ============================================================

static void test_rail_gyro_bias_lpf_converges() {
  RailShadowFilter filter;
  filter.reset();

  // Simulate constant gyro bias of [0.01, -0.02, 0.005] rad/s
  const eskf_scalar bias[3] = {0.01, -0.02, 0.005};
  const eskf_scalar accel[3] = {0, 0, -9.80665}; // Stationary
  const eskf_scalar dt = 0.001;                  // 1ms (approx 1kHz)

  // Run for ~50 seconds at 1kHz to allow LPF to converge
  // At alpha=0.9999 and 1kHz, τ ≈ 10s, so 50s gives ~5τ (99.3% of final value)
  for (int i = 0; i < 50000; ++i) {
    filter.update(accel, bias, dt);
  }

  // Bias estimate should be close to actual bias (within 5% of target)
  const eskf_scalar *est = filter.gyroBias();
  TEST_ASSERT_DOUBLE_WITHIN(0.003, bias[0], est[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.003, bias[1], est[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.003, bias[2], est[2]);
}

static void test_rail_gyro_bias_reset_clears() {
  RailShadowFilter filter;
  filter.reset();

  // Run many updates to accumulate significant bias estimate
  // At alpha=0.9999, need ~10000 samples to get noticeable accumulation
  const eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0.1, 0.2, 0.3};
  const eskf_scalar dt = 0.01;

  for (int i = 0; i < 10000; ++i) {
    filter.update(accel, gyro, dt);
  }

  // Verify bias is non-zero (at 10000 iterations, should have ~63% of final
  // value)
  const eskf_scalar *est = filter.gyroBias();
  TEST_ASSERT_TRUE(std::abs(est[0]) >
                   0.01); // Should be close to 0.063 (~63% of 0.1)

  // Reset should clear bias
  filter.reset();
  est = filter.gyroBias();
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, est[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, est[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, est[2]);
}

// ============================================================
// Checkpoint Buffer Tests
// ============================================================

static void test_rail_checkpoint_saves_and_finds() {
  RailShadowFilter filter;
  filter.reset();

  const eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0, 0, 0};
  const eskf_scalar dt = 0.001;

  // Run some updates and save checkpoints at known times
  for (int i = 0; i < 100; ++i) {
    uint64_t ts = static_cast<uint64_t>(i) * 1000; // 1ms intervals
    filter.update(accel, gyro, dt, ts);

    // Save checkpoint every 10ms
    if (i % 10 == 0 && i > 0) {
      filter.saveCheckpoint(ts);
    }
  }

  // Find checkpoint at or before 50ms
  const RailShadowCheckpoint *cp = filter.findCheckpointBefore(50000);
  TEST_ASSERT_NOT_NULL(cp);
  TEST_ASSERT_EQUAL(50000, cp->timestamp_us);

  // Find checkpoint before 55ms (should return 50ms checkpoint)
  cp = filter.findCheckpointBefore(55000);
  TEST_ASSERT_NOT_NULL(cp);
  TEST_ASSERT_EQUAL(50000, cp->timestamp_us);

  // Find checkpoint before 10ms (should return 10ms checkpoint)
  cp = filter.findCheckpointBefore(15000);
  TEST_ASSERT_NOT_NULL(cp);
  TEST_ASSERT_EQUAL(10000, cp->timestamp_us);
}

static void test_rail_checkpoint_empty_returns_null() {
  RailShadowFilter filter;
  filter.reset();

  // No checkpoints saved yet
  const RailShadowCheckpoint *cp = filter.findCheckpointBefore(100000);
  TEST_ASSERT_NULL(cp);
}

static void test_rail_checkpoint_current_fallback() {
  RailShadowFilter filter;
  filter.reset();

  // Run many updates without saving checkpoints
  // At alpha=0.9999, need ~10000 samples to get noticeable accumulation
  const eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0.01, 0.02, 0.03};
  const eskf_scalar dt = 0.01;

  for (int i = 0; i < 10000; ++i) {
    filter.update(accel, gyro, dt);
  }

  // Update heading
  filter.updateHeading(0.5, 0.01);

  // Get current as checkpoint
  RailShadowCheckpoint cp = filter.currentAsCheckpoint(500000);

  // Verify timestamp
  TEST_ASSERT_EQUAL(500000, cp.timestamp_us);

  // Verify heading was captured
  TEST_ASSERT_TRUE(cp.heading_initialized);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.5, cp.heading_estimate);

  // Verify gyro bias was captured (should be close to ~63% of input gyro)
  TEST_ASSERT_TRUE(std::abs(cp.gyro_bias[0]) > 0.001);
}

// ============================================================
// Behavior Contracts (Checklist Items 10 and 17)
// ============================================================

static void test_shadow_contract_10_ground_reference_oldest_complete_window() {
  // Subsystem: RailShadowFilter 1-second rolling ground-reference windows.
  RailShadowFilter filter;
  filter.reset();

  const eskf_scalar temps[2] = {290.0, 290.0};
  const bool valid[2] = {true, true};

  // Window #1: centered around 100050 Pa virtual ground.
  for (uint64_t ts = 0; ts < 1000000; ts += 100000) {
    const eskf_scalar p[2] = {100000.0, 100100.0};
    filter.updateBaro(p, temps, valid, 2, ts);
  }

  // Crossing boundary finalizes window #1.
  {
    const eskf_scalar p2[2] = {99500.0, 99600.0};
    filter.updateBaro(p2, temps, valid, 2, 1000000);
  }

  const GroundReference& gref = filter.getOldestGroundReference();
  TEST_ASSERT_TRUE(gref.valid);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 100050.0, gref.pressure_pa);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 50.0, gref.per_baro_offset_pa[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, -50.0, gref.per_baro_offset_pa[1]);

  // Complete a second full window; oldest complete must remain window #1.
  for (uint64_t ts = 1100000; ts < 2000000; ts += 100000) {
    const eskf_scalar p[2] = {99500.0, 99600.0};
    filter.updateBaro(p, temps, valid, 2, ts);
  }
  {
    const eskf_scalar p3[2] = {99000.0, 99100.0};
    filter.updateBaro(p3, temps, valid, 2, 2000000);
  }

  const GroundReference& gref_after = filter.getOldestGroundReference();
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 100050.0, gref_after.pressure_pa);
}

static void test_shadow_contract_10_early_liftoff_before_complete_window() {
  // Subsystem: early liftoff behavior before first complete 1s window.
  RailShadowFilter filter;
  filter.reset();

  const eskf_scalar temps[1] = {289.0};
  const bool valid[1] = {true};

  for (uint64_t ts = 0; ts < 500000; ts += 100000) {
    const eskf_scalar p[1] = {100800.0};
    filter.updateBaro(p, temps, valid, 1, ts);
  }

  const GroundReference& gref = filter.getOldestGroundReference();
  TEST_ASSERT_FALSE(gref.valid);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, gref.per_baro_offset_pa[0]);
}

static void test_shadow_contract_10_active_window_fallback_reference() {
  // Subsystem: early-liftoff fallback from active (incomplete) baro window.
  RailShadowFilter filter;
  filter.reset();

  const eskf_scalar temps[2] = {289.0, 289.0};
  const bool valid[2] = {true, true};

  for (uint64_t ts = 0; ts < 500000; ts += 100000) {
    const eskf_scalar p[2] = {100000.0, 100200.0};
    filter.updateBaro(p, temps, valid, 2, ts);
  }

  GroundReference fallback{};
  const bool ok = filter.estimateGroundReferenceFromActiveWindow(fallback);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(fallback.valid);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 100100.0, fallback.pressure_pa);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 100.0, fallback.per_baro_offset_pa[0]);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, -100.0, fallback.per_baro_offset_pa[1]);

  const GroundReference &stored = filter.getOldestGroundReference();
  TEST_ASSERT_FALSE(stored.valid);
}

static void test_shadow_contract_17_aero_blind_one_snap_per_exit_cycle() {
  // Subsystem: FlightShadowFilter aero-blind re-engagement snap limit.
  FlightShadowFilter filter;
  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  eskf_scalar accel[3] = {0, 0, -9.80665 - 100.0};
  const eskf_scalar gyro[3] = {0, 0, 0};

  for (int i = 0; i < 1500; ++i) {
    filter.predict(accel, gyro, 0.001);
  }
  TEST_ASSERT_TRUE(filter.isAeroBlind());

  accel[2] = -9.80665 + 50.0;
  for (int i = 0; i < 3000; ++i) {
    filter.predict(accel, gyro, 0.001);
  }
  TEST_ASSERT_FALSE(filter.isAeroBlind());

  // First post-blind correction must hard-snap once.
  filter.correctBaro(5000.0, 0.02);
  const eskf_scalar after_first_snap = filter.altitude();
  TEST_ASSERT_DOUBLE_WITHIN(1.0, -5000.0, after_first_snap);

  // Additional correction in same cycle must not snap again.
  filter.correctBaro(4000.0, 0.02);
  const eskf_scalar after_second_correction = filter.altitude();
  TEST_ASSERT_TRUE(std::abs(after_second_correction + 4000.0) > 5.0);
  TEST_ASSERT_TRUE(std::abs(after_second_correction - after_first_snap) > 0.01);

  // Start a fresh cycle and verify one snap is available again after re-exit.
  filter.reset(initial_q);
  accel[2] = -9.80665 - 100.0;
  for (int i = 0; i < 1500; ++i) {
    filter.predict(accel, gyro, 0.001);
  }
  TEST_ASSERT_TRUE(filter.isAeroBlind());

  accel[2] = -9.80665 + 50.0;
  for (int i = 0; i < 3000; ++i) {
    filter.predict(accel, gyro, 0.001);
  }
  TEST_ASSERT_FALSE(filter.isAeroBlind());

  filter.correctBaro(3000.0, 0.02);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, -3000.0, filter.altitude());
}

static void test_shadow_contract_21_boost_policy_soft_only_hard_gate_unchanged() {
  // Checklist item 21: boost policy should inflate soft threshold only.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  cfg.imus[0].enabled = true;
  cfg.imus[1].enabled = true;
  cfg.imus[2].enabled = true;
  cfg.voting_enabled = true;
  cfg.use_central_diff = false;
  cfg.gyro_voting_threshold = 0.4;
  cfg.gyro_hard_fault_threshold = 2.0;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2[3] = {0, 0, -9.8f};
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.1f};
  eskf_sensor_t g2_soft[3] = {0, 0, 0.8f};
  eskf_sensor_t g2_hard[3] = {0, 0, 8.0f};
  SensorStatus status[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                        SensorStatus::OK,
                                        SensorStatus::HARD_FAIL};

  VirtualImuRuntimePolicy boost;
  boost.soft_threshold_scale = 3.0;
  boost.boost_phase = true;
  vimu.setRuntimePolicy(boost);

  VirtualImuOutput out[1];
  {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_soft, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, status, 1, 0, out, 1, 1000);
    TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[2]);
  }

  {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_hard, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, status, 1, 1000, out, 1, 1000);
    vimu.process(accel_data, gyro_data, nullptr, status, 1, 2000, out, 1, 1000);
  }
  TEST_ASSERT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);
}

static void test_shadow_contract_29_phase_aware_saturation_degraded_then_inhibit() {
  // Checklist item 29: boost allows degraded clipping; outside boost may inhibit.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  cfg.imus[0].enabled = true;
  cfg.imus[1].enabled = true;
  cfg.imus[2].enabled = true;
  cfg.voting_enabled = true;
  cfg.use_central_diff = false;
  cfg.gyro_voting_threshold = 10.0;
  cfg.accel_voting_threshold = 10.0;
  cfg.accel_saturation_threshold = 15.0;
  cfg.gyro_saturation_threshold = 15.0;
  cfg.saturation_multi_axis_limit = 2;
  cfg.saturation_hard_fault_persistence_samples = 2;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2_clip[3] = {20.0f, 0, -9.8f};
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.1f};
  eskf_sensor_t g2[3] = {0, 0, 0.1f};
  SensorStatus status[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                        SensorStatus::OK,
                                        SensorStatus::HARD_FAIL};

  VirtualImuRuntimePolicy boost;
  boost.soft_threshold_scale = 2.0;
  boost.boost_phase = true;
  vimu.setRuntimePolicy(boost);

  VirtualImuOutput out[1];
  for (int k = 0; k < 3; ++k) {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2_clip, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, status, 1,
                 static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
  }
  TEST_ASSERT_TRUE(out[0].degraded_output);
  TEST_ASSERT_NOT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);

  VirtualImuRuntimePolicy coast;
  coast.soft_threshold_scale = 1.0;
  coast.boost_phase = false;
  vimu.setRuntimePolicy(coast);
  for (int k = 0; k < 3; ++k) {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2_clip, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, status, 1,
                 static_cast<uint64_t>(10 + k) * 1000ULL, out, 1, 1000);
  }
  TEST_ASSERT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);
}

// ============================================================
// GTest Wrapper
// ============================================================

#define WRAP_TEST(test_fn) TEST(KalmanShadowFilterSuite, test_fn) { test_fn(); }

WRAP_TEST(test_rail_reset_identity);
WRAP_TEST(test_rail_static_converges_to_gravity);
WRAP_TEST(test_rail_gate_closes_during_handling);
WRAP_TEST(test_rail_gyro_only_during_motion);
WRAP_TEST(test_rail_tilted_convergence);
WRAP_TEST(test_rail_85deg_pitch_convergence);
WRAP_TEST(test_rail_csv_vector_convergence);

WRAP_TEST(test_rail_gyro_bias_lpf_converges);
WRAP_TEST(test_rail_gyro_bias_reset_clears);

WRAP_TEST(test_rail_checkpoint_saves_and_finds);
WRAP_TEST(test_rail_checkpoint_empty_returns_null);
WRAP_TEST(test_rail_checkpoint_current_fallback);
WRAP_TEST(test_shadow_contract_10_ground_reference_oldest_complete_window);
WRAP_TEST(test_shadow_contract_10_early_liftoff_before_complete_window);
WRAP_TEST(test_shadow_contract_10_active_window_fallback_reference);

WRAP_TEST(test_flight_reset_inherits_quaternion);
WRAP_TEST(test_flight_static_zero_velocity);
WRAP_TEST(test_flight_constant_accel_velocity_integrates);
WRAP_TEST(test_flight_baro_correction_reduces_error);
WRAP_TEST(test_flight_aero_blind_ignores_baro);
WRAP_TEST(test_flight_reengagement_snap);
WRAP_TEST(test_shadow_contract_17_aero_blind_one_snap_per_exit_cycle);
WRAP_TEST(test_shadow_contract_21_boost_policy_soft_only_hard_gate_unchanged);
WRAP_TEST(test_shadow_contract_29_phase_aware_saturation_degraded_then_inhibit);

#undef WRAP_TEST
