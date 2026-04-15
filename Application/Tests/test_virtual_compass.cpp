// Virtual Compass Unit Tests - Magnetometer Calibration
// Tests Hard Iron, Soft Iron, Declination, and Magnitude Rejection
// Also tests tilt-compensated heading calculation via eskf_math helpers.

#include "Application/Tests/unity_gtest_compat.hpp"

#include <cmath>

#include "Application/Kalman/kalman/preprocessor/virtual_compass.hpp"
#include "Application/Kalman/kalman/eskf_math.hpp"

using namespace eskf;

constexpr eskf_scalar kTol = 1e-6;
constexpr eskf_scalar kTolF = 1e-4;
constexpr eskf_scalar kDegToRad = 3.14159265358979323846 / 180.0;

// ============================================================
// Identity Calibration (Passthrough)
// ============================================================

static void test_identity_calibration_passthrough() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  // Default calibration: zero bias, identity soft iron, zero declination
  
  compass.configure(cfg);
  
  eskf_sensor_t raw_mag[3] = {30.0f, 10.0f, 40.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // Should pass through unchanged
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 30.0, out.mag_calibrated[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 10.0, out.mag_calibrated[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 40.0, out.mag_calibrated[2]);
  TEST_ASSERT_TRUE(out.valid);
}

// ============================================================
// Hard Iron Correction
// ============================================================

static void test_hard_iron_correction() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  // Set hard iron bias
  cfg.calibration.hard_iron_bias[0] = 5.0;
  cfg.calibration.hard_iron_bias[1] = -3.0;
  cfg.calibration.hard_iron_bias[2] = 2.0;
  cfg.magnitude_check_enabled = false;  // Disable for this test
  
  compass.configure(cfg);
  
  // Raw reading with bias offset
  eskf_sensor_t raw_mag[3] = {35.0f, 7.0f, 42.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // After bias removal: (35-5, 7-(-3), 42-2) = (30, 10, 40)
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 30.0, out.mag_calibrated[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 10.0, out.mag_calibrated[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 40.0, out.mag_calibrated[2]);
}

// ============================================================
// Soft Iron Correction
// ============================================================

static void test_soft_iron_correction() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  // Set soft iron matrix (scale X by 2, Y by 0.5)
  cfg.calibration.soft_iron_matrix[0][0] = 2.0;
  cfg.calibration.soft_iron_matrix[1][1] = 0.5;
  cfg.calibration.soft_iron_matrix[2][2] = 1.0;
  cfg.magnitude_check_enabled = false;
  
  compass.configure(cfg);
  
  eskf_sensor_t raw_mag[3] = {15.0f, 20.0f, 40.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // After soft iron: (15*2, 20*0.5, 40*1) = (30, 10, 40)
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 30.0, out.mag_calibrated[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 10.0, out.mag_calibrated[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 40.0, out.mag_calibrated[2]);
}

// ============================================================
// Combined Hard + Soft Iron
// ============================================================

static void test_hard_and_soft_iron_combined() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  // Hard iron
  cfg.calibration.hard_iron_bias[0] = 10.0;
  cfg.calibration.hard_iron_bias[1] = 5.0;
  cfg.calibration.hard_iron_bias[2] = 0.0;
  
  // Soft iron (scale)
  cfg.calibration.soft_iron_matrix[0][0] = 0.5;
  cfg.calibration.soft_iron_matrix[1][1] = 2.0;
  cfg.calibration.soft_iron_matrix[2][2] = 1.0;
  cfg.magnitude_check_enabled = false;
  
  compass.configure(cfg);
  
  // Raw: (60, 25, 40)
  // After hard iron: (60-10, 25-5, 40-0) = (50, 20, 40)
  // After soft iron: (50*0.5, 20*2, 40*1) = (25, 40, 40)
  eskf_sensor_t raw_mag[3] = {60.0f, 25.0f, 40.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 25.0, out.mag_calibrated[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 40.0, out.mag_calibrated[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 40.0, out.mag_calibrated[2]);
}

// ============================================================
// Declination Correction
// ============================================================

// ============================================================
// Declination Correction
// ============================================================

static void test_declination_not_applied_in_compass() {
  // Declination is no longer applied in VirtualCompass.
  // It's applied in calculateTiltCompensatedHeading() after Earth-frame rotation.
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  // 45 degrees East declination (stored but not applied here)
  const eskf_scalar declination = 45.0 * kDegToRad;
  (void)declination;
  cfg.magnitude_check_enabled = false;
  
  compass.configure(cfg);
  
  // Pure magnetic north field (all in X)
  eskf_sensor_t raw_mag[3] = {50.0f, 10.0f, 30.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // Output should be unchanged (declination NOT applied in body frame)
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 50.0, out.mag_calibrated[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 10.0, out.mag_calibrated[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 30.0, out.mag_calibrated[2]);
  
  // Verify declination is accessible for downstream use (via TuningConfig, but here we just check our local var)
  // The VirtualCompassConfig no longer stores declination, so we can't test it on config()
}

static void test_declination_heading_shift() {
  // Test that declination correction works via the tilt-compensated heading helper
  // 10 degrees East declination
  eskf_scalar declination = 10.0 * kDegToRad;
  eskf_scalar q_identity[4] = {1, 0, 0, 0};
  
  // Pointing due magnetic east (90° magnetic heading)
  // Body frame: +Y is East when level
  eskf_scalar mag_body[3] = {0.0, 50.0, 0.0};
  
  // With 10° East declination, magnetic East (90°) becomes True East-Southeast (~80°)
  // True = Magnetic + Declination, so 90° magnetic becomes 100° true? 
  // No wait - the formula in calculateTiltCompensatedHeading adds declination.
  // If we're pointing at magnetic East and declination is +10 (East positive),
  // the heading should be 90 + 10 = 100° true.
  // But the old test expected 80°, which follows the opposite convention.
  // Let's verify: declination East positive means True North is East of Magnetic North.
  // So Magnetic East (90° from Mag North) is further East from True North = 90 + 10 = 100°.
  eskf_scalar heading = math::calculateTiltCompensatedHeading(mag_body, q_identity, declination);
  
  // Expected: 90° + 10° = 100° true heading
  TEST_ASSERT_DOUBLE_WITHIN(2.0 * kDegToRad, 100.0 * kDegToRad, heading);
}

// ============================================================
// Magnitude Validation
// ============================================================

static void test_magnitude_rejection_field_too_strong() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  cfg.expected_magnitude_ut = 50.0;
  cfg.magnitude_threshold_ut = 10.0;  // Accept 40-60 μT
  cfg.magnitude_check_enabled = true;
  
  compass.configure(cfg);
  
  // Strong field (launch rail steel): |B| ≈ 100 μT
  eskf_sensor_t raw_mag[3] = {60.0f, 60.0f, 60.0f};  // |B| ≈ 103.9
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // Should be rejected
  TEST_ASSERT_FALSE(out.valid);
}

static void test_magnitude_acceptance_field_normal() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  cfg.expected_magnitude_ut = 50.0;
  cfg.magnitude_threshold_ut = 10.0;  // Accept 40-60 μT
  cfg.magnitude_check_enabled = true;
  
  compass.configure(cfg);
  
  // Normal field: |B| = sqrt(30² + 20² + 30²) ≈ 46.9 μT
  eskf_sensor_t raw_mag[3] = {30.0f, 20.0f, 30.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // Should be accepted
  TEST_ASSERT_TRUE(out.valid);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 46.9, out.magnitude_ut);
}

static void test_magnitude_check_disabled_accepts_all() {
  VirtualCompass compass;
  VirtualCompassConfig cfg{};
  
  cfg.expected_magnitude_ut = 50.0;
  cfg.magnitude_threshold_ut = 1.0;  // Very tight
  cfg.magnitude_check_enabled = false;  // DISABLED
  
  compass.configure(cfg);
  
  // Way off field
  eskf_sensor_t raw_mag[3] = {100.0f, 100.0f, 100.0f};
  CompassOutput out = compass.process(raw_mag, 1000);
  
  // Should be accepted anyway
  TEST_ASSERT_TRUE(out.valid);
}

// ============================================================
// Tilt-Compensated Heading (via eskf_math helper)
// ============================================================

static void test_tilt_compensated_heading_level_north() {
  // Level orientation (identity quaternion), mag pointing North
  eskf_scalar q_identity[4] = {1, 0, 0, 0};
  eskf_scalar mag_body[3] = {50.0, 0.0, 30.0};  // North + Down dip
  
  eskf_scalar heading = math::calculateTiltCompensatedHeading(mag_body, q_identity, 0.0);
  
  // Should be ~0 (North)
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, heading);
}

static void test_tilt_compensated_heading_level_east() {
  // Level orientation, mag pointing East  
  eskf_scalar q_identity[4] = {1, 0, 0, 0};
  eskf_scalar mag_body[3] = {0.0, 50.0, 30.0};  // East + Down dip
  
  eskf_scalar heading = math::calculateTiltCompensatedHeading(mag_body, q_identity, 0.0);
  
  // Should be ~90° (East)
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 90.0 * kDegToRad, heading);
}

static void test_tilt_compensated_heading_pitched_up() {
  // Pitched up 80° (rocket nearly vertical), still pointing North
  // q = axis-angle rotation of 80° about Y-axis
  eskf_scalar pitch_rad = 80.0 * kDegToRad;
  eskf_scalar q_pitched[4];
  eskf_scalar axis_y[3] = {0, 1, 0};
  math::quatFromAxisAngle(q_pitched, axis_y, pitch_rad);
  
  // Quaternion convention: q rotates body→NED, so quatRotateVector(q, v_body) = v_NED
  // When pitched up 80° about Y:
  // - Body +X rotates toward NED -Z (up in world)
  // - Body +Z rotates toward NED +X (North)
  // 
  // Earth's mag field has horizontal North component.
  // In body frame, North appears mostly in body +Z (since body Z→NED X)
  eskf_scalar mag_body[3] = {10.0, 0.0, 50.0};  // North in body +Z
  
  eskf_scalar heading = math::calculateTiltCompensatedHeading(mag_body, q_pitched, 0.0);
  
  // After rotating to Earth frame, should resolve to ~0 (North)
  TEST_ASSERT_DOUBLE_WITHIN(0.3, 0.0, heading);
}

static void test_tilt_compensated_heading_vertical_90deg() {
  // Critical test: Rocket perfectly vertical (90° pitch) - the gimbal lock case
  // Old Euler-based code would fail here, new quaternion-based code should work.
  eskf_scalar pitch_rad = 90.0 * kDegToRad;
  eskf_scalar q_vertical[4];
  eskf_scalar axis_y[3] = {0, 1, 0};
  math::quatFromAxisAngle(q_vertical, axis_y, pitch_rad);
  
  // At 90° pitch about Y:
  // - Body +X points straight UP (NED -Z)
  // - Body +Z points exactly North (NED +X)
  // - Body Y is unchanged (East/NED +Y)
  //
  // Earth's horizontal mag field (pointing North) appears entirely in body +Z
  // Vertical component (down in NED +Z) appears in body -X
  eskf_scalar mag_body[3] = {-30.0, 0.0, 50.0};  // Down comp in -X, North in +Z
  
  eskf_scalar heading = math::calculateTiltCompensatedHeading(mag_body, q_vertical, 0.0);
  
  // After rotating to Earth frame, should resolve to ~0 (North)
  // This is THE critical test - gimbal lock fix verification
  TEST_ASSERT_DOUBLE_WITHIN(0.3, 0.0, heading);
}

static void test_tilt_compensated_heading_with_declination() {
  // Level, mag pointing Magnetic North, 10° East declination
  eskf_scalar q_identity[4] = {1, 0, 0, 0};
  eskf_scalar mag_body[3] = {50.0, 0.0, 30.0};
  eskf_scalar declination = 10.0 * kDegToRad;
  
  eskf_scalar heading = math::calculateTiltCompensatedHeading(mag_body, q_identity, declination);
  
  // True North = Magnetic + Declination, so heading shifts by +10°
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 10.0 * kDegToRad, heading);
}

// ============================================================
// GTest Wrapper
// ============================================================

#define WRAP_TEST(test_fn) TEST(KalmanVirtualCompassSuite, test_fn) { test_fn(); }

WRAP_TEST(test_identity_calibration_passthrough);
WRAP_TEST(test_hard_iron_correction);
WRAP_TEST(test_soft_iron_correction);
WRAP_TEST(test_hard_and_soft_iron_combined);
WRAP_TEST(test_declination_not_applied_in_compass);
WRAP_TEST(test_declination_heading_shift);
WRAP_TEST(test_magnitude_rejection_field_too_strong);
WRAP_TEST(test_magnitude_acceptance_field_normal);
WRAP_TEST(test_magnitude_check_disabled_accepts_all);
WRAP_TEST(test_tilt_compensated_heading_level_north);
WRAP_TEST(test_tilt_compensated_heading_level_east);
WRAP_TEST(test_tilt_compensated_heading_pitched_up);
WRAP_TEST(test_tilt_compensated_heading_vertical_90deg);
WRAP_TEST(test_tilt_compensated_heading_with_declination);

#undef WRAP_TEST
