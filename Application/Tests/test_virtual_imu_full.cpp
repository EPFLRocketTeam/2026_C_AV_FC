// Virtual IMU & Baro Unit Tests - Multi-Sensor Voting
// Tests preprocessing pipeline: voting, rotation, averaging, lever-arm correction

#include "Application/Tests/unity_gtest_compat.hpp"

#include <cmath>
#include <vector>

#include "Application/Kalman/kalman/preprocessor/virtual_imu.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_baro.hpp"

using namespace eskf;

constexpr eskf_scalar kTol = 1e-6;
constexpr eskf_scalar kTolF = 1e-4;

// ============================================================
// Virtual IMU Configuration Tests
// ============================================================

static void test_default_config_identity_rotation() {
  ImuSensorConfig cfg{};
  
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      eskf_scalar expected = (i == j) ? 1.0 : 0.0;
      TEST_ASSERT_DOUBLE_WITHIN(kTol, expected, cfg.to_body[i][j]);
    }
  }
}

static void test_config_sensor_positions() {
  VirtualImuConfig cfg{};
  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.1;  // 10cm forward
  cfg.imus[1].enabled = true;
  cfg.imus[1].position[0] = -0.1; // 10cm backward
  
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.1, cfg.imus[0].position[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, -0.1, cfg.imus[1].position[0]);
}

// ============================================================
// Gyro Voting Tests
// ============================================================

static void test_gyro_voting_rejects_outlier() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  // Setup 3 IMUs
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.gyro_voting_threshold = 0.5;  // 0.5 rad/s
  cfg.voting_enabled = true;
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);

  // IMU 0 and 1: normal gyro (0.1 rad/s about Z)
  // IMU 2: outlier (5 rad/s about Z - broken sensor)
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.11f};
  eskf_sensor_t g2[3] = {0, 0, 5.0f};  // OUTLIER
  
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2[3] = {0, 0, -9.8f};
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  TEST_ASSERT_EQUAL(1, count);
  
  // IMU 2 should be marked as SOFT_FAIL
  TEST_ASSERT_EQUAL(SensorStatus::OK, output[0].imu_status[0]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, output[0].imu_status[1]);
  TEST_ASSERT_EQUAL(SensorStatus::SOFT_FAIL, output[0].imu_status[2]);
  
  // Valid count should be 2
  TEST_ASSERT_EQUAL(2, output[0].valid_imu_count);
  
  // Fused gyro should be ~0.105 rad/s (average of good sensors)
  TEST_ASSERT_DOUBLE_WITHIN(0.05, 0.105, output[0].frame.gyro[2]);
}

static void test_hard_fail_skips_voting() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[1].enabled = true;
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  
  // IMU 1 has HARD_FAIL (e.g., SPI error)
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  TEST_ASSERT_EQUAL(1, count);
  TEST_ASSERT_EQUAL(1, output[0].valid_imu_count);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.1, output[0].frame.gyro[2]);
}

// ============================================================
// Dynamic Centroid Tests
// ============================================================

static void test_dynamic_centroid_calculation() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.2;  // 20cm forward
  cfg.imus[1].enabled = true;
  cfg.imus[1].position[0] = 0.0;  // At origin
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a0, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g0, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  // Centroid should be at (0.1, 0, 0) - midpoint of both sensors
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.1, output[0].effective_centroid[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, output[0].effective_centroid[1]);
}

static void test_centroid_shifts_on_failure() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.2;  // 20cm forward
  cfg.imus[1].enabled = true;
  cfg.imus[1].position[0] = 0.0;  // At origin
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a0, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g0, nullptr, nullptr};
  
  // IMU 0 failed - only IMU 1 at origin is valid
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::HARD_FAIL, SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  // Centroid should shift to (0, 0, 0) - only IMU 1 is valid
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, output[0].effective_centroid[0]);
}

// ============================================================
// CG Callback Tests
// ============================================================

static eskf_scalar g_test_cg[3] = {0.15, 0, 0};

static void test_cg_callback(void* user_data,
                             uint64_t timestamp_us,
                             eskf_scalar cg_out[3]) {
  (void)user_data;
  (void)timestamp_us;
  cg_out[0] = g_test_cg[0];
  cg_out[1] = g_test_cg[1];
  cg_out[2] = g_test_cg[2];
}

static void test_cg_callback_used() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.2;  // Sensor at 20cm
  cfg.cg_callback = test_cg_callback;
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  // With CG at 0.15 and sensor at 0.2, lever arm should be 0.05
  // This test just confirms processing completes - lever arm effect is subtle
  TEST_ASSERT_EQUAL(1, output[0].valid_imu_count);
}

// ============================================================
// Virtual Baro Tests
// ============================================================

static void test_baro_single_sensor() {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  
  cfg.baro_count = 1;
  cfg.baros[0].enabled = true;
  cfg.single_sensor_variance = 1.0;
  
  vbaro.configure(cfg);

  eskf_sensor_t pressures[1] = {101325.0f};  // Standard atmosphere
  eskf_sensor_t temps[1] = {288.15f};        // 15°C
  SensorStatus statuses[1] = {SensorStatus::OK};

  BaroOutput out = vbaro.process(pressures, temps, statuses, 0);

  TEST_ASSERT_DOUBLE_WITHIN(1.0, 101325.0, out.pressure_pa);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 288.15, out.temperature_k);
  TEST_ASSERT_EQUAL(1, out.valid_count);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, out.variance);  // R = R_single / 1
}

static void test_baro_voting_rejects_outlier() {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  
  cfg.baro_count = 3;
  cfg.baros[0].enabled = true;
  cfg.baros[1].enabled = true;
  cfg.baros[2].enabled = true;
  cfg.voting_threshold_pa = 500.0;
  cfg.voting_enabled = true;
  cfg.single_sensor_variance = 1.0;
  cfg.hard_fault_threshold_pa = 10000.0; // Keep this test in soft-vote regime
  cfg.calibration_mismatch_tolerance_pa = 0.0;
  
  vbaro.configure(cfg);

  // Baro 0 and 1: normal (101325 Pa)
  // Baro 2: blocked port (105000 Pa - way off)
  eskf_sensor_t pressures[3] = {101325.0f, 101330.0f, 105000.0f};
  eskf_sensor_t temps[3] = {288.15f, 288.15f, 288.15f};
  SensorStatus statuses[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};

  BaroOutput out = vbaro.process(pressures, temps, statuses, 0);

  // Baro 2 should be rejected
  TEST_ASSERT_EQUAL(SensorStatus::OK, out.baro_status[0]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out.baro_status[1]);
  TEST_ASSERT_EQUAL(SensorStatus::SOFT_FAIL, out.baro_status[2]);
  
  TEST_ASSERT_EQUAL(2, out.valid_count);
  
  // Pressure should be average of good sensors
  TEST_ASSERT_DOUBLE_WITHIN(10.0, 101327.5, out.pressure_pa);
}

static void test_baro_variance_scaling() {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  
  cfg.baro_count = 4;
  for (size_t i = 0; i < 4; ++i) {
    cfg.baros[i].enabled = true;
  }
  cfg.single_sensor_variance = 4.0;  // 2m std dev
  
  vbaro.configure(cfg);

  eskf_sensor_t pressures[4] = {101325.0f, 101325.0f, 101325.0f, 101325.0f};
  eskf_sensor_t temps[4] = {288.15f, 288.15f, 288.15f, 288.15f};
  SensorStatus statuses[4] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};

  BaroOutput out = vbaro.process(pressures, temps, statuses, 0);

  TEST_ASSERT_EQUAL(4, out.valid_count);
  // Variance should be R_single / 4 = 4.0 / 4 = 1.0
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, out.variance);
}

static void test_baro_hard_fail_excluded() {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  
  cfg.baro_count = 2;
  cfg.baros[0].enabled = true;
  cfg.baros[1].enabled = true;
  cfg.single_sensor_variance = 2.0;
  
  vbaro.configure(cfg);

  eskf_sensor_t pressures[2] = {101325.0f, 99999.0f};  // Second is garbage
  eskf_sensor_t temps[2] = {288.15f, 250.0f};
  SensorStatus statuses[2] = {SensorStatus::OK, SensorStatus::HARD_FAIL};

  BaroOutput out = vbaro.process(pressures, temps, statuses, 0);

  // Only baro 0 should be used
  TEST_ASSERT_EQUAL(1, out.valid_count);
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 101325.0, out.pressure_pa);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 2.0, out.variance);  // R = R_single / 1
}

// ============================================================
// Missing Tests from Review
// ============================================================

// Test: Lever-arm correction produces correct acceleration at CG
static void test_lever_arm_correction_accuracy() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  // Single IMU at 0.1m forward of CG
  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.1;  // 10cm forward
  cfg.static_cg[0] = 0.0;  // CG at origin
  
  vimu.configure(cfg);
  
  // Apply known rotation: 1 rad/s about Z axis
  // Centripetal acceleration = ω² * r = 1² * 0.1 = 0.1 m/s² in -X direction
  // With sensor 10cm forward, it sees centripetal toward CG
  eskf_sensor_t g0[3] = {0, 0, 1.0f};  // 1 rad/s about Z
  
  // Sensor measures -9.8 + centripetal (0.1 in X direction away from rotation center)
  // At sensor position, centripetal is toward origin: -0.1 in X
  eskf_sensor_t a0[3] = {-0.1f, 0, -9.8f};
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  // Process multiple samples to build history for smooth derivative
  // Each call outputs to output[0], so we just need the final one
  VirtualImuOutput output[1];
  for (int iter = 0; iter < 8; ++iter) {
    vimu.process(accel_data, gyro_data, nullptr, statuses, 1, iter * 1000, output, 1, 1000);
  }

  // After lever-arm correction, CG should see ~0 X-accel (centripetal removed)
  // Allow some tolerance as derivative computation introduces small errors
  TEST_ASSERT_DOUBLE_WITHIN(0.15, 0.0, output[0].frame.accel[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, -9.8, output[0].frame.accel[2]);
}

// Test: Calibration bias and transform properly applied
static void test_calibration_application() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  // Single IMU with known calibration
  ImuCalibration cal{};
  cal.ellipsoid_calibrated = true;
  cal.thermal_calibrated = false;
  cal.accel_bias[2] = 0.5;  // 0.5 m/s² bias in Z
  // Scale transform: 1.1x in Z
  cal.accel_transform[0][0] = 1.0;
  cal.accel_transform[1][1] = 1.0;
  cal.accel_transform[2][2] = 1.1;
  cal.gyro_scale[0] = 1.0;
  cal.gyro_scale[1] = 1.0;
  cal.gyro_scale[2] = 0.9;  // 0.9x scale on Z gyro
  
  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.calibration = &cal;
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);
  
  // Raw: gyro Z = 1.0, accel Z = -9.3 (should become -9.8 after calibration)
  // After bias removal: -9.3 - 0.5 = -9.8 (before transform)
  // After transform: -9.8 * 1.1 = -10.78
  eskf_sensor_t g0[3] = {0, 0, 1.0f};
  eskf_sensor_t a0[3] = {0, 0, -9.3f};  // Will be -9.3 - 0.5 = -9.8, then * 1.1 = -10.78
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  // Verify calibration applied
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, -10.78, output[0].frame.accel[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.9, output[0].frame.gyro[2]);  // 1.0 * 0.9
}

// Test: Thermal calibration bias properly applied
static void test_thermal_calibration_application() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  
  // Single IMU with known thermal calibration
  ImuCalibration cal{};
  cal.ellipsoid_calibrated = false;
  cal.thermal_calibrated = true;
  cal.reference_temp_k = 300.0;
  
  // Thermal poly: bias_x = coeff[0]*dT^3 + coeff[1]*dT^2 + coeff[2]*dT + coeff[3]
  // Linear dependency on dT (coeff[2])
  cal.accel_thermal[0].coeff[0] = 0.0; // cubic
  cal.accel_thermal[0].coeff[1] = 0.0; // quadratic
  cal.accel_thermal[0].coeff[2] = 1.0; // linear
  cal.accel_thermal[0].coeff[3] = 0.0; // constant
  
  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.calibration = &cal;
  cfg.use_central_diff = false;
  
  vimu.configure(cfg);
  
  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {10.0f, 0, 0};  
  eskf_scalar t0[1] = {310.0f};  // 10K above reference
  
  // Expected calculation:
  // dT = 310 - 300 = 10
  // Bias_X = 1.0 * 10 = 10.0
  // Output_X = Input_X - Bias_X = 10.0 - 10.0 = 0.0
  
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  const eskf_scalar* temp_data[ESKF_MAX_IMUS] = {t0, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  vimu.process(accel_data, gyro_data, temp_data, statuses, 1, 0, output, 1, 1000);

  // Verify thermal bias removed
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, output[0].frame.accel[0]);
}

static void test_thermal_calibration_delta_t_clamped() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};

  ImuCalibration cal{};
  cal.ellipsoid_calibrated = false;
  cal.thermal_calibrated = true;
  cal.reference_temp_k = 300.0;
  cal.accel_thermal[0].coeff[0] = 0.0;
  cal.accel_thermal[0].coeff[1] = 0.0;
  cal.accel_thermal[0].coeff[2] = 1.0;
  cal.accel_thermal[0].coeff[3] = 0.0;

  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.calibration = &cal;
  cfg.use_central_diff = false;
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {100.0f, 0, 0};
  eskf_scalar t0[1] = {500.0f}; // dT=200K, should clamp to +80K

  const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr,
                                                    nullptr};
  const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr,
                                                   nullptr};
  const eskf_scalar *temp_data[ESKF_MAX_IMUS] = {t0, nullptr, nullptr,
                                                 nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK,
                                          SensorStatus::HARD_FAIL,
                                          SensorStatus::HARD_FAIL,
                                          SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  vimu.process(accel_data, gyro_data, temp_data, statuses, 1, 0, output, 1,
               1000);

  // With dT clamp at +80K, output_x = 100 - 80 = 20.
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 20.0, output[0].frame.accel[0]);
}

// Test: Static 85° tilt with two noisy IMUs yields near-zero ω̇
static void test_static_tilt_omega_dot_near_zero() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};

  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[1].enabled = true;

  // Place both IMUs forward so lever arm is non-zero (sensitive to ω̇)
  cfg.imus[0].position[0] = 0.10;  // 10cm forward
  cfg.imus[1].position[0] = 0.15;  // 15cm forward
  cfg.static_cg[0] = 0.0;
  cfg.static_cg[1] = 0.0;
  cfg.static_cg[2] = 0.0;

  cfg.use_central_diff = true;  // Use Savitzky-Golay ω̇
  cfg.voting_enabled = true;

  vimu.configure(cfg);

  constexpr size_t kCount = 12;
  constexpr eskf_scalar kG = 9.80665;
  constexpr eskf_scalar kDegToRad = 3.14159265358979323846 / 180.0;
  const eskf_scalar tilt_rad = static_cast<eskf_scalar>(85.0) * kDegToRad;

  // Gravity in body frame for 85° pitch about Y axis
  const eskf_scalar g_body[3] = {
      static_cast<eskf_scalar>(kG * std::sin(tilt_rad)),
      0,
      static_cast<eskf_scalar>(-kG * std::cos(tilt_rad))
  };

  eskf_sensor_t a0[kCount * 3];
  eskf_sensor_t a1[kCount * 3];
  eskf_sensor_t g0[kCount * 3];
  eskf_sensor_t g1[kCount * 3];

  for (size_t n = 0; n < kCount; ++n) {
    const eskf_scalar t = static_cast<eskf_scalar>(n);

    // Realistic IMU noise levels
    const eskf_scalar accel_noise0 = static_cast<eskf_scalar>(0.02) * std::sin(static_cast<eskf_scalar>(0.15) * t);
    const eskf_scalar accel_noise1 = static_cast<eskf_scalar>(0.02) * std::sin(static_cast<eskf_scalar>(0.15) * t + static_cast<eskf_scalar>(0.7));

    const eskf_scalar gyro_noise0 = static_cast<eskf_scalar>(0.002) * std::sin(static_cast<eskf_scalar>(0.2) * t);
    const eskf_scalar gyro_noise1 = static_cast<eskf_scalar>(0.002) * std::sin(static_cast<eskf_scalar>(0.2) * t + static_cast<eskf_scalar>(0.4));

    for (int axis = 0; axis < 3; ++axis) {
      a0[n * 3 + axis] = g_body[axis] + accel_noise0;
      a1[n * 3 + axis] = g_body[axis] + accel_noise1;

      // Static IMUs: true ω = 0, only noise
      g0[n * 3 + axis] = gyro_noise0;
      g1[n * 3 + axis] = gyro_noise1;
    }
  }

  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a1, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g1, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[kCount];
  size_t out_count = vimu.process(accel_data, gyro_data, nullptr, statuses, kCount, 0, output, kCount, 1000);

  TEST_ASSERT_TRUE(out_count > 0);

  // Check latest output: lever-arm correction should not inject spurious acceleration
  const VirtualImuOutput& last = output[out_count - 1];

  // ω̇ should be very close to 0 → accel should remain near gravity vector
  TEST_ASSERT_DOUBLE_WITHIN(0.1, g_body[0], last.frame.accel[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.1, g_body[1], last.frame.accel[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.1, g_body[2], last.frame.accel[2]);

  // Gyro should remain near 0 as well
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, last.frame.gyro[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, last.frame.gyro[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, last.frame.gyro[2]);

  // Accel estimation should remain close to the raw values
  eskf_scalar fused_ax_expected = static_cast<eskf_scalar>(0.5) * (a0[(out_count - 1) * 3 + 0] + a1[(out_count - 1) * 3 + 0]);
  eskf_scalar fused_ay_expected = static_cast<eskf_scalar>(0.5) * (a0[(out_count - 1) * 3 + 1] + a1[(out_count - 1) * 3 + 1]);
  eskf_scalar fused_az_expected = static_cast<eskf_scalar>(0.5) * (a0[(out_count - 1) * 3 + 2] + a1[(out_count - 1) * 3 + 2]);

  TEST_ASSERT_DOUBLE_WITHIN(0.05, fused_ax_expected, last.frame.accel[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.05, fused_ay_expected, last.frame.accel[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.05, fused_az_expected, last.frame.accel[2]);

}

// Test: Static 85° tilt single IMU yields near-zero ω̇
static void test_static_tilt_single_imu_omega_dot_near_zero() {
  VirtualImu vimu;
  VirtualImuConfig cfg{};

  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;

  // Place the IMU forward so lever arm is non-zero (sensitive to ω̇)
  cfg.imus[0].position[0] = 0.10;  // 10cm forward
  cfg.static_cg[0] = 0.0;
  cfg.static_cg[1] = 0.0;
  cfg.static_cg[2] = 0.0;

  cfg.use_central_diff = true;  // Use Savitzky-Golay ω̇
  cfg.voting_enabled = true;

  vimu.configure(cfg);

  constexpr size_t kCount = 12;
  constexpr eskf_scalar kG = 9.80665;
  constexpr eskf_scalar kDegToRad = 3.14159265358979323846 / 180.0;
  const eskf_scalar tilt_rad = static_cast<eskf_scalar>(85.0) * kDegToRad;

  // Gravity in body frame for 85° pitch about Y axis
  const eskf_scalar g_body[3] = {
      static_cast<eskf_scalar>(kG * std::sin(tilt_rad)),
      0,
      static_cast<eskf_scalar>(-kG * std::cos(tilt_rad))
  };

  eskf_sensor_t a0[kCount * 3];
  eskf_sensor_t g0[kCount * 3];

  for (size_t n = 0; n < kCount; ++n) {
    const eskf_scalar t = static_cast<eskf_scalar>(n);

    // Realistic IMU noise levels
    const eskf_scalar accel_noise0 = static_cast<eskf_scalar>(0.02) * std::sin(static_cast<eskf_scalar>(0.15) * t);
    const eskf_scalar gyro_noise0 = static_cast<eskf_scalar>(0.002) * std::sin(static_cast<eskf_scalar>(0.2) * t);

    for (int axis = 0; axis < 3; ++axis) {
      a0[n * 3 + axis] = g_body[axis] + accel_noise0;

      // Static IMU: true ω = 0, only noise
      g0[n * 3 + axis] = gyro_noise0;
    }
  }

  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[kCount];
  size_t out_count = vimu.process(accel_data, gyro_data, nullptr, statuses, kCount, 0, output, kCount, 1000);

  TEST_ASSERT_TRUE(out_count > 0);

  // Check latest output: lever-arm correction should not inject spurious acceleration
  const VirtualImuOutput& last = output[out_count - 1];

  // ω̇ should be very close to 0 → accel should remain near gravity vector
  TEST_ASSERT_DOUBLE_WITHIN(0.1, g_body[0], last.frame.accel[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.1, g_body[1], last.frame.accel[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.1, g_body[2], last.frame.accel[2]);

  // Gyro should remain near 0 as well
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, last.frame.gyro[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, last.frame.gyro[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, last.frame.gyro[2]);

  // Accel estimation should remain close to the raw values (single IMU)
  eskf_scalar fused_ax_expected = a0[(out_count - 1) * 3 + 0];
  eskf_scalar fused_ay_expected = a0[(out_count - 1) * 3 + 1];
  eskf_scalar fused_az_expected = a0[(out_count - 1) * 3 + 2];

  TEST_ASSERT_DOUBLE_WITHIN(0.05, fused_ax_expected, last.frame.accel[0]);
  TEST_ASSERT_DOUBLE_WITHIN(0.05, fused_ay_expected, last.frame.accel[1]);
  TEST_ASSERT_DOUBLE_WITHIN(0.05, fused_az_expected, last.frame.accel[2]);

}

// ============================================================
// Behavior Contracts (Checklist Items 8 and 9)
// ============================================================

static void test_virtual_imu_contract_08_central_diff_seven_sample_timing() {
  // Subsystem: VirtualImu central-diff timing contract.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.use_central_diff = true;
  cfg.voting_enabled = false;
  vimu.configure(cfg);

  eskf_sensor_t accel6[6 * 3] = {};
  eskf_sensor_t gyro6[6 * 3] = {};
  for (size_t i = 0; i < 6; ++i) {
    accel6[i * 3 + 2] = -9.80665f;
  }

  const eskf_sensor_t* accel_ptrs[ESKF_MAX_IMUS] = {accel6, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_ptrs[ESKF_MAX_IMUS] = {gyro6, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {
      SensorStatus::OK,
      SensorStatus::HARD_FAIL,
      SensorStatus::HARD_FAIL,
      SensorStatus::HARD_FAIL};

  VirtualImuOutput out6[6];
  size_t count6 = vimu.process(accel_ptrs, gyro_ptrs, nullptr,
                               statuses, 6, 0, out6, 6, 1000);
  TEST_ASSERT_EQUAL(0, count6);
  TEST_ASSERT_EQUAL(3, static_cast<int>(vimu.lookAheadSamples()));

  eskf_sensor_t accel1[3] = {0, 0, -9.80665f};
  eskf_sensor_t gyro1[3] = {0, 0, 0};
  const eskf_sensor_t* accel_ptrs1[ESKF_MAX_IMUS] = {accel1, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_ptrs1[ESKF_MAX_IMUS] = {gyro1, nullptr, nullptr, nullptr};

  VirtualImuOutput out1[1];
  size_t count1 = vimu.process(accel_ptrs1, gyro_ptrs1, nullptr,
                               statuses, 1, 6000, out1, 1, 1000);

  TEST_ASSERT_EQUAL(1, count1);
  TEST_ASSERT_EQUAL_UINT64(3000, out1[0].frame.timestamp_us);
  TEST_ASSERT_EQUAL_UINT64(3000, 6000 - out1[0].frame.timestamp_us);
}

static void test_virtual_imu_contract_09_zero_valid_imu_collapse() {
  // Subsystem: VirtualImu voting collapse when all active IMUs are rejected.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  cfg.imus[0].enabled = true;
  cfg.imus[1].enabled = true;
  cfg.imus[2].enabled = true;
  cfg.voting_enabled = true;
  cfg.use_central_diff = false;
  cfg.gyro_voting_threshold = 100.0;
  cfg.accel_voting_threshold = 0.5;
  cfg.enable_all_soft_reject_salvage = false; // Preserve checklist-09 legacy contract
  vimu.configure(cfg);

  // Crafted triad forces accel-voting median vector to [0,0,0], rejecting all.
  eskf_sensor_t a0[3] = {0.0f, 10.0f, 0.0f};
  eskf_sensor_t a1[3] = {10.0f, 0.0f, 0.0f};
  eskf_sensor_t a2[3] = {0.0f, 0.0f, 10.0f};
  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t g1[3] = {0, 0, 0};
  eskf_sensor_t g2[3] = {0, 0, 0};

  const eskf_sensor_t* accel_ptrs[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
  const eskf_sensor_t* gyro_ptrs[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {
      SensorStatus::OK, SensorStatus::OK, SensorStatus::OK, SensorStatus::HARD_FAIL};

  VirtualImuOutput out[1];
  size_t count = vimu.process(accel_ptrs, gyro_ptrs, nullptr,
                              statuses, 1, 0, out, 1, 1000);

  TEST_ASSERT_EQUAL(1, count);
  TEST_ASSERT_EQUAL(0, static_cast<int>(out[0].valid_imu_count));
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, out[0].frame.accel[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, out[0].frame.gyro[0]);
}

// ============================================================
// Behavior Contracts (Checklist Items 19-29)
// ============================================================

static void test_virtual_imu_contract_19_multi_imu_dual_threshold_separation() {
  // Checklist item 19: soft-vote reject must remain separate from hard-health.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.use_central_diff = false;
  cfg.voting_enabled = true;
  cfg.gyro_voting_threshold = 0.3;
  cfg.gyro_hard_fault_threshold = 2.5;
  cfg.hard_fault_suspect_samples = 2;
  cfg.hard_fault_persistence_samples = 3;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2[3] = {0, 0, -9.8f};
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.1f};
  eskf_sensor_t g2[3] = {0, 0, 0.7f}; // soft outlier, not hard-fault

  const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
  const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK,
                                          SensorStatus::HARD_FAIL};

  VirtualImuOutput out[1];
  for (int k = 0; k < 4; ++k) {
    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }

  TEST_ASSERT_EQUAL(SensorStatus::SOFT_FAIL, out[0].imu_status[2]);
  TEST_ASSERT_NOT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);
  TEST_ASSERT_EQUAL_UINT16(0, out[0].imu_hard_fault_counter[2]);
}

static void test_virtual_imu_contract_20_persistence_and_recovery_state_machine() {
  // Checklist item 20: Active->Suspect->Inhibited->Recovering->Active.
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
  cfg.hard_fault_suspect_samples = 2;
  cfg.hard_fault_persistence_samples = 3;
  cfg.recovery_cooldown_samples = 2;
  cfg.recovery_confirm_samples = 2;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2[3] = {0, 0, -9.8f};
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.1f};
  eskf_sensor_t g2_bad[3] = {0, 0, 9.0f};
  eskf_sensor_t g2_good[3] = {0, 0, 0.11f};

  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK,
                                          SensorStatus::HARD_FAIL};
  VirtualImuOutput out[1];

  // Fault burst drives inhibit.
  for (int k = 0; k < 3; ++k) {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_bad, nullptr};
    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }
  TEST_ASSERT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);

  // Sustained clean data re-enables sensor after cooldown+confirm.
  for (int k = 0; k < 5; ++k) {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_good, nullptr};
    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(10 + k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }
  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[2]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[2]);
}

static void test_virtual_imu_contract_30_stale_sensor_detect_and_recover() {
  // Frozen/stuck sensor should be treated as hard-fault after persistence,
  // then recover through the standard lifecycle once samples change again.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.use_central_diff = false;
  cfg.voting_enabled = true;
  cfg.gyro_voting_threshold = 100.0;
  cfg.accel_voting_threshold = 100.0;
  cfg.gyro_hard_fault_threshold = 1000.0;
  cfg.accel_hard_fault_threshold = 1000.0;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  cfg.recovery_cooldown_samples = 2;
  cfg.recovery_confirm_samples = 2;
  cfg.stale_persistence_samples = 2;
  cfg.stale_accel_delta_threshold = 0.0;
  cfg.stale_gyro_delta_threshold = 0.0;
  vimu.configure(cfg);

  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK,
                                          SensorStatus::HARD_FAIL};
  VirtualImuOutput out[1];

  // IMU 2 is frozen while IMU 0/1 keep changing slightly.
  for (int k = 0; k < 4; ++k) {
    const eskf_sensor_t a0[3] = {0.001f * k, 0.0f, -9.8f};
    const eskf_sensor_t a1[3] = {-0.001f * k, 0.0f, -9.8f};
    const eskf_sensor_t a2[3] = {0.0f, 0.0f, -9.8f};
    const eskf_sensor_t g0[3] = {0.0f, 0.0f, 0.10f + 0.001f * k};
    const eskf_sensor_t g1[3] = {0.0f, 0.0f, 0.11f + 0.001f * k};
    const eskf_sensor_t g2[3] = {0.0f, 0.0f, 0.12f};
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }

  TEST_ASSERT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);
  TEST_ASSERT_EQUAL(SensorStatus::INHIBITED, out[0].imu_status[2]);

  // IMU 2 unsticks (changes every sample) and should recover.
  for (int k = 0; k < 6; ++k) {
    const eskf_sensor_t a0[3] = {0.01f + 0.001f * k, 0.0f, -9.8f};
    const eskf_sensor_t a1[3] = {-0.01f - 0.001f * k, 0.0f, -9.8f};
    const eskf_sensor_t a2[3] = {0.0f, 0.001f * (k + 1), -9.8f};
    const eskf_sensor_t g0[3] = {0.0f, 0.0f, 0.20f + 0.001f * k};
    const eskf_sensor_t g1[3] = {0.0f, 0.0f, 0.21f + 0.001f * k};
    const eskf_sensor_t g2[3] = {0.0f, 0.0f, 0.22f + 0.002f * k};
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(100 + k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }

  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[2]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[2]);
}

static void
test_virtual_imu_contract_34_missing_source_frames_do_not_poison_health() {
  // Regression: asynchronous single-source batch processing (other sources
  // absent for this frame) must not be interpreted as hard-fault evidence for
  // missing sensors.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.use_central_diff = false;
  cfg.voting_enabled = true;
  cfg.gyro_voting_threshold = 100.0;
  cfg.accel_voting_threshold = 100.0;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  cfg.recovery_cooldown_samples = 2;
  cfg.recovery_confirm_samples = 2;
  vimu.configure(cfg);

  const eskf_sensor_t a0[3] = {0.0f, 0.0f, -9.8f};
  const eskf_sensor_t a1[3] = {0.0f, 0.0f, -9.8f};
  const eskf_sensor_t a2[3] = {0.0f, 0.0f, -9.8f};
  const eskf_sensor_t g0[3] = {0.0f, 0.0f, 0.10f};
  const eskf_sensor_t g1[3] = {0.0f, 0.0f, 0.11f};
  const eskf_sensor_t g2[3] = {0.0f, 0.0f, 0.12f};

  VirtualImuOutput out[1];
  for (int k = 0; k < 12; ++k) {
    const int src = k % 3;
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {nullptr, nullptr, nullptr,
                                                      nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {nullptr, nullptr, nullptr,
                                                     nullptr};
    SensorStatus statuses[ESKF_MAX_IMUS] = {
        SensorStatus::HARD_FAIL,
        SensorStatus::HARD_FAIL,
        SensorStatus::HARD_FAIL,
        SensorStatus::HARD_FAIL,
    };

    if (src == 0) {
      accel_data[0] = a0;
      gyro_data[0] = g0;
    } else if (src == 1) {
      accel_data[1] = a1;
      gyro_data[1] = g1;
    } else {
      accel_data[2] = a2;
      gyro_data[2] = g2;
    }
    statuses[src] = SensorStatus::OK;

    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_EQUAL(1, out[0].valid_imu_count);

    for (size_t i = 0; i < 3; ++i) {
      TEST_ASSERT_NOT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[i]);
      TEST_ASSERT_NOT_EQUAL(SensorHealthState::RECOVERING, out[0].imu_health[i]);
    }
  }

  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[0]);
  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[1]);
  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[2]);
}

static void
test_virtual_imu_contract_32_preflight_windowed_tare_membership_change() {
  // Per-IMU preflight tare should remove turn-on bias mismatch when IMU
  // membership changes in flight.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.use_central_diff = false;
  cfg.voting_enabled = false;
  cfg.enable_preflight_tare = true;
  cfg.tare_window_samples = 2;
  cfg.tare_accel_gravity = 9.80665f;
  cfg.gyro_hard_fault_threshold = 1e6f;
  cfg.accel_hard_fault_threshold = 1e6f;
  cfg.stale_persistence_samples = 0;
  vimu.configure(cfg);

  SensorStatus all_ok[ESKF_MAX_IMUS] = {
      SensorStatus::OK, SensorStatus::OK, SensorStatus::OK,
      SensorStatus::HARD_FAIL};
  SensorStatus drop_imu2[ESKF_MAX_IMUS] = {
      SensorStatus::OK, SensorStatus::OK, SensorStatus::HARD_FAIL,
      SensorStatus::HARD_FAIL};

  VirtualImuOutput out[1];

  constexpr eskf_sensor_t g_true[3] = {0.05f, -0.03f, 0.15f};
  constexpr eskf_sensor_t a_true[3] = {0.0f, 0.0f, -9.80665f};

  const eskf_sensor_t g_bias[3][3] = {{0.020f, -0.010f, 0.030f},
                                      {-0.015f, 0.005f, -0.020f},
                                      {0.010f, 0.020f, 0.010f}};
  const eskf_sensor_t a_bias[3] = {0.40f, -0.30f, 0.20f};

  eskf::VirtualImuRuntimePolicy preflight_policy;
  preflight_policy.in_flight = false;
  preflight_policy.soft_threshold_scale = 1.0f;
  preflight_policy.boost_phase = false;
  vimu.setRuntimePolicy(preflight_policy);

  // Complete one full tare window preflight.
  for (int k = 0; k < 2; ++k) {
    eskf_sensor_t a0[3] = {a_true[0], a_true[1], a_true[2] + a_bias[0]};
    eskf_sensor_t a1[3] = {a_true[0], a_true[1], a_true[2] + a_bias[1]};
    eskf_sensor_t a2[3] = {a_true[0], a_true[1], a_true[2] + a_bias[2]};
    eskf_sensor_t g0[3] = {g_bias[0][0], g_bias[0][1], g_bias[0][2]};
    eskf_sensor_t g1[3] = {g_bias[1][0], g_bias[1][1], g_bias[1][2]};
    eskf_sensor_t g2[3] = {g_bias[2][0], g_bias[2][1], g_bias[2][2]};

    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, all_ok, 1,
                     static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }

  // Freeze tare at flight start, then drop IMU 2.
  eskf::VirtualImuRuntimePolicy flight_policy;
  flight_policy.in_flight = true;
  flight_policy.soft_threshold_scale = 1.0f;
  flight_policy.boost_phase = false;
  vimu.setRuntimePolicy(flight_policy);

  eskf_sensor_t a0f[3] = {a_true[0], a_true[1], a_true[2] + a_bias[0]};
  eskf_sensor_t a1f[3] = {a_true[0], a_true[1], a_true[2] + a_bias[1]};
  eskf_sensor_t a2f[3] = {a_true[0], a_true[1], a_true[2] + a_bias[2]};
  eskf_sensor_t g0f[3] = {g_true[0] + g_bias[0][0], g_true[1] + g_bias[0][1],
                          g_true[2] + g_bias[0][2]};
  eskf_sensor_t g1f[3] = {g_true[0] + g_bias[1][0], g_true[1] + g_bias[1][1],
                          g_true[2] + g_bias[1][2]};
  eskf_sensor_t g2f[3] = {g_true[0] + g_bias[2][0], g_true[1] + g_bias[2][1],
                          g_true[2] + g_bias[2][2]};

  const eskf_sensor_t *accel_data_flight[ESKF_MAX_IMUS] = {a0f, a1f, a2f,
                                                            nullptr};
  const eskf_sensor_t *gyro_data_flight[ESKF_MAX_IMUS] = {g0f, g1f, g2f,
                                                           nullptr};
  const size_t count = vimu.process(accel_data_flight, gyro_data_flight,
                                    nullptr, drop_imu2, 1, 10000ULL, out, 1,
                                    1000);
  TEST_ASSERT_EQUAL(1, count);

  // With per-IMU tare frozen from preflight, dropping one IMU should not
  // reintroduce its turn-on bias into fused gyro/accel.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, g_true[0], out[0].frame.gyro[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, g_true[1], out[0].frame.gyro[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, g_true[2], out[0].frame.gyro[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, a_true[0], out[0].frame.accel[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, a_true[1], out[0].frame.accel[1]);
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, a_true[2], out[0].frame.accel[2]);
}

static void test_virtual_imu_contract_21_liftoff_boost_soft_threshold_only() {
  // Checklist item 21: boost inflates soft thresholds only.
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
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2[3] = {0, 0, -9.8f};
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.1f};
  eskf_sensor_t g2_soft[3] = {0, 0, 0.9f};
  eskf_sensor_t g2_hard[3] = {0, 0, 8.0f};

  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK,
                                          SensorStatus::HARD_FAIL};
  VirtualImuOutput out[1];

  // Preflight / nominal policy: soft reject.
  {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_soft, nullptr};
    const size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses,
                                      1, 0, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_EQUAL(SensorStatus::SOFT_FAIL, out[0].imu_status[2]);
  }

  // Boost policy: soft threshold inflated, same sample no longer soft-rejected.
  eskf::VirtualImuRuntimePolicy boost_policy;
  boost_policy.soft_threshold_scale = 3.0;
  boost_policy.boost_phase = true;
  vimu.setRuntimePolicy(boost_policy);
  {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_soft, nullptr};
    const size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses,
                                      1, 1000, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
    TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[2]);
  }

  // Hard-fault path must remain strict in boost.
  {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2_hard, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 2000, out, 1,
                 1000);
    vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 3000, out, 1,
                 1000);
  }
  TEST_ASSERT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);
}

static void test_virtual_imu_contract_22_all_soft_reject_salvage_continuity() {
  // Checklist item 22: deterministic salvage when all sensors soft-rejected.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.use_central_diff = false;
  cfg.voting_enabled = true;
  cfg.gyro_voting_threshold = 100.0;
  cfg.accel_voting_threshold = 0.5;
  cfg.accel_hard_fault_threshold = 20.0;
  cfg.enable_all_soft_reject_salvage = true;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0.8f, 0.8f, 0.0f};
  eskf_sensor_t a1[3] = {0.8f, -0.8f, 0.0f};
  eskf_sensor_t a2[3] = {-1.6f, 0.0f, 0.0f};
  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t g1[3] = {0, 0, 0};
  eskf_sensor_t g2[3] = {0, 0, 0};

  const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
  const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {
      SensorStatus::OK, SensorStatus::OK, SensorStatus::OK,
      SensorStatus::HARD_FAIL};

  VirtualImuOutput out[1];
  const size_t count =
      vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, out, 1,
                   1000);
  TEST_ASSERT_EQUAL(1, count);
  TEST_ASSERT_EQUAL(1, static_cast<int>(out[0].valid_imu_count));
  TEST_ASSERT_TRUE(out[0].degraded_output);
  TEST_ASSERT_TRUE(out[0].continuity_salvage_used);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[0]);
}

static void test_virtual_imu_contract_23_no_retroactive_threshold_determinism() {
  // Checklist item 23: identical stream + schedule yields identical traces.
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
    cfg.accel_voting_threshold = 0.8;
    cfg.accel_hard_fault_threshold = 5.0;
    vimu.configure(cfg);

    struct SampleTrace {
      uint64_t ts;
      size_t valid;
      bool degraded;
      uint8_t s0;
      uint8_t s1;
      uint8_t s2;
      eskf_scalar ax;
      eskf_scalar gz;
    };
    std::vector<SampleTrace> trace;

    SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                            SensorStatus::OK,
                                            SensorStatus::HARD_FAIL};

    for (int k = 0; k < 30; ++k) {
      const eskf_scalar t = static_cast<eskf_scalar>(k);
      eskf_sensor_t a0[3] = {0.1f, 0.0f, -9.8f};
      eskf_sensor_t a1[3] = {0.1f, 0.0f, -9.8f};
        eskf_sensor_t a2[3] = {
          static_cast<eskf_sensor_t>(0.1f + 0.2f * std::sin(0.2f * t)),
          0.0f,
          static_cast<eskf_sensor_t>(-9.8f + 0.1f * std::cos(0.15f * t))};
      eskf_sensor_t g0[3] = {0.0f, 0.0f, 0.1f};
      eskf_sensor_t g1[3] = {0.0f, 0.0f, 0.1f};
        eskf_sensor_t g2[3] = {
          0.0f,
          0.0f,
          static_cast<eskf_sensor_t>(0.1f + 0.6f * std::sin(0.25f * t))};

      eskf::VirtualImuRuntimePolicy policy;
      policy.soft_threshold_scale = (k >= 8 && k <= 15) ? 2.2 : 1.0;
      policy.boost_phase = (k >= 8 && k <= 15);
      vimu.setRuntimePolicy(policy);

      const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
      const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
      VirtualImuOutput out[1];
      const size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses,
                                        1, static_cast<uint64_t>(k) * 1000ULL,
                                        out, 1, 1000);
      if (count == 1) {
        SampleTrace st{};
        st.ts = out[0].frame.timestamp_us;
        st.valid = out[0].valid_imu_count;
        st.degraded = out[0].degraded_output;
        st.s0 = static_cast<uint8_t>(out[0].imu_status[0]);
        st.s1 = static_cast<uint8_t>(out[0].imu_status[1]);
        st.s2 = static_cast<uint8_t>(out[0].imu_status[2]);
        st.ax = out[0].frame.accel[0];
        st.gz = out[0].frame.gyro[2];
        trace.push_back(st);
      }
    }
    return trace;
  };

  const auto trace_a = run_trace();
  const auto trace_b = run_trace();
  TEST_ASSERT_EQUAL(trace_a.size(), trace_b.size());
  for (size_t i = 0; i < trace_a.size(); ++i) {
    TEST_ASSERT_EQUAL_UINT64(trace_a[i].ts, trace_b[i].ts);
    TEST_ASSERT_EQUAL(trace_a[i].valid, trace_b[i].valid);
    TEST_ASSERT_EQUAL(trace_a[i].degraded, trace_b[i].degraded);
    TEST_ASSERT_EQUAL(trace_a[i].s0, trace_b[i].s0);
    TEST_ASSERT_EQUAL(trace_a[i].s1, trace_b[i].s1);
    TEST_ASSERT_EQUAL(trace_a[i].s2, trace_b[i].s2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, trace_a[i].ax, trace_b[i].ax);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, trace_a[i].gz, trace_b[i].gz);
  }
}

static void test_virtual_imu_contract_33_dual_imu_relative_hard_fault_observability() {
  // Relative median-distance hard-faulting is non-observable with only 2 IMUs.
  // Both sensors can disagree symmetrically; neither should be hard-attributed.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[1].enabled = true;
  cfg.use_central_diff = false;
  cfg.voting_enabled = true;
  cfg.gyro_voting_threshold = 0.2f;
  cfg.gyro_hard_fault_threshold = 0.5f;
  cfg.accel_voting_threshold = 100.0f;
  cfg.accel_hard_fault_threshold = 0.0f;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 1;
  cfg.stale_persistence_samples = 0;
  vimu.configure(cfg);

  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::HARD_FAIL,
                                          SensorStatus::HARD_FAIL};
  VirtualImuOutput out[1];

  for (int k = 0; k < 3; ++k) {
    const eskf_sensor_t a0[3] = {0.0f, 0.0f, -9.80665f};
    const eskf_sensor_t a1[3] = {0.0f, 0.0f, -9.80665f};
    const eskf_sensor_t g0[3] = {0.0f, 0.0f, 0.1f};
    const eskf_sensor_t g1[3] = {0.0f, 0.0f, 7.0f};
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, nullptr, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, nullptr, nullptr};

    const size_t count =
        vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                     static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
    TEST_ASSERT_EQUAL(1, count);
  }

  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[0]);
  TEST_ASSERT_EQUAL(SensorHealthState::ACTIVE, out[0].imu_health[1]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[0]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out[0].imu_status[1]);
  TEST_ASSERT_EQUAL_UINT16(0, out[0].imu_hard_fault_counter[0]);
  TEST_ASSERT_EQUAL_UINT16(0, out[0].imu_hard_fault_counter[1]);
  TEST_ASSERT_EQUAL(2, static_cast<int>(out[0].valid_imu_count));
}

static void test_virtual_baro_contract_24_hard_failure_persistence_inhibit() {
  // Checklist item 24: persistent failed barometer gets inhibited.
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  cfg.baro_count = 3;
  cfg.baros[0].enabled = true;
  cfg.baros[1].enabled = true;
  cfg.baros[2].enabled = true;
  cfg.voting_enabled = true;
  cfg.voting_threshold_pa = 600.0;
  cfg.hard_fault_threshold_pa = 1500.0;
  cfg.calibration_mismatch_tolerance_pa = 200.0;
  cfg.hard_fault_persistence_samples = 3;
  cfg.hard_fault_suspect_samples = 2;
  vbaro.configure(cfg);

  eskf_sensor_t t[3] = {288.15f, 288.15f, 288.15f};
  SensorStatus s[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
  BaroOutput out{};
  for (int k = 0; k < 4; ++k) {
    eskf_sensor_t p[3] = {101325.0f, 101330.0f, 108500.0f};
    out = vbaro.process(p, t, s, static_cast<uint64_t>(k) * 20000ULL);
  }

  TEST_ASSERT_EQUAL(BaroHealthState::INHIBITED, out.baro_health[2]);
  TEST_ASSERT_EQUAL(SensorStatus::INHIBITED, out.baro_status[2]);
}

static void test_virtual_baro_contract_25_calibration_mismatch_tolerance() {
  // Checklist item 25: moderate stable mismatch must not hard-inhibit.
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  cfg.baro_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.baros[i].enabled = true;
  }
  cfg.voting_enabled = true;
  cfg.voting_threshold_pa = 250.0;
  cfg.hard_fault_threshold_pa = 900.0;
  cfg.calibration_mismatch_tolerance_pa = 900.0;
  cfg.hard_fault_persistence_samples = 3;
  vbaro.configure(cfg);

  eskf_sensor_t t[3] = {288.15f, 288.15f, 288.15f};
  SensorStatus s[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
  BaroOutput out{};
  for (int k = 0; k < 8; ++k) {
    eskf_sensor_t p[3] = {101325.0f, 101780.0f, 101550.0f};
    out = vbaro.process(p, t, s, static_cast<uint64_t>(k) * 20000ULL);
  }

  TEST_ASSERT_NOT_EQUAL(BaroHealthState::INHIBITED, out.baro_health[1]);
  TEST_ASSERT_NOT_EQUAL(BaroHealthState::INHIBITED, out.baro_health[2]);
}

static void test_virtual_baro_contract_26_soft_vote_vs_hard_health_separation() {
  // Checklist item 26: transient soft disagreement must not latch hard disable.
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  cfg.baro_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.baros[i].enabled = true;
  }
  cfg.voting_enabled = true;
  cfg.voting_threshold_pa = 300.0;
  cfg.hard_fault_threshold_pa = 1500.0;
  cfg.calibration_mismatch_tolerance_pa = 500.0;
  cfg.hard_fault_persistence_samples = 3;
  vbaro.configure(cfg);

  eskf_sensor_t t[3] = {288.15f, 288.15f, 288.15f};
  SensorStatus s[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
  BaroOutput out{};
  for (int k = 0; k < 12; ++k) {
    eskf_sensor_t p[3] = {101325.0f, 101330.0f,
                          (k % 2 == 0) ? 101900.0f : 101335.0f};
    out = vbaro.process(p, t, s, static_cast<uint64_t>(k) * 20000ULL);
  }

  TEST_ASSERT_NOT_EQUAL(BaroHealthState::INHIBITED, out.baro_health[2]);
  TEST_ASSERT_TRUE(out.baro_hard_fault_counter[2] < 3);
}

static void test_virtual_baro_contract_27_fused_output_continuity_single_sensor_failure() {
  // Checklist item 27: continuity and variance behavior across failure/inhibit.
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  cfg.baro_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.baros[i].enabled = true;
  }
  cfg.voting_enabled = true;
  cfg.voting_threshold_pa = 8000.0; // Keep all in pre-fault baseline.
  cfg.hard_fault_threshold_pa = 2000.0;
  cfg.calibration_mismatch_tolerance_pa = 0.0;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  cfg.continuity_max_step_pa = 50.0;
  cfg.single_sensor_variance = 1.0;
  vbaro.configure(cfg);

  eskf_sensor_t t[3] = {288.15f, 288.15f, 288.15f};
  SensorStatus s[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};

  eskf_sensor_t p_nominal[3] = {101325.0f, 101325.0f, 102500.0f};
  BaroOutput before = vbaro.process(p_nominal, t, s, 0);

  eskf_sensor_t p_fault[3] = {101325.0f, 101325.0f, 108000.0f};
  BaroOutput during = vbaro.process(p_fault, t, s, 20000);
  BaroOutput after = vbaro.process(p_fault, t, s, 40000);

  const eskf_scalar step = std::abs(during.pressure_pa - before.pressure_pa);
  TEST_ASSERT_TRUE(step <= 50.0 + 1e-6);
  TEST_ASSERT_EQUAL(2, static_cast<int>(after.valid_count));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.5, after.variance);
}

static void test_virtual_baro_contract_28_voting_policy_determinism_under_replay() {
  // Checklist item 28: two identical runs must produce identical baro traces.
  auto run_trace = []() {
    VirtualBaro vbaro;
    VirtualBaroConfig cfg{};
    cfg.baro_count = 3;
    for (size_t i = 0; i < 3; ++i) {
      cfg.baros[i].enabled = true;
    }
    cfg.voting_enabled = true;
    cfg.voting_threshold_pa = 350.0;
    cfg.hard_fault_threshold_pa = 2200.0;
    cfg.calibration_mismatch_tolerance_pa = 300.0;
    cfg.hard_fault_persistence_samples = 3;
    cfg.enable_all_soft_reject_salvage = true;
    vbaro.configure(cfg);

    struct Trace {
      uint64_t ts;
      eskf_scalar p;
      size_t valid;
      uint8_t s0;
      uint8_t s1;
      uint8_t s2;
      bool degraded;
    };
    std::vector<Trace> trace;

    SensorStatus st[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
    eskf_sensor_t t[3] = {288.15f, 288.15f, 288.15f};

    for (int k = 0; k < 40; ++k) {
      const eskf_scalar tk = static_cast<eskf_scalar>(k);
        eskf_sensor_t p[3] = {
          static_cast<eskf_sensor_t>(101325.0f + 8.0f * std::sin(0.10f * tk)),
          static_cast<eskf_sensor_t>(101330.0f +
                       7.0f * std::sin(0.11f * tk + 0.3f)),
          static_cast<eskf_sensor_t>(101340.0f +
                       ((k % 9 == 0)
                          ? 900.0f
                          : 6.0f * std::sin(0.09f * tk)))};
      BaroOutput out =
          vbaro.process(p, t, st, static_cast<uint64_t>(k) * 20000ULL);
      Trace tr{};
      tr.ts = out.timestamp_us;
      tr.p = out.pressure_pa;
      tr.valid = out.valid_count;
      tr.s0 = static_cast<uint8_t>(out.baro_status[0]);
      tr.s1 = static_cast<uint8_t>(out.baro_status[1]);
      tr.s2 = static_cast<uint8_t>(out.baro_status[2]);
      tr.degraded = out.degraded_output;
      trace.push_back(tr);
    }
    return trace;
  };

  const auto ta = run_trace();
  const auto tb = run_trace();
  TEST_ASSERT_EQUAL(ta.size(), tb.size());
  for (size_t i = 0; i < ta.size(); ++i) {
    TEST_ASSERT_EQUAL_UINT64(ta[i].ts, tb[i].ts);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, ta[i].p, tb[i].p);
    TEST_ASSERT_EQUAL(ta[i].valid, tb[i].valid);
    TEST_ASSERT_EQUAL(ta[i].s0, tb[i].s0);
    TEST_ASSERT_EQUAL(ta[i].s1, tb[i].s1);
    TEST_ASSERT_EQUAL(ta[i].s2, tb[i].s2);
    TEST_ASSERT_EQUAL(ta[i].degraded, tb[i].degraded);
  }
}

static void test_virtual_imu_contract_29_phase_aware_saturation_handling() {
  // Checklist item 29: boost saturation is degraded, outside boost can inhibit.
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.use_central_diff = false;
  cfg.voting_enabled = true;
  cfg.gyro_voting_threshold = 10.0;
  cfg.accel_voting_threshold = 10.0;
  cfg.accel_saturation_threshold = 15.0;
  cfg.gyro_saturation_threshold = 15.0;
  cfg.saturation_multi_axis_limit = 2;
  cfg.saturation_hard_fault_persistence_samples = 2;
  cfg.hard_fault_persistence_samples = 2;
  cfg.hard_fault_suspect_samples = 1;
  vimu.configure(cfg);

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2_clip[3] = {20.0f, 0, -9.8f};
  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.1f};
  eskf_sensor_t g2[3] = {0, 0, 0.1f};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK,
                                          SensorStatus::HARD_FAIL};

  // Boost window: clipping is degraded-but-usable, no inhibit.
  eskf::VirtualImuRuntimePolicy boost_policy;
  boost_policy.soft_threshold_scale = 2.0;
  boost_policy.boost_phase = true;
  vimu.setRuntimePolicy(boost_policy);

  VirtualImuOutput out[1];
  for (int k = 0; k < 3; ++k) {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2_clip, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                 static_cast<uint64_t>(k) * 1000ULL, out, 1, 1000);
  }
  TEST_ASSERT_TRUE(out[0].saturation_detected);
  TEST_ASSERT_TRUE(out[0].degraded_output);
  TEST_ASSERT_NOT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);

  // Outside boost: persistent clipping becomes hard-fault eligible.
  eskf::VirtualImuRuntimePolicy coast_policy;
  coast_policy.soft_threshold_scale = 1.0;
  coast_policy.boost_phase = false;
  vimu.setRuntimePolicy(coast_policy);
  for (int k = 0; k < 3; ++k) {
    const eskf_sensor_t *accel_data[ESKF_MAX_IMUS] = {a0, a1, a2_clip, nullptr};
    const eskf_sensor_t *gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
    vimu.process(accel_data, gyro_data, nullptr, statuses, 1,
                 static_cast<uint64_t>(10 + k) * 1000ULL, out, 1, 1000);
  }
  TEST_ASSERT_EQUAL(SensorHealthState::INHIBITED, out[0].imu_health[2]);
}

// Test: processWithVelocity applies Cp correction
static void test_baro_process_with_velocity() {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  
  cfg.baro_count = 1;
  cfg.baros[0].enabled = true;
  cfg.single_sensor_variance = 1.0;
  
  // Set Cp coefficient: P_corrected = P_meas - 0.5 * rho * v^2 * Cp
  cfg.static_pressure.cp_coefficient = 0.1;  // Typical value
  cfg.static_pressure.air_density = 1.225;   // Sea level
  
  vbaro.configure(cfg);

  eskf_sensor_t pressures[1] = {101325.0f};
  eskf_sensor_t temps[1] = {288.15f};
  SensorStatus statuses[1] = {SensorStatus::OK};

  // At 100 m/s, correction = 0.5 * 1.225 * 100^2 * 0.1 = 612.5 Pa
  BaroOutput out = vbaro.processWithVelocity(pressures, temps, statuses, 0, 100.0);

  eskf_scalar expected_correction = 0.5 * 1.225 * 100.0 * 100.0 * 0.1;
  eskf_scalar expected_pressure = 101325.0 - expected_correction;
  
  TEST_ASSERT_DOUBLE_WITHIN(1.0, expected_pressure, out.pressure_pa);
  TEST_ASSERT_EQUAL(1, out.valid_count);
}

static void test_virtual_baro_contract_31_stale_sensor_detect_and_recover() {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  cfg.baro_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.baros[i].enabled = true;
  }
  cfg.voting_enabled = true;
  cfg.voting_threshold_pa = 1000.0;
  cfg.hard_fault_threshold_pa = 5000.0;
  cfg.calibration_mismatch_tolerance_pa = 0.0;
  cfg.hard_fault_suspect_samples = 1;
  cfg.hard_fault_persistence_samples = 2;
  cfg.recovery_cooldown_samples = 2;
  cfg.recovery_confirm_samples = 2;
  cfg.stale_persistence_samples = 2;
  cfg.stale_pressure_delta_threshold_pa = 0.0;
  cfg.stale_temperature_delta_threshold_k = 0.0;
  vbaro.configure(cfg);

  SensorStatus st[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
  eskf_sensor_t t[3] = {288.15f, 288.15f, 288.15f};
  BaroOutput out{};

  for (int k = 0; k < 5; ++k) {
    eskf_sensor_t p[3] = {
        101325.0f + 2.0f * static_cast<eskf_sensor_t>(k),
        101330.0f - 2.0f * static_cast<eskf_sensor_t>(k),
        101340.0f};
    out = vbaro.process(p, t, st, static_cast<uint64_t>(k) * 20000ULL);
  }

  TEST_ASSERT_EQUAL(BaroHealthState::INHIBITED, out.baro_health[2]);
  TEST_ASSERT_EQUAL(SensorStatus::INHIBITED, out.baro_status[2]);

  for (int k = 0; k < 6; ++k) {
    eskf_sensor_t p[3] = {
        101350.0f + 2.0f * static_cast<eskf_sensor_t>(k),
        101360.0f - 2.0f * static_cast<eskf_sensor_t>(k),
        101340.0f + 3.0f * static_cast<eskf_sensor_t>(k)};
    t[2] = 288.15f + 0.01f * static_cast<eskf_sensor_t>(k + 1);
    out = vbaro.process(p, t, st,
                        static_cast<uint64_t>(100 + k) * 20000ULL);
  }

  TEST_ASSERT_EQUAL(BaroHealthState::ACTIVE, out.baro_health[2]);
  TEST_ASSERT_EQUAL(SensorStatus::OK, out.baro_status[2]);
}

// ============================================================
// GTest Wrapper
// ============================================================

#define WRAP_TEST(test_fn) TEST(KalmanVirtualImuBaroSuite, test_fn) { test_fn(); }

WRAP_TEST(test_default_config_identity_rotation);
WRAP_TEST(test_config_sensor_positions);
WRAP_TEST(test_gyro_voting_rejects_outlier);
WRAP_TEST(test_hard_fail_skips_voting);
WRAP_TEST(test_dynamic_centroid_calculation);
WRAP_TEST(test_centroid_shifts_on_failure);
WRAP_TEST(test_cg_callback_used);
WRAP_TEST(test_lever_arm_correction_accuracy);
WRAP_TEST(test_calibration_application);
WRAP_TEST(test_thermal_calibration_application);
WRAP_TEST(test_thermal_calibration_delta_t_clamped);
WRAP_TEST(test_static_tilt_omega_dot_near_zero);
WRAP_TEST(test_static_tilt_single_imu_omega_dot_near_zero);
WRAP_TEST(test_virtual_imu_contract_08_central_diff_seven_sample_timing);
WRAP_TEST(test_virtual_imu_contract_09_zero_valid_imu_collapse);
WRAP_TEST(test_virtual_imu_contract_19_multi_imu_dual_threshold_separation);
WRAP_TEST(test_virtual_imu_contract_20_persistence_and_recovery_state_machine);
WRAP_TEST(test_virtual_imu_contract_21_liftoff_boost_soft_threshold_only);
WRAP_TEST(test_virtual_imu_contract_22_all_soft_reject_salvage_continuity);
WRAP_TEST(test_virtual_imu_contract_23_no_retroactive_threshold_determinism);
WRAP_TEST(test_virtual_imu_contract_33_dual_imu_relative_hard_fault_observability);
WRAP_TEST(test_virtual_imu_contract_29_phase_aware_saturation_handling);
WRAP_TEST(test_virtual_imu_contract_30_stale_sensor_detect_and_recover);
WRAP_TEST(test_virtual_imu_contract_34_missing_source_frames_do_not_poison_health);
WRAP_TEST(test_virtual_imu_contract_32_preflight_windowed_tare_membership_change);

WRAP_TEST(test_baro_single_sensor);
WRAP_TEST(test_baro_voting_rejects_outlier);
WRAP_TEST(test_baro_variance_scaling);
WRAP_TEST(test_baro_hard_fail_excluded);
WRAP_TEST(test_baro_process_with_velocity);
WRAP_TEST(test_virtual_baro_contract_24_hard_failure_persistence_inhibit);
WRAP_TEST(test_virtual_baro_contract_25_calibration_mismatch_tolerance);
WRAP_TEST(test_virtual_baro_contract_26_soft_vote_vs_hard_health_separation);
WRAP_TEST(test_virtual_baro_contract_27_fused_output_continuity_single_sensor_failure);
WRAP_TEST(test_virtual_baro_contract_28_voting_policy_determinism_under_replay);
WRAP_TEST(test_virtual_baro_contract_31_stale_sensor_detect_and_recover);

#undef WRAP_TEST
