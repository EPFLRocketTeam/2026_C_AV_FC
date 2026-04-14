#include "Application/Kalman/kalman/preprocessor/virtual_imu.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_baro.hpp"
#include "Application/Kalman/kalman/shadow_filter.hpp"

#include <cmath>

#include <gtest/gtest.h>

using namespace eskf;

namespace {
constexpr eskf_scalar kTol = 1e-6;
constexpr eskf_scalar kTolF = 1e-4;

eskf_scalar g_test_cg[3] = {0.15, 0.0, 0.0};

void testCgCallback(void* user_data, uint64_t timestamp_us, eskf_scalar cg_out[3]) {
  (void)user_data;
  (void)timestamp_us;
  cg_out[0] = g_test_cg[0];
  cg_out[1] = g_test_cg[1];
  cg_out[2] = g_test_cg[2];
}

}  // namespace

TEST(KtpVirtualImuParity, DefaultConfigIdentityRotation) {
  ImuSensorConfig cfg{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const eskf_scalar expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(cfg.to_body[i][j], expected, kTol);
    }
  }
}

TEST(KtpVirtualImuParity, GyroVotingRejectsOutlier) {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 3;
  for (size_t i = 0; i < 3; ++i) {
    cfg.imus[i].enabled = true;
  }
  cfg.gyro_voting_threshold = 0.5;
  cfg.voting_enabled = true;
  cfg.use_central_diff = false;
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0.1f};
  eskf_sensor_t g1[3] = {0, 0, 0.11f};
  eskf_sensor_t g2[3] = {0, 0, 5.0f};

  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  eskf_sensor_t a1[3] = {0, 0, -9.8f};
  eskf_sensor_t a2[3] = {0, 0, -9.8f};

  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  const size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0,
                                    output, 1, 1000);

  ASSERT_EQ(count, 1u);
  EXPECT_EQ(output[0].imu_status[0], SensorStatus::OK);
  EXPECT_EQ(output[0].imu_status[1], SensorStatus::OK);
  EXPECT_EQ(output[0].imu_status[2], SensorStatus::SOFT_FAIL);
  EXPECT_EQ(output[0].valid_imu_count, 2u);
  EXPECT_NEAR(output[0].frame.gyro[2], 0.105, 0.05);
}

TEST(KtpVirtualImuParity, HardFailSkipsVoting) {
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
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL,
                                          SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  const size_t count = vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0,
                                    output, 1, 1000);

  ASSERT_EQ(count, 1u);
  EXPECT_EQ(output[0].valid_imu_count, 1u);
  EXPECT_NEAR(output[0].frame.gyro[2], 0.1, kTolF);
}

TEST(KtpVirtualImuParity, DynamicCentroidShiftsOnFailure) {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 2;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.2;
  cfg.imus[1].enabled = true;
  cfg.imus[1].position[0] = 0.0;
  cfg.use_central_diff = false;
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a0, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g0, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::HARD_FAIL, SensorStatus::OK,
                                          SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  (void)vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  EXPECT_NEAR(output[0].effective_centroid[0], 0.0, kTol);
}

TEST(KtpVirtualImuParity, CgCallbackUsed) {
  VirtualImu vimu;
  VirtualImuConfig cfg{};
  cfg.imu_count = 1;
  cfg.imus[0].enabled = true;
  cfg.imus[0].position[0] = 0.2;
  cfg.cg_callback = testCgCallback;
  cfg.use_central_diff = false;
  vimu.configure(cfg);

  eskf_sensor_t g0[3] = {0, 0, 0};
  eskf_sensor_t a0[3] = {0, 0, -9.8f};
  const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, nullptr, nullptr, nullptr};
  const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, nullptr, nullptr, nullptr};
  SensorStatus statuses[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::HARD_FAIL,
                                          SensorStatus::HARD_FAIL, SensorStatus::HARD_FAIL};

  VirtualImuOutput output[1];
  (void)vimu.process(accel_data, gyro_data, nullptr, statuses, 1, 0, output, 1, 1000);

  EXPECT_EQ(output[0].valid_imu_count, 1u);
}

TEST(KtpVirtualBaroParity, VotingRejectsOutlier) {
  VirtualBaro vbaro;
  VirtualBaroConfig cfg{};
  cfg.baro_count = 3;
  cfg.baros[0].enabled = true;
  cfg.baros[1].enabled = true;
  cfg.baros[2].enabled = true;
  cfg.voting_threshold_pa = 500.0;
  cfg.voting_enabled = true;
  cfg.single_sensor_variance = 1.0;
  cfg.hard_fault_threshold_pa = 10000.0;
  cfg.calibration_mismatch_tolerance_pa = 0.0;
  vbaro.configure(cfg);

  eskf_sensor_t pressures[3] = {101325.0f, 101330.0f, 105000.0f};
  eskf_sensor_t temps[3] = {288.15f, 288.15f, 288.15f};
  SensorStatus statuses[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};

  const BaroOutput out = vbaro.process(pressures, temps, statuses, 0);

  EXPECT_EQ(out.baro_status[2], SensorStatus::SOFT_FAIL);
  EXPECT_EQ(out.valid_count, 2u);
  EXPECT_NEAR(out.pressure_pa, 101327.5, 10.0);
}

TEST(KtpShadowParity, RailResetIdentity) {
  RailShadowFilter filter;
  filter.reset();

  const eskf_scalar* q = filter.quaternion();
  EXPECT_NEAR(q[0], 1.0, kTol);
  EXPECT_NEAR(q[1], 0.0, kTol);
  EXPECT_NEAR(q[2], 0.0, kTol);
  EXPECT_NEAR(q[3], 0.0, kTol);
  EXPECT_TRUE(filter.isGateOpen());
}

TEST(KtpShadowParity, RailGateClosesDuringHandling) {
  RailShadowFilter filter;
  filter.reset();

  eskf_scalar accel[3] = {0, 0, -9.80665};
  const eskf_scalar gyro[3] = {0, 0, 0};

  filter.update(accel, gyro, 0.01);
  EXPECT_TRUE(filter.isGateOpen());

  accel[0] = 5.0;
  filter.update(accel, gyro, 0.01);
  EXPECT_FALSE(filter.isGateOpen());

  accel[0] = 0.0;
  filter.update(accel, gyro, 0.01);
  EXPECT_TRUE(filter.isGateOpen());
}

TEST(KtpShadowParity, FlightAeroBlindIgnoresBaro) {
  FlightShadowFilter filter;
  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  const eskf_scalar accel[3] = {0, 0, -9.80665 - 100.0};
  const eskf_scalar gyro[3] = {0, 0, 0};

  for (int i = 0; i < 1500; ++i) {
    filter.predict(accel, gyro, 0.001);
  }
  EXPECT_TRUE(filter.isAeroBlind());

  const eskf_scalar alt_before = filter.altitude();
  filter.correctBaro(0.0, 0.02);
  EXPECT_NEAR(filter.altitude(), alt_before, kTol);
}

TEST(KtpShadowParity, FlightReengagementSnap) {
  FlightShadowFilter filter;
  const eskf_scalar initial_q[4] = {1, 0, 0, 0};
  filter.reset(initial_q);

  eskf_scalar accel[3] = {0, 0, -9.80665 - 100.0};
  const eskf_scalar gyro[3] = {0, 0, 0};

  for (int i = 0; i < 1500; ++i) {
    filter.predict(accel, gyro, 0.001);
  }
  EXPECT_TRUE(filter.isAeroBlind());

  accel[2] = -9.80665 + 50.0;
  for (int i = 0; i < 4000; ++i) {
    filter.predict(accel, gyro, 0.001);
  }

  EXPECT_FALSE(filter.isAeroBlind());

  const eskf_scalar z_baro = 5000.0;
  filter.correctBaro(z_baro, 0.02);
  EXPECT_NEAR(filter.altitude(), -z_baro, 1.0);
}
