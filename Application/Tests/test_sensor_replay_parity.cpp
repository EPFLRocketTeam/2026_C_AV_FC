#include "Application/Kalman/kalman/preprocessor/virtual_baro.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_imu.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

using namespace eskf;

TEST(KalmanSensorReplayParity, Contract23VirtualImuTwoPassTraceIdentical) {
  auto run_trace = []() {
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
    vimu.configure(cfg);

    std::vector<uint32_t> trace;
    SensorStatus status[ESKF_MAX_IMUS] = {SensorStatus::OK, SensorStatus::OK,
                                          SensorStatus::OK,
                                          SensorStatus::HARD_FAIL};

    for (int k = 0; k < 20; ++k) {
      eskf_sensor_t a0[3] = {0, 0, -9.8f};
      eskf_sensor_t a1[3] = {0, 0, -9.8f};
      eskf_sensor_t a2[3] = {0.2f * std::sin(0.2f * k), 0, -9.8f};
      eskf_sensor_t g0[3] = {0, 0, 0.1f};
      eskf_sensor_t g1[3] = {0, 0, 0.1f};
      eskf_sensor_t g2[3] = {0, 0, 0.1f + 0.7f * std::sin(0.3f * k)};

      VirtualImuRuntimePolicy pol;
      pol.soft_threshold_scale = (k >= 5 && k <= 10) ? 2.0 : 1.0;
      pol.boost_phase = (k >= 5 && k <= 10);
      vimu.setRuntimePolicy(pol);

      const eskf_sensor_t* accel_data[ESKF_MAX_IMUS] = {a0, a1, a2, nullptr};
      const eskf_sensor_t* gyro_data[ESKF_MAX_IMUS] = {g0, g1, g2, nullptr};
      VirtualImuOutput out[1];
      const size_t n = vimu.process(accel_data, gyro_data, nullptr, status, 1,
                                    static_cast<uint64_t>(k) * 1000ULL, out,
                                    1, 1000);
      if (n == 1) {
        uint32_t packed = (static_cast<uint32_t>(out[0].valid_imu_count) << 16) |
                          (static_cast<uint32_t>(out[0].degraded_output) << 8) |
                          static_cast<uint32_t>(out[0].imu_status[2]);
        trace.push_back(packed);
      }
    }
    return trace;
  };

  const auto a = run_trace();
  const auto b = run_trace();
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i], b[i]);
  }
}

TEST(KalmanSensorReplayParity, Contract28VirtualBaroTwoPassTraceIdentical) {
  auto run_trace = []() {
    VirtualBaro vbaro;
    VirtualBaroConfig cfg{};
    cfg.baro_count = 3;
    cfg.baros[0].enabled = true;
    cfg.baros[1].enabled = true;
    cfg.baros[2].enabled = true;
    cfg.voting_enabled = true;
    cfg.voting_threshold_pa = 350.0;
    cfg.hard_fault_threshold_pa = 2200.0;
    cfg.calibration_mismatch_tolerance_pa = 300.0;
    vbaro.configure(cfg);

    std::vector<uint32_t> trace;
    SensorStatus status[3] = {SensorStatus::OK, SensorStatus::OK, SensorStatus::OK};
    eskf_sensor_t temps[3] = {288.15f, 288.15f, 288.15f};

    for (int k = 0; k < 30; ++k) {
      eskf_sensor_t p[3] = {
          101325.0f + 12.0f * std::sin(0.09f * k),
          101330.0f + 10.0f * std::sin(0.10f * k + 0.1f),
          101335.0f + ((k % 11 == 0) ? 900.0f : 8.0f * std::sin(0.11f * k))};
      BaroOutput out =
          vbaro.process(p, temps, status, static_cast<uint64_t>(k) * 20000ULL);
      uint32_t packed = (static_cast<uint32_t>(out.valid_count) << 16) |
                        (static_cast<uint32_t>(out.degraded_output) << 8) |
                        static_cast<uint32_t>(out.baro_status[2]);
      trace.push_back(packed);
    }
    return trace;
  };

  const auto a = run_trace();
  const auto b = run_trace();
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i], b[i]);
  }
}
