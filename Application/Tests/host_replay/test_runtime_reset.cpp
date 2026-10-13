#include <cassert>
#include <cstdio>
#include "Application/Data/data.hpp"
#include "Application/Data/ring_buffer.hpp"
#include "Application/Kalman/kalman_health.hpp"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Kalman/kalman/eskf_yieldable.hpp"
extern "C" {
#include "Application/Kalman/kalman_process.h"
}
#include "Drivers/InvIMU/InvIMU.h"

using Drivers::InvIMU::IMUData;
extern uint64_t g_sim_us;
extern RingBuffer<IMUData, 128u> imuData1, imuData2, imuData3, imuData4;

int main() {
  // Ground history wraps deliberately, but active-flight overflow remains loss.
  eskf::EskfYieldable filter;
  filter.init(eskf::TuningConfig{});
  eskf::State initial{};
  initial.q[0] = 1;
  initial.timestamp_us = 1000000;
  filter.initialize(initial, eskf::InitialCovariance{}, eskf::ProcessNoise{});
  eskf::ImuFrame frame{};
  frame.accel[2] = -9.80665;
  for (unsigned i = 0; i < ESKF_IMU_BUFFER_SIZE * 3; ++i) {
    frame.timestamp_us = initial.timestamp_us + i * 156;
    filter.pushImu(frame, 156e-6);
  }
  assert(filter.stats().imu_drops == 0);
  assert(filter.imuBufferCount() == ESKF_IMU_BUFFER_SIZE);
  eskf::LiftoffInitData snap{};
  snap.q[0] = 1;
  snap.liftoff_us = frame.timestamp_us;
  filter.injectLiftoffSnap(snap, frame.timestamp_us);
  assert(!filter.isHibernating());
  for (unsigned i = 0; i < ESKF_IMU_BUFFER_SIZE + 10; ++i) {
    frame.timestamp_us += 156;
    filter.pushImu(frame, 156e-6);
  }
  assert(filter.stats().imu_drops > 0);

  auto &store = flight_computer::GOATStore::get_instance();
  store.stateStore.set(flight_computer::State::INIT);
  g_sim_us = 1000000;
  kalman_loop();
  RingBuffer<IMUData, 128u> *rings[] = {&imuData1, &imuData2, &imuData3, &imuData4};
  auto feed = [&] {
    for (unsigned j = 0; j < 64; ++j) {
      IMUData s{};
      s.accel_z = -9.80665f;
      s.timestamp_us = g_sim_us - 10000 + j * 156;
      for (auto *ring : rings) ring->append(s);
    }
    kalman_loop();
  };
  feed();
  auto before = KalmanHealthStore::instance().get();
  assert(before.imu_samples_consumed > 0);
  assert(before.kalman_behind_us == 0); // ground history intentionally hibernates
  assert(before.imu_ring_hwm[0] > 0);
  const uint32_t generation = kalman_reset_generation();
  kalman_note_main_loop_iteration_us(12345);
  assert(kalman_request_reset() == 1);
  g_sim_us += 20000;
  kalman_loop(); // no new samples; cached counters must not reappear
  auto reset = KalmanHealthStore::instance().get();
  assert(reset.imu_samples_consumed == 0);
  assert(kalman_reset_generation() == generation + 1);
  assert(reset.max_main_loop_iteration_us == 0);
  for (auto depth : reset.imu_ring_hwm) assert(depth == 0);
  feed();
  assert(KalmanHealthStore::instance().get().imu_samples_consumed > 0);
  store.stateStore.set(flight_computer::State::IGNITION);
  kalman_on_state_change(flight_computer::State::IGNITION);
  assert(kalman_request_reset() == 0);
  assert(kalman_reset_generation() == generation + 1);
  std::puts("PASS: runtime health reset, input resumption and flight-state refusal");
}
