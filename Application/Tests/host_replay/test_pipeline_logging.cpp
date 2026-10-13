#include <cassert>
#include <cstdio>
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Kalman/kalman/eskf_logger.hpp"

extern uint64_t g_sim_us;

struct CountingLogger : eskf::NullEskfLogger {
  unsigned count = 0;
  uint64_t last = 0;
  bool enabled = true;
  bool imuPipelineEnabled() const override { return enabled; }
  void logImuPipeline(const eskf::ImuPipelineSnapshot& snapshot) override {
    if (last) assert(snapshot.timestamp_us - last >= 10000);
    last = snapshot.timestamp_us;
    ++count;
  }
};

int main() {
  app::EskfEstimator logged, reference;
  logged.configureReplaySensorCounts(4, 0);
  reference.configureReplaySensorCounts(4, 0);
  logged.reset();
  reference.reset();
  CountingLogger logger;
  app::ImuSample sample{};
  sample.ax = 9.80665f;
  g_sim_us = 1000000;
  auto feed = [&](unsigned n) {
    for (unsigned j = 0; j < n; ++j, g_sim_us += 156) {
      for (unsigned source = 0; source < 4; ++source) {
        app::ImuBatch batch{&sample, 1, g_sim_us, 156, (uint8_t)source};
        eskf::setEskfLogger(&logger);
        logged.processImuBatch(batch);
        eskf::setEskfLogger(nullptr);
        reference.processImuBatch(batch);
      }
      eskf::setEskfLogger(&logger);
      logged.onTick(g_sim_us);
      eskf::setEskfLogger(nullptr);
      reference.onTick(g_sim_us);
      const auto a = logged.output(), b = reference.output();
      assert(a.altitude_m == b.altitude_m);
      assert(a.vertical_velocity_mps == b.vertical_velocity_mps);
      for (unsigned i = 0; i < 4; ++i) assert(a.quaternion[i] == b.quaternion[i]);
    }
  };
  feed(6400); // approximately one second, regardless of burst/poll timing
  assert(logger.count >= 95 && logger.count <= 101);
  logger.enabled = false;
  const unsigned disabled_count = logger.count;
  feed(640);
  assert(logger.count == disabled_count);
  logger.enabled = true;
  logged.onLiftoff(g_sim_us / 1000);
  reference.onLiftoff(g_sim_us / 1000);
  assert(logged.inFlight() && reference.inFlight());
  const unsigned flight_start_count = logger.count;
  feed(6400);
  assert(logger.count - flight_start_count >= 95);
  assert(logger.count - flight_start_count <= 101);
  logged.reset();
  reference.reset();
  logger.count = 0;
  logger.last = 0;
  feed(6400);
  assert(logger.count >= 95 && logger.count <= 101);
  eskf::setEskfLogger(nullptr);
  std::puts("PASS: pipeline rate bounded before/after liftoff and reset; outputs unchanged");
}
