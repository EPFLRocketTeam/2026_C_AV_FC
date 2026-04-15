#include "Application/Kalman/AppLayer/apogee_factory.hpp"
#include "Application/Kalman/AppLayer/apogee_hub.hpp"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

namespace {

app::sensors::gnss::GnssSample makeValidGnss(uint64_t ts_us,
                                             int32_t lat_deg7,
                                             int32_t lon_deg7,
                                             int32_t alt_msl_mm) {
    app::sensors::gnss::GnssSample s{};
    s.valid = true;
    s.fix_type = 3;
    s.timestamp_us = ts_us;
    s.pps_timestamp_us = ts_us;
    s.lat_deg7 = lat_deg7;
    s.lon_deg7 = lon_deg7;
    s.alt_msl_mm = alt_msl_mm;
    s.h_acc_mm = 2000;
    s.v_acc_mm = 3000;
    s.s_acc_mms = 300;
    return s;
}

class AlwaysDetectAlgo : public app::IApogeeAlgorithm {
 public:
  void reset() override { armed_ = false; }
  void arm(uint32_t liftoff_ms) override {
    (void)liftoff_ms;
    armed_ = true;
  }
  bool armed() const override { return armed_; }
  app::ApogeeVerdict update(uint32_t now_ms,
                            const app::ApogeeInput& input) override {
    (void)input;
    if (!armed_) return {};
    return app::ApogeeVerdict{app::ApogeeVerdict::Result::Detected,
                              now_ms,
                              0.0f,
                              0.0f,
                              "always"};
  }
  const char* name() const override { return "Always"; }

 private:
  bool armed_ = false;
};

class NeverDetectAlgo : public app::IApogeeAlgorithm {
 public:
  void reset() override { armed_ = false; }
  void arm(uint32_t liftoff_ms) override {
    (void)liftoff_ms;
    armed_ = true;
  }
  bool armed() const override { return armed_; }
  app::ApogeeVerdict update(uint32_t now_ms,
                            const app::ApogeeInput& input) override {
    (void)now_ms;
    (void)input;
    return {};
  }
  const char* name() const override { return "Never"; }

 private:
  bool armed_ = false;
};

}  // namespace

TEST(KalmanParityConsensus, BothAgreeDetects) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 1000.0f;
    input.eskf_valid = true;
    input.eskf_diverged = false;
    input.shadow_valid = true;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -5.0f;
    input.time_since_liftoff_ms = 5000;
    input.eskf_velocity_down_mps = 5.0f;
    input.shadow_velocity_down_mps = 5.0f;

    const app::ApogeeVerdict decision = algo->update(5000, input);
    EXPECT_EQ(decision.result, app::ApogeeVerdict::Result::Detected);
    ASSERT_NE(decision.reason, nullptr);
}

TEST(KalmanParityConsensus, EskfEarlyVeto) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 1000.0f;
    input.eskf_valid = true;
    input.eskf_diverged = false;
    input.shadow_valid = true;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -5.0f;
    input.time_since_liftoff_ms = 5000;
    input.eskf_velocity_down_mps = 2.0f;
    input.shadow_velocity_down_mps = -50.0f;

    const app::ApogeeVerdict decision = algo->update(5000, input);
    EXPECT_EQ(decision.result, app::ApogeeVerdict::Result::Vetoed);
}

TEST(KalmanParityConsensus, EskfEarlyModerateAscentVeto) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 1000.0f;
    input.eskf_valid = true;
    input.eskf_diverged = false;
    input.shadow_valid = true;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -5.0f;
    input.time_since_liftoff_ms = 5000;
    input.eskf_velocity_down_mps = 2.0f;
    input.shadow_velocity_down_mps = -5.0f;

    const app::ApogeeVerdict decision = algo->update(5000, input);
    EXPECT_EQ(decision.result, app::ApogeeVerdict::Result::Vetoed);
}

TEST(KalmanParityConsensus, EskfEarlyNearZeroAllows) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 1000.0f;
    input.eskf_valid = true;
    input.eskf_diverged = false;
    input.shadow_valid = true;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -5.0f;
    input.time_since_liftoff_ms = 5000;
    input.eskf_velocity_down_mps = 1.0f;
    input.shadow_velocity_down_mps = -0.5f;

    const app::ApogeeVerdict decision = algo->update(5000, input);
    EXPECT_EQ(decision.result, app::ApogeeVerdict::Result::Detected);
}

TEST(KalmanParityConsensus, ShadowTimerTimeoutOverride) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 1200.0f;
    input.eskf_valid = true;
    input.eskf_diverged = false;
    input.shadow_valid = true;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -4.0f;
    input.time_since_liftoff_ms = 6000;
    input.eskf_velocity_down_mps = -5.0f;
    input.shadow_velocity_down_mps = 3.0f;

    const app::ApogeeVerdict first = algo->update(5000, input);
    EXPECT_EQ(first.result, app::ApogeeVerdict::Result::WaitingTimer);

    const app::ApogeeVerdict waiting = algo->update(6400, input);
    EXPECT_EQ(waiting.result, app::ApogeeVerdict::Result::WaitingTimer);

    const app::ApogeeVerdict timeout = algo->update(6500, input);
    EXPECT_EQ(timeout.result, app::ApogeeVerdict::Result::Detected);
}

TEST(KalmanParityConsensus, ShadowTimerSurvivesTransientClear) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 1100.0f;
    input.eskf_valid = true;
    input.eskf_diverged = false;
    input.shadow_valid = true;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -4.0f;
    input.time_since_liftoff_ms = 6000;

    input.eskf_velocity_down_mps = -5.0f;
    input.shadow_velocity_down_mps = 2.0f;
    const app::ApogeeVerdict start = algo->update(5000, input);
    EXPECT_EQ(start.result, app::ApogeeVerdict::Result::WaitingTimer);

    input.shadow_velocity_down_mps = -0.1f;
    const app::ApogeeVerdict clear = algo->update(6400, input);
    EXPECT_EQ(clear.result, app::ApogeeVerdict::Result::NoDetection);

    input.shadow_velocity_down_mps = 2.0f;
    const app::ApogeeVerdict timeout = algo->update(6501, input);
    EXPECT_EQ(timeout.result, app::ApogeeVerdict::Result::Detected);
}

TEST(KalmanParityConsensus, DivergedShadowFallbackDetects) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 900.0f;
    input.eskf_valid = false;
    input.eskf_diverged = true;
    input.shadow_valid = true;
    input.shadow_velocity_down_mps = 6.0f;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -2.0f;
    input.time_since_liftoff_ms = 5000;

    const app::ApogeeVerdict verdict = algo->update(5000, input);
    EXPECT_EQ(verdict.result, app::ApogeeVerdict::Result::Detected);
}

TEST(KalmanParityConsensus, DivergedSmallShadowDescentStartsTimer) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 905.0f;
    input.eskf_valid = false;
    input.eskf_diverged = true;
    input.shadow_valid = true;
    input.shadow_velocity_down_mps = 0.2f;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -2.0f;
    input.time_since_liftoff_ms = 5000;

    const app::ApogeeVerdict verdict = algo->update(5000, input);
    EXPECT_EQ(verdict.result, app::ApogeeVerdict::Result::WaitingTimer);
}

TEST(KalmanParityConsensus, DivergedNearZeroTimeoutOverride) {
    auto algo = app::createConsensusApogeeDetector();
    algo->reset();
    algo->arm(0);

    app::ApogeeInput input{};
    input.altitude_m = 950.0f;
    input.eskf_valid = false;
    input.eskf_diverged = true;
    input.shadow_valid = true;
    input.shadow_velocity_down_mps = -5.0f;
    input.is_coast_phase = true;
    input.body_accel_x_mps2 = -2.0f;
    input.time_since_liftoff_ms = 5000;

    const app::ApogeeVerdict first = algo->update(5000, input);
    EXPECT_EQ(first.result, app::ApogeeVerdict::Result::WaitingTimer);

    const app::ApogeeVerdict waiting = algo->update(6400, input);
    EXPECT_EQ(waiting.result, app::ApogeeVerdict::Result::WaitingTimer);

    const app::ApogeeVerdict timeout = algo->update(6500, input);
    EXPECT_EQ(timeout.result, app::ApogeeVerdict::Result::Detected);
}

TEST(KalmanParityApogeeHub, PrimaryOnlyTriggers) {
    app::ApogeeHub hub;
    ASSERT_EQ(hub.addAlgorithm(std::make_unique<NeverDetectAlgo>()), 0);
    ASSERT_EQ(hub.addAlgorithm(std::make_unique<AlwaysDetectAlgo>()), 1);
    hub.setPrimary(0);
    hub.arm(0);

    app::ApogeeInput input{};
    input.is_coast_phase = true;
    input.time_since_liftoff_ms = 5000;

    const app::ApogeeDecision decision = hub.update(5000, input);
    EXPECT_FALSE(decision.triggered);
    EXPECT_EQ(decision.result_count, 2u);
}

TEST(KalmanParityEstimator, Contract13FirstFixNoFuseAfterLateLiftoff) {
    app::EskfEstimator est;
    app::EskfEstimator::Config cfg{};
    cfg.gps_delay_us = 0;
    est.configure(cfg);
    est.reset();
    est.onLiftoff(0);

    auto& filter = const_cast<eskf::EskfYieldable&>(est.filter());
    filter.catchUp(0, 100000);
    eskf::State seeded = filter.state();
    seeded.p[0] = 42.0;
    seeded.p[1] = -17.0;
    seeded.p[2] = 8.0;
    filter.core().setState(seeded);

    const auto first_fix = makeValidGnss(2000000, 485000000, 23500000, 120000);
    est.processGpsSample(first_fix);
    est.onTick(2500000);

    const auto& s = est.filter().state();
    EXPECT_NEAR(s.p[0], 42.0, 1e-9);
    EXPECT_NEAR(s.p[1], -17.0, 1e-9);
    EXPECT_NEAR(s.p[2], 8.0, 1e-9);
}

TEST(KalmanParityEstimator, Contract13LiftoffUsesLatestPreflightAnchor) {
    app::EskfEstimator est;
    app::EskfEstimator::Config cfg{};
    cfg.gps_delay_us = 0;
    cfg.enable_gps_cog_heading = 0;
    cfg.enable_gps_cog_heading_fusion = 0;
    est.configure(cfg);
    est.reset();

    const auto first_preflight = makeValidGnss(100000, 485000000, 23500000, 120000);
    const auto latest_preflight = makeValidGnss(900000, 485009000, 23500000, 120000);
    est.processGpsSample(first_preflight);
    est.processGpsSample(latest_preflight);

    est.onLiftoff(1000);

    const auto inflight_fix = makeValidGnss(2000000, 485009000, 23500000, 120000);
    est.processGpsSample(inflight_fix);
    est.onTick(2500000);

    const auto& s = est.filter().state();
    EXPECT_LT(std::abs(s.p[0]), 5.0);
    EXPECT_LT(std::abs(s.p[1]), 5.0);
}

TEST(KalmanParityEstimator, Contract16PreflightBypassSkipsCatchup) {
    app::EskfEstimator est;
    app::EskfEstimator::Config cfg{};
    cfg.gps_delay_us = 0;
    est.configure(cfg);
    est.reset();

    app::ImuSample samples[4] = {};
    for (int i = 0; i < 4; ++i) {
      samples[i].ax = 0.0f;
      samples[i].ay = 0.0f;
      samples[i].az = -9.80665f;
      samples[i].gx = 0.0f;
      samples[i].gy = 0.0f;
      samples[i].gz = 0.0f;
      samples[i].temperature = 0;
      samples[i].internal_timestamp = static_cast<uint16_t>(i);
    }

    for (int k = 0; k < 30; ++k) {
      app::ImuBatch batch{};
      batch.data = samples;
      batch.count = 4;
      batch.t0_us = static_cast<uint64_t>(k) * 4000ULL;
      batch.dt_us = 1000;
      batch.source = 0;
      est.processImuBatch(batch);
      est.onTick(batch.t0_us + 4000ULL);
    }

    EXPECT_FALSE(est.inFlight());
    EXPECT_EQ(est.filter().kalmanTimestamp(), 0u);
    EXPECT_GT(est.filter().imuBufferCount(), 0u);
    EXPECT_NE(est.railShadow().findCheckpointBefore(100000), nullptr);
}

TEST(KalmanParityEstimator, Contract20OnTickWrapDomainMonotonic) {
    app::EskfEstimator est;
    app::EskfEstimator::Config cfg{};
    cfg.gps_delay_us = 0;
    est.configure(cfg);
    est.configureReplaySensorCounts(1, 0);
    est.reset();
    est.onLiftoff(0);

    app::ImuSample samples[4] = {};
    for (int i = 0; i < 4; ++i) {
      samples[i].ax = 0.0f;
      samples[i].ay = 0.0f;
      samples[i].az = -9.80665f;
      samples[i].gx = 0.0f;
      samples[i].gy = 0.0f;
      samples[i].gz = 0.0f;
      samples[i].temperature = 0;
      samples[i].internal_timestamp = static_cast<uint16_t>(i);
    }

    const uint64_t kWrapUs = (1ULL << 32);

    for (int k = 0; k < 10; ++k) {
      app::ImuBatch before_wrap{};
      before_wrap.data = samples;
      before_wrap.count = 4;
      before_wrap.t0_us = (kWrapUs - 40000ULL) + static_cast<uint64_t>(k) * 4000ULL;
      before_wrap.dt_us = 1000;
      before_wrap.source = 0;
      est.processImuBatch(before_wrap);
    }
    est.onTick(kWrapUs - 2000ULL);

    const uint64_t ts_before = est.filter().kalmanTimestamp();
    EXPECT_GE(ts_before, kWrapUs - 40000ULL);
    EXPECT_LE(ts_before, kWrapUs - 2000ULL);

    for (int k = 0; k < 6; ++k) {
      app::ImuBatch after_wrap{};
      after_wrap.data = samples;
      after_wrap.count = 4;
      after_wrap.t0_us = (kWrapUs + 2000ULL) + static_cast<uint64_t>(k) * 4000ULL;
      after_wrap.dt_us = 1000;
      after_wrap.source = 0;
      est.processImuBatch(after_wrap);
    }
    est.onTick(kWrapUs + 30000ULL);

    const uint64_t ts_after = est.filter().kalmanTimestamp();
    EXPECT_GE(ts_after, kWrapUs);
    EXPECT_LE(ts_after, kWrapUs + 30000ULL);
    EXPECT_GT(ts_after, ts_before);
}
