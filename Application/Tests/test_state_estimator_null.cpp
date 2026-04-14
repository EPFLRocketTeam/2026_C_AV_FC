#include "Application/Kalman/AppLayer/state_estimator.hpp"

#include <gtest/gtest.h>

TEST(NullEstimatorPorted, ComputesAltitudeAndVelocityFromBaroBatch) {
    app::NullEstimator est;
    est.reset();

    app::BaroSample samples[2] = {};
    samples[0].pressurePa = 101325.0f;
    samples[1].pressurePa = 100000.0f;

    app::BaroBatch batch{};
    batch.data = samples;
    batch.count = 2;
    batch.t0_us = 1000000;
    batch.dt_us = 10000;

    est.processBaroBatch(batch);
    const app::EstimatorOutput out = est.output();

    EXPECT_TRUE(out.altitude_valid);
    EXPECT_TRUE(out.velocity_valid);
    EXPECT_GT(out.altitude_m, -1.0f);
    EXPECT_NE(out.vertical_velocity_mps, 0.0f);
    EXPECT_EQ(out.last_update_ms, 1010u);
}

TEST(NullEstimatorPorted, LiftoffTimestampClampsLastUpdate) {
    app::NullEstimator est;
    est.reset();
    est.onLiftoff(5000);

    app::BaroSample sample{};
    sample.pressurePa = 101325.0f;

    app::BaroBatch batch{};
    batch.data = &sample;
    batch.count = 1;
    batch.t0_us = 1000000;
    batch.dt_us = 1000;

    est.processBaroBatch(batch);
    const app::EstimatorOutput out = est.output();

    EXPECT_EQ(out.last_update_ms, 5000u);
}
