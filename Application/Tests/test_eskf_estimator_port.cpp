#include "Application/Kalman/AppLayer/eskf_estimator.hpp"

#include <cmath>

#include <gtest/gtest.h>

TEST(EskfEstimatorPorted, BasicLifecycleSmoke) {
    app::EskfEstimator estimator;
    estimator.reset();

    app::ImuSample imu_sample{};
    app::ImuBatch imu_batch{};
    imu_batch.data = &imu_sample;
    imu_batch.count = 1;
    imu_batch.t0_us = 1000000;
    imu_batch.dt_us = 1000;
    imu_batch.source = 0;

    estimator.processImuBatch(imu_batch);
    estimator.onFlightStateChange(flight_computer::State::ASCENT);
    estimator.onLiftoff(1200);
    estimator.onTick(1200000);

    const app::EstimatorOutput out = estimator.output();
    EXPECT_TRUE(std::isfinite(out.altitude_m));
    EXPECT_TRUE(std::isfinite(out.vertical_velocity_mps));
}
