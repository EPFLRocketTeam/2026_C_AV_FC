#include "kalman/eskf_tuning_config.hpp"
#include "kalman/eskf_yieldable.hpp"

#include <gtest/gtest.h>

TEST(KalmanP0Smoke, InitializesAndCatchesUpEmptyTimeline) {
    eskf::EskfYieldable filter;

    const eskf::TuningConfig tuning = eskf::getDefaultTuningConfig();
    filter.init(tuning);

    eskf::State initial_state{};
    initial_state.setIdentity();

    const eskf::InitialCovariance p0 = eskf::InitialCovariance::defaults();
    const eskf::ProcessNoise q = eskf::ProcessNoise::defaults();
    filter.initialize(initial_state, p0, q);

    eskf::YieldableConfig cfg{};
    cfg.catchupBudgetUs = 1000;
    filter.configure(cfg);

    EXPECT_TRUE(filter.catchUp(0, cfg.catchupBudgetUs));
}
