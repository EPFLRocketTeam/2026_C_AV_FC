#include "Application/Kalman/AppLayer/apogee_factory.hpp"
#include "Application/Kalman/AppLayer/apogee_hub.hpp"

#include <gtest/gtest.h>

namespace {

app::ApogeeInput nominalConsensusInput(uint32_t t_since_liftoff_ms) {
    app::ApogeeInput in{};
    in.eskf_velocity_down_mps = 5.0f;
    in.eskf_valid = true;
    in.eskf_diverged = false;
    in.shadow_velocity_down_mps = 4.0f;
    in.shadow_valid = true;
    in.body_accel_x_mps2 = 0.0f;
    in.altitude_m = 1200.0f;
    in.is_coast_phase = true;
    in.time_since_liftoff_ms = t_since_liftoff_ms;
    return in;
}

}  // namespace

TEST(ApogeeHubSuite, TriggersWhenPrimaryConsensusDetects) {
    app::ApogeeHub hub;
    ASSERT_GE(hub.addAlgorithm(app::createConsensusApogeeDetector()), 0);
    hub.setPrimary(0);
    hub.arm(1000);

    const app::ApogeeInput input = nominalConsensusInput(3000);
    const app::ApogeeDecision decision = hub.update(5000, input);

    EXPECT_TRUE(decision.triggered);
    ASSERT_EQ(decision.result_count, 1u);
    EXPECT_EQ(decision.primary_verdict.result,
              app::ApogeeVerdict::Result::Detected);
}

TEST(ApogeeHubSuite, ShadowEarlyUsesTimeoutOverride) {
    app::ApogeeHub hub;
    ASSERT_GE(hub.addAlgorithm(app::createConsensusApogeeDetector()), 0);
    hub.setPrimary(0);
    hub.arm(1000);

    app::ApogeeInput input{};
    input.eskf_velocity_down_mps = -10.0f;  // ESKF still ascending
    input.eskf_valid = true;
    input.shadow_velocity_down_mps = 2.0f; // Shadow says descending
    input.shadow_valid = true;
    input.body_accel_x_mps2 = 0.0f;
    input.altitude_m = 900.0f;
    input.is_coast_phase = true;
    input.time_since_liftoff_ms = 3000;

    const app::ApogeeDecision first = hub.update(4000, input);
    EXPECT_FALSE(first.triggered);
    EXPECT_EQ(first.primary_verdict.result,
              app::ApogeeVerdict::Result::WaitingTimer);

    const app::ApogeeDecision second = hub.update(5600, input);
    EXPECT_TRUE(second.triggered);
    EXPECT_EQ(second.primary_verdict.result,
              app::ApogeeVerdict::Result::Detected);
}
