#pragma once

#include "Application/Data/data.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/Kalman/AppLayer/apogee_algorithm.hpp"
#include "Application/Kalman/AppLayer/state_estimator.hpp"

namespace app {

flight_computer::NavigationData mapEstimatorToNavigation(
    const EstimatorOutput& output,
    const flight_computer::bmp3_data& baro,
    const flight_computer::Vector3& body_accel_mps2);

ApogeeInput buildApogeeInput(const EstimatorOutput& output,
                             bool eskf_diverged,
                             flight_computer::State state,
                             float body_accel_x_mps2,
                             uint32_t liftoff_ms,
                             uint32_t now_ms);

}  // namespace app
