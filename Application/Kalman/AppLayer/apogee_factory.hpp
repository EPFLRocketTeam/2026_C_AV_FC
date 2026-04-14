#pragma once

#include "Application/Kalman/AppLayer/apogee_algorithm.hpp"

#include <memory>

namespace app {

std::unique_ptr<IApogeeAlgorithm> createConsensusApogeeDetector();

}  // namespace app
