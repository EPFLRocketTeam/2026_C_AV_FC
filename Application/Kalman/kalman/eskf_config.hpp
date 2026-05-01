#pragma once
// ESKF Configuration Header - Compile-time settings for Error-State Kalman Filter
// Part of Phase 1: Standalone Core ESKF Library
//
// This file includes:
// - eskf_tuning_config.hpp: Runtime TuningConfig struct (preferred)
// - eskf_tuning_defaults.hpp: Compile-time macro defaults (backward compat)
// - eskf_sizes.hpp: Buffer sizes and structural constants

#include "eskf_tuning_config.hpp"
#include "eskf_tuning_defaults.hpp"
#include "eskf_sizes.hpp"
#include "target_platform.hpp"

namespace eskf {
namespace constants {
  // Local gravity computed from default latitude
  // For runtime injection, use TuningConfig::local_gravity instead
  constexpr eskf_scalar kGravityLocal = computeLocalGravity(ESKF_LAUNCH_SITE_LATITUDE_DEG);

  // ISA sea-level air density (kg/m^3).
  constexpr eskf_scalar kAirDensitySeaLevel = static_cast<eskf_scalar>(1.225);
}
} // namespace eskf
