// Pressure to Altitude Conversion Implementation
// Part of Phase 2: ESKF Pre-Processing Layer
//
// Unified hypsometric equation for barometric altitude.

#include "pressure_altitude.hpp"
#include <cmath>

namespace eskf {

eskf_scalar pressureToAltitudeAgl(eskf_scalar pressure_pa, 
                                   const GroundReference& ground) {
  // Validate inputs
  if (!ground.valid || 
      ground.pressure_pa <= 0 || 
      ground.temperature_k <= 0 ||
      pressure_pa <= 0) {
    return 0;
  }
  
  // Hypsometric equation:
  // h = (T_ground / L_b) * ((P / P_ground)^exponent - 1)
  //
  // Note: L_b is negative (-0.0065), so division by L_b gives negative,
  // and (P/P_ground)^exp - 1 is negative when P < P_ground (ascending),
  // resulting in positive altitude. Math works out correctly.
  
  const eskf_scalar base = pressure_pa / ground.pressure_pa;
  const eskf_scalar power_term = std::pow(base, baro_constants::kExponent);
  const eskf_scalar altitude = (ground.temperature_k / baro_constants::kLapseRate) 
                               * (power_term - 1.0);
  
  return static_cast<eskf_scalar>(altitude);
}

eskf_scalar pressureToAltitudeIsa(eskf_scalar pressure_pa) {
  // ISA model with sea-level reference (fallback)
  if (pressure_pa <= 0) {
    return 0;
  }
  
  const eskf_scalar base = pressure_pa / baro_constants::kP0;
  const eskf_scalar power_term = std::pow(base, baro_constants::kExponent);
  const eskf_scalar altitude = (baro_constants::kT0 / baro_constants::kLapseRate) 
                               * (power_term - 1.0);
  
  return static_cast<eskf_scalar>(altitude);
}

} // namespace eskf
