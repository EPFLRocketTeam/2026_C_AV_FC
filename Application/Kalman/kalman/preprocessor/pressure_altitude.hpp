// Pressure to Altitude Conversion - Unified Hypsometric Equation
// Part of Phase 2: ESKF Pre-Processing Layer
//
// Single source of truth for barometric altitude calculation.
// Uses ISA lapse rate model with ground-calibrated reference.
//
// Reference: Kalman_filter_planification.md Section 3.4

#pragma once
#include "../eskf_config.hpp"

namespace eskf {

// ============================================================
// Ground Reference Structure
// ============================================================

/// Ground reference data for AGL altitude conversion.
/// Captured during pre-liftoff calibration (~5 seconds of pad time).
struct GroundReference {
  /// Virtual ground pressure (Pa) - averaged across all healthy baros
  eskf_scalar pressure_pa = 101325.0;
  
  /// Ground temperature (K) - captured at pad, used throughout flight
  eskf_scalar temperature_k = 288.15;
  
  /// Per-sensor tare offset (Pa) to normalize to virtual ground
  /// Offset_i = virtual_ground - sensor_i_average
  /// Applied BEFORE voting/averaging: corrected = raw + offset
  eskf_scalar per_baro_offset_pa[ESKF_MAX_BAROS] = {0};
  
  /// True if ground calibration is complete (enough samples collected)
  bool valid = false;
};

// ============================================================
// Pressure-to-Altitude Functions
// ============================================================

/// Convert pressure to Altitude Above Ground Level (AGL).
///
/// Uses the hypsometric (barometric) equation:
///   h = (T_ground / L_b) * ((P / P_ground)^exponent - 1)
///
/// Where:
///   T_ground = Ground temperature at pad (K) - fixed at pad time
///   P_ground = Ground pressure at pad (Pa)
///   L_b = ISA lapse rate (-0.0065 K/m)
///   exponent = R*L_b / (g0*M) ≈ 0.190263
///
/// @param pressure_pa Current measured pressure (Pa)
/// @param ground Ground reference from pre-flight calibration
/// @return Altitude AGL in meters (positive up), 0 if invalid reference
eskf_scalar pressureToAltitudeAgl(eskf_scalar pressure_pa, 
                                   const GroundReference& ground);

/// Convert pressure to absolute ISA altitude (relative to sea level).
///
/// Uses standard atmosphere: P0 = 101325 Pa, T0 = 288.15 K
/// This is a FALLBACK for when ground reference is unavailable.
///
/// @param pressure_pa Current measured pressure (Pa)
/// @return Altitude MSL in meters (ISA model)
eskf_scalar pressureToAltitudeIsa(eskf_scalar pressure_pa);

// ============================================================
// Physical Constants (ISA Model)
// ============================================================

namespace baro_constants {
  constexpr double kRgas = 8.31446;        // Universal gas constant (J/(mol·K))
  constexpr double kG0 = 9.80665;          // Standard gravity (m/s²)
  constexpr double kMair = 0.0289644;      // Molar mass of dry air (kg/mol)
  constexpr double kLapseRate = -0.0065;   // Temperature lapse rate (K/m)
  constexpr double kP0 = 101325.0;         // Sea-level pressure (Pa)
  constexpr double kT0 = 288.15;           // Sea-level temperature (K)
  
  // Pre-computed exponent: -(R * L_b) / (g0 * M) ≈ 0.190263
  // L_b is negative in the troposphere ISA model, so the extra minus yields
  // the standard positive exponent used in the hypsometric equation.
  constexpr double kExponent = -(kRgas * kLapseRate) / (kG0 * kMair);
}

} // namespace eskf
