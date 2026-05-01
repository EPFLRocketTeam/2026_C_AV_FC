// Sensor Calibration Parameters
// Part of Phase 2: ESKF Pre-Processing Layer
//
// Contains hardcoded lab calibration values for:
// - IMU: Thermal, Ellipsoid (Accel), Scale Factor (Gyro)
// - Barometer: Bias, Scale, Static Pressure Compensation
// - Magnetometer: Hard Iron, Soft Iron, Declination
//
// See Kalman_filter_planification.md Section 6 for calibration protocols.
//
// USAGE:
// 1. Perform lab calibration procedures (Section 6 Phases 1-5)
// 2. Fill in values below
// 3. Recompile

#pragma once
#include "../eskf_config.hpp"
#include "../eskf_math.hpp"  // For math::mat3Vec3Multiply
#include <cmath>

namespace eskf {

// ============================================================
// IMU Calibration Parameters (Section 6 Phases 1-4)
// ============================================================

/// Thermal polynomial coefficients for bias drift compensation
/// Bias(T) = coeff[0]*dT³ + coeff[1]*dT² + coeff[2]*dT + coeff[3]
/// where dT = T_current - reference_temp_k
struct ThermalPoly {
  eskf_scalar coeff[4] = {0, 0, 0, 0};  // [a, b, c, d] for 3rd order
};

/// Per-IMU calibration parameters
struct ImuCalibration {
  // --- Calibration Status Flags ---
  /// True if ellipsoid fitting (accel bias + transform) has been performed
  bool ellipsoid_calibrated = false;
  
  /// True if thermal polynomials have been calibrated
  bool thermal_calibrated = false;
  
  // --- Accelerometer Calibration (Section 6 Phase 2: Ellipsoid Fitting) ---
  /// Accelerometer bias (m/s²) - center offset from sphere fitting
  eskf_scalar accel_bias[3] = {0, 0, 0};
  
  /// Accelerometer transform matrix (3x3) - combines scale and skew
  /// Applied as: a_cal = transform * (a_raw - bias)
  eskf_scalar accel_transform[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  
  // --- Gyroscope Calibration (Section 6 Phase 3: Scale Factor) ---
  /// Gyroscope bias (rad/s) - constant offset to remove
  /// Applied as: omega_unbiased = omega_raw - gyro_bias
  eskf_scalar gyro_bias[3] = {0, 0, 0};
  
  /// Gyroscope scale factors per axis (dimensionless)
  /// Applied as: omega_cal = (omega_raw - gyro_bias) * scale
  eskf_scalar gyro_scale[3] = {1.0, 1.0, 1.0};

  /// Gyro g-sensitivity compensation matrix.
  /// Compensation model: omega_corr = omega_meas - G * a_cal
  /// Units: rad/s per (m/s^2)
  eskf_scalar gyro_g_sensitivity[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
  };
  
  // --- Thermal Calibration (Section 6 Phase 1: Heat Ramp) ---
  /// Reference temperature (K) - the "zero point" for thermal correction
  /// Typically 25°C (298.15K) or power-on temperature
  eskf_scalar reference_temp_k = 298.15;
  
  /// Thermal polynomials for each accelerometer axis
  ThermalPoly accel_thermal[3];  // X, Y, Z
  
  /// Thermal polynomials for each gyroscope axis
  ThermalPoly gyro_thermal[3];   // X, Y, Z

  /// Optional calibrated temperature range bounds (K).
  /// When valid (max > min), dT is clamped to this range relative to
  /// reference_temp_k before polynomial evaluation.
  eskf_scalar thermal_valid_min_temp_k = 0;
  eskf_scalar thermal_valid_max_temp_k = 0;
};

// ============================================================
// Barometer Calibration Parameters
// ============================================================

/// Per-barometer calibration parameters
struct BaroCalibration {
  /// Pressure bias (Pa) - additive offset
  eskf_scalar pressure_bias_pa = 0.0;
  
  /// Pressure scale factor (dimensionless)
  /// Applied as: p_cal = (p_raw - bias) * scale
  eskf_scalar pressure_scale = 1.0;
  
  /// Temperature bias (K) - offset for internal temp sensor
  eskf_scalar temperature_bias_k = 0.0;
};

/// Static Pressure Compensation (Section 3.4.D)
/// Corrects for Bernoulli effect on static ports
struct StaticPressureCompensation {
  /// Pressure coefficient (dimensionless)
  /// P_static = P_meas - 0.5 * rho * v² * Cp
  /// Typical values: -0.1 to +0.1 for good ports, up to ±0.5 for poor designs
  /// MUST be 0.0 for maiden flights - only set after post-flight analysis
  eskf_scalar cp_coefficient = 0.0;
  
  /// Air density (kg/m³) - for dynamic pressure calculation
  /// Default: ISA sea level (1.225 kg/m³)
  eskf_scalar air_density = constants::kAirDensitySeaLevel;
};

// ============================================================
// Magnetometer Calibration Parameters (Section 6 Phase 5)
// ============================================================

/// Magnetometer calibration parameters
struct MagCalibration {
  // --- Hard Iron Correction (Section 6 Phase 5.2) ---
  /// Hard iron bias (μT) - geometric center offset from permanent magnets
  eskf_scalar hard_iron_bias[3] = {0, 0, 0};
  
  // --- Soft Iron Correction (Section 6 Phase 5.2) ---
  /// Soft iron matrix (3x3) - corrects ellipsoid to sphere
  /// Applied as: B_cal = soft_iron * (B_raw - hard_iron)
  eskf_scalar soft_iron_matrix[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  
  // --- Residual Variance (Section 6 Phase 5.3) ---
  /// Calibration residual variance (μT²)
  /// Calculated from the variance of magnitudes after ellipsoid fitting.
  /// Used to set R_mag in the Kalman Filter.
  eskf_scalar variance_ut_sq = 1.0;

  /// Rotation matrix from magnetometer sensor frame to body frame.
  eskf_scalar sensor_to_body[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  
  // Note: Site-specific parameters (declination_rad, expected_magnitude_ut,
  // magnitude_threshold_ut, expected_dip_rad, dip_threshold_rad) are in
  // TuningConfig, not here. They depend on launch location AND must be
  // runtime-configurable for replay with different sites.
};

// ============================================================
// GPS Calibration Parameters (Section 6.2, 6.4.B)
// ============================================================

/// GPS calibration parameters
struct GpsCalibration {
  // --- Antenna Lever-Arm (Section 3.2) ---
  /// GPS antenna position in body frame (meters)
  /// Vector from CG to antenna phase center, used for velocity correction.
  eskf_scalar lever_arm_body[3] = {0, 0, 0};

  /// Antenna datum-referenced position in body frame (m).
  eskf_scalar antenna_position_datum[3] = {0, 0, 0};

  /// True when antenna_position_datum should be used to derive dynamic lever arm.
  bool antenna_position_valid = false;
  
  // Note: trust_factor and vel_lag_us are algorithm tuning parameters
  // and belong in TuningConfig, not hardware calibration.
};

// ============================================================
// Calibration Data Arrays (Per-Sensor)
// ============================================================

/// Complete calibration data for all sensors
struct SensorCalibrationData {
  /// Per-IMU calibration (up to ESKF_MAX_IMUS sensors)
  ImuCalibration imu[ESKF_MAX_IMUS];
  
  /// Per-barometer calibration (up to ESKF_MAX_BAROS sensors)
  BaroCalibration baro[ESKF_MAX_BAROS];
  
  /// Static pressure compensation (shared across all baros)
  StaticPressureCompensation static_pressure;
  
  /// Magnetometer calibration (single sensor)
  MagCalibration mag;
  
  /// GPS calibration
  GpsCalibration gps;
};

// ============================================================
// Helper Functions
// ============================================================

/// Clamp dT to the calibrated IMU thermal range when bounds are valid.
/// Falls back to passthrough when bounds are not configured.
inline eskf_scalar clampThermalDeltaTempToCalibrationBounds(
    const ImuCalibration &cal,
    eskf_scalar dT) {
  if (!std::isfinite(dT)) {
    return 0;
  }

  const eskf_scalar min_temp = cal.thermal_valid_min_temp_k;
  const eskf_scalar max_temp = cal.thermal_valid_max_temp_k;
  if (!(std::isfinite(min_temp) && std::isfinite(max_temp) &&
        max_temp > min_temp)) {
    return dT;
  }

  const eskf_scalar min_dT = min_temp - cal.reference_temp_k;
  const eskf_scalar max_dT = max_temp - cal.reference_temp_k;
  if (dT < min_dT) {
    return min_dT;
  }
  if (dT > max_dT) {
    return max_dT;
  }
  return dT;
}

/// Evaluate 3rd order thermal polynomial
/// @param poly Polynomial coefficients [a, b, c, d]
/// @param dT Temperature difference from reference (K)
/// @return Bias correction value
inline eskf_scalar evalThermalPoly(const ThermalPoly& poly, eskf_scalar dT) {
  if (!std::isfinite(dT)) {
    dT = 0;
  }

  // Final guardrail against unstable cubic extrapolation when calibration
  // bounds are unavailable or malformed.
  constexpr eskf_scalar kMaxAbsDeltaTempK = 80.0;
  if (dT > kMaxAbsDeltaTempK) {
    dT = kMaxAbsDeltaTempK;
  } else if (dT < -kMaxAbsDeltaTempK) {
    dT = -kMaxAbsDeltaTempK;
  }

  // Bias(dT) = a*dT³ + b*dT² + c*dT + d
  return poly.coeff[0] * dT * dT * dT 
       + poly.coeff[1] * dT * dT 
       + poly.coeff[2] * dT 
       + poly.coeff[3];
}

/// Apply 3x3 matrix to 3-vector: out = M * v
/// @deprecated Use eskf::math::mat3Vec3Multiply from eskf_math.hpp instead
inline void applyMatrix3(const eskf_scalar M[3][3], 
                         const eskf_scalar v[3], 
                         eskf_scalar out[3]) {
  // Forward to eskf_math.hpp implementation for consistency
  math::mat3Vec3Multiply(out, M, v);
}

} // namespace eskf
