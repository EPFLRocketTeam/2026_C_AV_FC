// Hardware-Specific Sensor Calibration Data
// Application Layer - KTP Flight Computer
//
// This file contains lab calibration values for the specific sensors used
// in the KTP flight computer:
//   - 2x ICM-45605 IMUs
//   - 1x BMP581 Barometer  
//   - 1x MMC5983MA Magnetometer
//
// CALIBRATION STATUS:
//   All values below are PLACEHOLDERS (identity/zero). They must be replaced
//   with actual lab calibration values before flight. See docs/calibration_procedures.md
//   for the calibration protocols.
//
// NOTE: This file is application-specific. Other applications with different
// sensor configurations should create their own calibration data file.
//
// See Kalman_filter_planification.md Section 6 for full calibration theory.

#pragma once
#include "Application/Kalman/kalman/preprocessor/sensor_calibration.hpp"
#include "Application/Kalman/AppLayer/eskf_app_config.hpp"

namespace eskf {

// ============================================================
// IMU Calibration Data (2x ICM-45605)
// ============================================================
// 
// Calibration procedure (Section 6 Phases 1-4):
//   Phase 1: Thermal characterization - Heat ramp from -18°C to 45-60°C
//   Phase 2: Ellipsoid fitting - 20 static orientations, solve for bias+transform
//   Phase 3: Gyro scale - 5 full revolutions on each axis, magnitude integration
//   Phase 4: Inter-triad alignment - 60s smooth rotation, gravity vector matching
//
// Current status: UNCALIBRATED (identity transform, zero bias)

namespace detail {

// IMU 0: Primary ICM-45605
constexpr ImuCalibration kImu0Calibration = {
  // --- Status Flags ---
  .ellipsoid_calibrated = false,  // TODO: Set true after Phase 2
  .thermal_calibrated = false,    // TODO: Set true after Phase 1
  
  // --- Accelerometer Ellipsoid Fitting (Phase 2) ---
  // Center offset from sphere fitting (m/s²)
  .accel_bias = {0.0, 0.0, 0.0},  // TODO: Fill from ellipsoid fit
  
  // Transform matrix: combines scale and skew correction
  // Applied as: a_cal = transform * (a_raw - bias)
  .accel_transform = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  },  // TODO: Fill from ellipsoid fit
  
  // --- Gyroscope Calibration (Phase 3) ---
  // Constant bias offset (rad/s)
  .gyro_bias = {0.0, 0.0, 0.0},  // TODO: Fill from static test
  
  // Sensitivity correction per axis (dimensionless)
  .gyro_scale = {1.0, 1.0, 1.0},  // TODO: Fill from rotation test

  // Gyro g-sensitivity matrix (rad/s per (m/s^2))
  .gyro_g_sensitivity = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
  },
  
  // --- Thermal Compensation (Phase 1) ---
  // Reference temperature for polynomial evaluation (K)
  .reference_temp_k = 298.15,  // 25°C - adjust to match calibration reference
  
  // Thermal polynomials: Bias(dT) = a*dT³ + b*dT² + c*dT + d
  .accel_thermal = {
    {{0.0, 0.0, 0.0, 0.0}},  // X axis
    {{0.0, 0.0, 0.0, 0.0}},  // Y axis
    {{0.0, 0.0, 0.0, 0.0}}   // Z axis
  },  // TODO: Fill from heat ramp test
  
  .gyro_thermal = {
    {{0.0, 0.0, 0.0, 0.0}},  // X axis
    {{0.0, 0.0, 0.0, 0.0}},  // Y axis
    {{0.0, 0.0, 0.0, 0.0}}   // Z axis
  }   // TODO: Fill from heat ramp test
};

// IMU 1: Secondary ICM-45605
constexpr ImuCalibration kImu1Calibration = {
  // --- Status Flags ---
  .ellipsoid_calibrated = false,
  .thermal_calibrated = false,
  
  // --- Accelerometer Ellipsoid Fitting ---
  .accel_bias = {0.0, 0.0, 0.0},
  .accel_transform = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  },
  
  // --- Gyroscope Calibration ---
  .gyro_bias = {0.0, 0.0, 0.0},
  .gyro_scale = {1.0, 1.0, 1.0},
  .gyro_g_sensitivity = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
  },
  
  // --- Thermal Compensation ---
  .reference_temp_k = 298.15,
  .accel_thermal = {
    {{0.0, 0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0, 0.0}}
  },
  .gyro_thermal = {
    {{0.0, 0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0, 0.0}}
  }
};

} // namespace detail

/// Get IMU calibration for sensor at given index.
/// @param index Sensor index (0 = primary ICM-45605, 1 = secondary ICM-45605)
/// @return Reference to calibration data (default identity if index out of range)
inline const ImuCalibration& getImuCalibration(size_t index) {
  static constexpr ImuCalibration kImuCalibrations[2] = {
    detail::kImu0Calibration,
    detail::kImu1Calibration
  };
  static constexpr ImuCalibration kDefaultCal{};
  return (index < 2) ? kImuCalibrations[index] : kDefaultCal;
}

// ============================================================
// Barometer Calibration Data (1x BMP581)
// ============================================================
//
// The BMP581 has excellent factory calibration. These values are for
// additional fine-tuning if cross-sensor taring shows residual offset.
//
// Current status: Using factory defaults (zero bias, unity scale)

namespace detail {

constexpr BaroCalibration kBaro0Calibration = {
  // Pressure bias (Pa) - additive offset from factory truth
  .pressure_bias_pa = 0.0,  // TODO: Compare against reference barometer
  
  // Pressure scale factor - typically 1.0 for Bosch sensors
  .pressure_scale = 1.0,
  
  // Temperature bias (K) - offset in internal temp sensor
  .temperature_bias_k = 0.0
};

} // namespace detail

/// Get barometer calibration for sensor at given index.
/// @param index Sensor index (0 = BMP581)
/// @return Reference to calibration data
inline const BaroCalibration& getBaroCalibration(size_t index) {
  static constexpr BaroCalibration kBaroCalibrations[1] = {
    detail::kBaro0Calibration
  };
  static constexpr BaroCalibration kDefaultCal{};
  return (index < 1) ? kBaroCalibrations[index] : kDefaultCal;
}

// ============================================================
// Static Pressure Compensation (Bernoulli Correction)
// ============================================================
//
// Corrects for the effect of airflow over static ports.
// P_static = P_meas - 0.5 * rho * v² * Cp
//
// CRITICAL: cp_coefficient MUST be 0.0 for maiden flights!
// Only set after post-flight analysis of static vs GPS altitude.
//
// Current status: Disabled (cp_coefficient = 0)

namespace detail {

constexpr StaticPressureCompensation kStaticPressure = {
  // Pressure coefficient (dimensionless)
  // Typical values: -0.1 to +0.1 for good ports, up to ±0.5 for poor designs
  .cp_coefficient = 0.0,  // MUST be 0 for maiden flights
  
  // Air density (kg/m³) - for dynamic pressure calculation
  // Default: ISA sea level (1.225 kg/m³)
  .air_density = 1.225
};

} // namespace detail

/// Get static pressure compensation settings.
/// @return Reference to static pressure compensation data
inline const StaticPressureCompensation& getStaticPressureCompensation() {
  return detail::kStaticPressure;
}

// ============================================================
// Magnetometer Calibration Data (1x MMC5983MA)
// ============================================================
//
// CRITICAL: Magnetometer calibration MUST be performed with the
// system in FINAL FLIGHT CONFIGURATION:
//   - All batteries, screws, and electrical components in place
//   - System POWERED ON (PCB currents create magnetic fields)
//   - Calibration performed OUTDOORS away from rebar/wiring
//
// Calibration procedure (Section 6 Phase 5):
//   1. Figure-8 rotation capturing all 3D octants (~2000 samples)
//   2. Ellipsoid fit → hard iron bias + soft iron matrix
//   3. Variance extraction → measurement noise R_mag
//
// Site-specific parameters (declination, expected magnitude, thresholds)
// are in TuningConfig, not here - they depend on launch location.
//
// Current status: UNCALIBRATED (zero bias, identity transform)

namespace detail {

constexpr MagCalibration kMagCalibration = {
  // --- Hard Iron Correction (Section 6 Phase 5.2) ---
  // Geometric center offset from permanent magnets/currents (μT)
  .hard_iron_bias = {0.0, 0.0, 0.0},  // TODO: Fill from ellipsoid fit
  
  // --- Soft Iron Correction (Section 6 Phase 5.2) ---
  // 3x3 matrix correcting ellipsoid → sphere
  // Applied as: B_cal = soft_iron * (B_raw - hard_iron)
  .soft_iron_matrix = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  },  // TODO: Fill from ellipsoid fit
  
  // --- Residual Variance (Section 6 Phase 5.3) ---
  // Variance of magnitudes after calibration (μT²)
  // Sets R_mag in Kalman Filter
  .variance_ut_sq = 1.0,  // TODO: Calculate from calibration residuals

  // Magnetometer sensor frame aligned with body frame by default.
  .sensor_to_body = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  }
};

} // namespace detail

/// Get magnetometer calibration.
/// @return Reference to MMC5983MA calibration data
inline const MagCalibration& getMagCalibration() {
  return detail::kMagCalibration;
}

// ============================================================
// GPS Calibration Data
// ============================================================
//
// GPS calibration (Section 6.2, 6.4.B):
//   - Velocity lag: Determine via "Rocking Test" cross-correlation
//   - Lever arm: Physical measurement from CG to antenna phase center
//   - Trust factor: Inflate reported accuracy (GPS tends to be optimistic)
//
// Current status: Lever arm and trust factor set, velocity lag TBD

namespace detail {

constexpr GpsCalibration kGpsCalibration = {
  // --- Antenna Lever Arm (Section 3.2) ---
  // GPS antenna position relative to CG in body frame (meters)
  // X = forward (toward nose), Y = right, Z = down
  .lever_arm_body = {0.0, 0.0, 0.0},  // TODO: Measure from CAD/physical

  // Datum-referenced antenna position is disabled by default in firmware data.
  .antenna_position_datum = {0.0, 0.0, 0.0},
  .antenna_position_valid = false
  
  // Note: trust_factor and vel_lag_us are in TuningConfig
};

} // namespace detail

/// Get GPS calibration.
/// @return Reference to GPS calibration data
inline const GpsCalibration& getGpsCalibration() {
  return detail::kGpsCalibration;
}

// ============================================================
// IMU Lever-Arm Positions
// ============================================================
//
// Sensor positions in body frame, used for lever-arm corrections.
// These should match the values in eskf_app_config.hpp.

namespace detail {

/// IMU 0 position in body frame (meters, X=fwd, Y=right, Z=down)
constexpr eskf_scalar kImu0Position[3] = {
  ESKF_IMU0_POS_X, ESKF_IMU0_POS_Y, ESKF_IMU0_POS_Z
};

/// IMU 1 position in body frame (meters)
constexpr eskf_scalar kImu1Position[3] = {
  ESKF_IMU1_POS_X, ESKF_IMU1_POS_Y, ESKF_IMU1_POS_Z
};

constexpr eskf_scalar kIdentity3x3[3][3] = {
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0}
};

} // namespace detail

/// Get IMU position for lever-arm correction.
/// @param index IMU index (0 or 1)
/// @return Pointer to 3-element position array
inline const eskf_scalar* getImuPosition(size_t index) {
  return (index == 0) ? detail::kImu0Position : detail::kImu1Position;
}

/// Get IMU sensor-to-body rotation matrix (3x3).
inline const eskf_scalar (*getImuSensorToBody(size_t index))[3] {
  (void)index;
  return detail::kIdentity3x3;
}

/// Get magnetometer sensor-to-body rotation matrix (3x3).
inline const eskf_scalar (*getMagSensorToBody())[3] {
  return detail::kIdentity3x3;
}

// ============================================================
// CG (Center of Gravity) Table
// ============================================================
//
// Time-varying CG for propellant burn modeling.
// Each entry specifies CG position at a given time since liftoff.
// Linear interpolation is used between entries.

namespace detail {

/// Default CG table with single entry (constant CG)
/// TODO: Populate with actual burn curve from motor data
constexpr size_t kCgTableSize = 1;
constexpr eskf_scalar kCgTable[8][4] = {
  // {time_ms, x, y, z}
  {0, 0.0, 0.0, 0.0},  // t=0: Static CG position
  {0, 0.0, 0.0, 0.0},  // Unused
  {0, 0.0, 0.0, 0.0},
  {0, 0.0, 0.0, 0.0},
  {0, 0.0, 0.0, 0.0},
  {0, 0.0, 0.0, 0.0},
  {0, 0.0, 0.0, 0.0},
  {0, 0.0, 0.0, 0.0},
};

} // namespace detail

/// Get CG table size (number of valid entries)
inline size_t getCgTableSize() {
  return detail::kCgTableSize;
}

/// Get CG table entry.
/// @param index Entry index (0 to getCgTableSize()-1)
/// @return Pointer to {time_ms, x, y, z} array
inline const eskf_scalar* getCgTableEntry(size_t index) {
  return (index < 8) ? detail::kCgTable[index] : detail::kCgTable[0];
}

} // namespace eskf

// ============================================================
// CalibrationConfig Population
// ============================================================

#include "Application/Kalman/AppLayer/calibration_config.hpp"
#include "Application/Kalman/AppLayer/eskf_app_config.hpp"

namespace ktp {

/// Populate CalibrationConfig from eskf calibration data
inline CalibrationConfig getDefaultCalibrationConfig() {
  CalibrationConfig cfg{};
  
  // --- IMU Calibration ---
  for (size_t i = 0; i < kMaxCalibImus; ++i) {
    const auto& imu = eskf::getImuCalibration(i);
    
    cfg.imu[i].ellipsoid_calibrated = imu.ellipsoid_calibrated ? 1 : 0;
    cfg.imu[i].thermal_calibrated = imu.thermal_calibrated ? 1 : 0;
    
    for (int j = 0; j < 3; ++j) {
      cfg.imu[i].accel_bias[j] = static_cast<float>(imu.accel_bias[j]);
      cfg.imu[i].gyro_bias[j] = static_cast<float>(imu.gyro_bias[j]);
      cfg.imu[i].gyro_scale[j] = static_cast<float>(imu.gyro_scale[j]);
      cfg.imu[i].position[j] = static_cast<float>(eskf::getImuPosition(i)[j]);
      
      for (int k = 0; k < 3; ++k) {
        cfg.imu[i].accel_transform[j][k] = static_cast<float>(imu.accel_transform[j][k]);
        cfg.imu[i].gyro_g_sensitivity[j][k] =
            static_cast<float>(imu.gyro_g_sensitivity[j][k]);
        cfg.imu[i].sensor_to_body[j][k] =
            static_cast<float>(eskf::getImuSensorToBody(i)[j][k]);
      }
      
      for (int c = 0; c < 4; ++c) {
        cfg.imu[i].accel_thermal[j][c] = static_cast<float>(imu.accel_thermal[j].coeff[c]);
        cfg.imu[i].gyro_thermal[j][c] = static_cast<float>(imu.gyro_thermal[j].coeff[c]);
      }
    }
    cfg.imu[i].reference_temp_k = static_cast<float>(imu.reference_temp_k);
    cfg.imu[i].thermal_valid_min_temp_k =
      static_cast<float>(imu.thermal_valid_min_temp_k);
    cfg.imu[i].thermal_valid_max_temp_k =
      static_cast<float>(imu.thermal_valid_max_temp_k);
  }
  
  // --- Barometer Calibration ---
  const auto& baro = eskf::getBaroCalibration(0);
  cfg.baro.pressure_bias_pa = static_cast<float>(baro.pressure_bias_pa);
  cfg.baro.pressure_scale = static_cast<float>(baro.pressure_scale);
  cfg.baro.temperature_bias_k = static_cast<float>(baro.temperature_bias_k);
  
  // --- Static Pressure Compensation ---
  const auto& static_p = eskf::getStaticPressureCompensation();
  cfg.static_pressure.cp_coefficient = static_cast<float>(static_p.cp_coefficient);
  cfg.static_pressure.air_density = static_cast<float>(static_p.air_density);
  
  // --- Magnetometer Calibration ---
  // Note: Site-specific params (declination, magnitude, threshold) are in TuningConfig
  const auto& mag = eskf::getMagCalibration();
  for (int j = 0; j < 3; ++j) {
    cfg.mag.hard_iron_bias[j] = static_cast<float>(mag.hard_iron_bias[j]);
    for (int k = 0; k < 3; ++k) {
      cfg.mag.soft_iron_matrix[j][k] = static_cast<float>(mag.soft_iron_matrix[j][k]);
      cfg.mag.sensor_to_body[j][k] =
          static_cast<float>(eskf::getMagSensorToBody()[j][k]);
    }
  }
  cfg.mag.variance_ut_sq = static_cast<float>(mag.variance_ut_sq);
  
  // --- GPS Calibration ---
  const auto& gps = eskf::getGpsCalibration();
  for (int j = 0; j < 3; ++j) {
    cfg.gps.lever_arm[j] = static_cast<float>(gps.lever_arm_body[j]);
    cfg.gps.antenna_position_datum[j] =
        static_cast<float>(gps.antenna_position_datum[j]);
  }
  cfg.gps.antenna_position_valid = gps.antenna_position_valid ? 1 : 0;
  
  // --- CG Table ---
  cfg.cg_table_size = static_cast<uint8_t>(eskf::getCgTableSize());
  for (size_t i = 0; i < kMaxCgTableEntries && i < eskf::getCgTableSize(); ++i) {
    const eskf_scalar* entry = eskf::getCgTableEntry(i);
    cfg.cg_table[i].time_ms = static_cast<uint32_t>(entry[0]);
    cfg.cg_table[i].position[0] = static_cast<float>(entry[1]);
    cfg.cg_table[i].position[1] = static_cast<float>(entry[2]);
    cfg.cg_table[i].position[2] = static_cast<float>(entry[3]);
  }
  
  return cfg;
}

}  // namespace ktp

