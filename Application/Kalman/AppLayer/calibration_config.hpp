// CalibrationConfig - Full hardware calibration data for SD logging and replay
//
// This struct contains complete sensor calibration parameters that are saved
// to SD at startup, enabling:
// - Verification of correct calibration values
// - Archival for future reference
// - Simulation stress-testing with modified calibration
//
// Note: This is a separate record from ReplayableConfig (algorithm tuning)
// due to size (~600 bytes, which fragments across blocks automatically).

#pragma once
#include <cstdint>
#include <cstddef>

namespace appcfg {

/// Maximum number of CG table entries for time-varying CG model
constexpr size_t kMaxCgTableEntries = 8;

/// Maximum number of IMUs supported
constexpr size_t kMaxCalibImus = 2;

/// Time-varying CG lookup table entry
struct CgTableEntry {
  uint32_t time_ms;     ///< Time since liftoff (ms)
  float position[3];    ///< CG position at this time (m, body frame X=fwd, Y=right, Z=down)
};

/// Full calibration configuration for all sensors
/// Written to SD at startup for replay compatibility
struct CalibrationConfig {
  // Frame consistency contract (important):
  // - cg_table[].position, imu[].position, and gps.antenna_position_datum must
  //   all be expressed in the SAME body frame: X=forward, Y=right, Z=down.
  // - sensor_to_body matrices map vectors from sensor frame to that same body
  //   frame: v_body = R_sensor_to_body * v_sensor.
  //
  // Easy example:
  //   cg(t0) = [0.10, 0.00, 0.00] m
  //   antenna_position_datum = [0.45, 0.02, 0.00] m
  //   => lever_arm(t0) = antenna - cg = [0.35, 0.02, 0.00] m
  //
  //   If an IMU is mounted +90 deg about +Z relative to body:
  //     R_sensor_to_body =
  //       [ 0 -1  0 ]
  //       [ 1  0  0 ]
  //       [ 0  0  1 ]
  //   Then a sensor-frame vector is rotated into the body frame with this R.

  // ============================================================
  // IMU Calibration (2x IMU)
  // ============================================================
  struct ImuCal {
    // --- Status Flags ---
    uint8_t ellipsoid_calibrated;   ///< True if ellipsoid fitting performed
    uint8_t thermal_calibrated;     ///< True if thermal polynomials calibrated
    
    // --- Accelerometer Calibration ---
    float accel_bias[3];            ///< Accelerometer bias (m/s²)
    float accel_transform[3][3];    ///< Scale + skew matrix (3x3)
    
    // --- Gyroscope Calibration ---
    float gyro_bias[3];             ///< Gyro bias offset (rad/s)
    float gyro_scale[3];            ///< Gyro scale factors per axis

    // --- Gyro g-Sensitivity Compensation ---
    // Compensation model: omega_corr = omega_meas - G * a_cal
    // Units: rad/s per (m/s^2)
    float gyro_g_sensitivity[3][3]; ///< Gyro g-sensitivity matrix
    
    // --- Thermal Calibration ---
    float reference_temp_k;         ///< Reference temperature (K)
    float accel_thermal[3][4];      ///< Accel thermal poly: [axis][coeff a,b,c,d]
    float gyro_thermal[3][4];       ///< Gyro thermal poly: [axis][coeff a,b,c,d]
    float thermal_valid_min_temp_k; ///< Optional calibrated min temperature (K)
    float thermal_valid_max_temp_k; ///< Optional calibrated max temperature (K)
    
    // --- Lever-Arm Position ---
    float position[3];              ///< Sensor position in body frame (m)

    // --- Sensor Mounting Transform ---
    // Rotation from IMU sensor frame to body frame.
    float sensor_to_body[3][3];     ///< IMU mounting rotation (sensor->body)
  };
  ImuCal imu[kMaxCalibImus];        // ~188 bytes × 2 = ~376 bytes

  // ============================================================
  // Barometer Calibration
  // ============================================================
  struct BaroCal {
    float pressure_bias_pa;         ///< Pressure bias (Pa)
    float pressure_scale;           ///< Pressure scale factor
    float temperature_bias_k;       ///< Temperature bias (K)
  };
  BaroCal baro;                     // ~12 bytes

  // ============================================================
  // Static Pressure Compensation
  // ============================================================
  struct StaticPressureCal {
    float cp_coefficient;           ///< Bernoulli pressure coefficient
    float air_density;              ///< Air density (kg/m³)
  };
  StaticPressureCal static_pressure; // ~8 bytes

  // ============================================================
  // Magnetometer Calibration
  // ============================================================
  struct MagCal {
    float hard_iron_bias[3];        ///< Hard iron offset (μT)
    float soft_iron_matrix[3][3];   ///< Soft iron correction matrix
    float variance_ut_sq;           ///< Calibration variance (μT²)
    float sensor_to_body[3][3];     ///< Magnetometer mounting rotation (sensor->body)
    // Note: Site-specific params (declination, expected_magnitude, threshold, dip)
    // are in TuningConfig (replayable_config), not here.
  };
  MagCal mag;                       // ~44 bytes

  // ============================================================
  // GPS Calibration
  // ============================================================
  struct GpsCal {
    float lever_arm[3];             ///< Antenna position from CG (m)
    float antenna_position_datum[3];///< Antenna datum-referenced position (m)
    uint8_t antenna_position_valid; ///< 1 if antenna_position_datum should be used
    // Note: trust_factor and vel_lag_us are in TuningConfig
  };
  GpsCal gps;                       // ~12 bytes

  // ============================================================
  // CG Table (time-varying center of gravity)
  // ============================================================
  uint8_t cg_table_size;            ///< Number of valid entries (0-8)
  CgTableEntry cg_table[kMaxCgTableEntries];  // ~128 bytes

  // Total: ~616 bytes
} __attribute__((packed));

/// Get default calibration config from hw_calibration_data.hpp values
CalibrationConfig getDefaultCalibrationConfig();

}  // namespace appcfg
