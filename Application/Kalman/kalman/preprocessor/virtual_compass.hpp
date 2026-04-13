// Virtual Compass Preprocessing Pipeline
// Part of Phase 2: ESKF Pre-Processing Layer
//
// Implements magnetometer calibration with:
// - Hard Iron correction (bias removal)
// - Soft Iron correction (ellipsoid to sphere)
// - Magnetic declination correction (magnetic to true north)
// - Magnitude validation (outlier rejection)
//
// See Kalman_filter_planification.md Section 6 Phase 5 for calibration protocols.

#pragma once
#include "../eskf_types.hpp"
#include "../eskf_config.hpp"
#include "sensor_calibration.hpp"
#include <cstdint>

namespace eskf {

// ============================================================
// Virtual Compass Configuration
// ============================================================

/// Configuration for the Virtual Compass preprocessing pipeline
struct VirtualCompassConfig {
  /// Magnetometer calibration parameters (hard/soft iron only)
  MagCalibration calibration;

  /// Mounting rotation from magnetometer sensor frame to body frame.
  eskf_scalar sensor_to_body[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };
  
  // --- Site-Specific Validation (from TuningConfig) ---
  /// Expected Earth field magnitude at launch site (μT)
  /// Look up from NOAA for launch site (typically 25-65 μT)
  eskf_scalar expected_magnitude_ut = 50.0;
  
  /// Magnitude rejection threshold (μT)
  /// Reject if |measured - expected| > threshold
  eskf_scalar magnitude_threshold_ut = 10.0;
  
  /// Enable magnitude-based outlier rejection
  bool magnitude_check_enabled = true;
  
  // Note: Declination is applied externally via calculateTiltCompensatedHeading()
  // Note: Dip angle validation is not implemented here - requires attitude quaternion
};

// ============================================================
// Virtual Compass Output
// ============================================================

/// Calibrated magnetometer output
struct CompassOutput {
  /// Calibrated magnetic field vector (μT), rotated to True North frame
  eskf_scalar mag_calibrated[3];
  
  /// Computed heading (radians), declination-corrected
  /// Measured clockwise from True North (0 = North, π/2 = East)
  eskf_scalar heading_rad;
  
  /// Calibrated field magnitude (μT)
  eskf_scalar magnitude_ut;
  
  /// Computed dip angle (radians) - angle of field below horizontal
  /// Positive = field points downward (Northern hemisphere typical)
  eskf_scalar dip_angle_rad;
  
  /// True if magnitude check passed (or check disabled)
  bool valid;
  
  /// True if dip angle check passed (or check disabled)
  bool dip_valid;
  
  /// Timestamp of measurement (microseconds)
  uint64_t timestamp_us;
};

// ============================================================
// Virtual Compass Class
// ============================================================

/// Virtual Compass Preprocessing Pipeline
///
/// Applies calibration to raw magnetometer data:
/// 1. Hard Iron correction: remove bias
/// 2. Soft Iron correction: transform ellipsoid to sphere
/// 3. Magnitude validation: reject if outside expected range
/// 4. Declination rotation: convert to True North frame
class VirtualCompass {
 public:
  VirtualCompass() = default;

  /// Configure the preprocessor
  void configure(const VirtualCompassConfig& cfg);

  /// Reset internal state
  void reset();

  /// Process raw magnetometer reading
  ///
  /// @param raw_mag   Raw magnetometer vector [3] (μT)
  /// @param timestamp_us  Measurement timestamp
  /// @return          Calibrated output with heading
  CompassOutput process(const eskf_sensor_t raw_mag[3], uint64_t timestamp_us);

  /// Get current configuration (const access)
  const VirtualCompassConfig& config() const { return cfg_; }

 private:
  VirtualCompassConfig cfg_{};
};

} // namespace eskf
