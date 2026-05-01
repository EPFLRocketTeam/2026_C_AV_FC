// Virtual Compass Preprocessing Pipeline Implementation
// Part of Phase 2: ESKF Pre-Processing Layer
//
// See Kalman_filter_planification.md Section 6 Phase 5 for algorithm details.

#include "virtual_compass.hpp"
#include <cmath>

namespace eskf {

void VirtualCompass::configure(const VirtualCompassConfig& cfg) {
  cfg_ = cfg;
  reset();
}

void VirtualCompass::reset() {
  // No internal state to reset
}

CompassOutput VirtualCompass::process(const eskf_sensor_t raw_mag[3], 
                                       uint64_t timestamp_us) {
  CompassOutput out{};
  out.timestamp_us = timestamp_us;
  out.valid = true;
  out.dip_valid = true;
  
  const MagCalibration& cal = cfg_.calibration;
  
  // --- Step 1: Hard Iron Correction ---
  // Remove bias caused by permanent magnets (battery casings, buzzers, etc.)
  eskf_scalar corrected[3];
  for (int i = 0; i < 3; ++i) {
    corrected[i] = static_cast<eskf_scalar>(raw_mag[i]) - cal.hard_iron_bias[i];
  }
  
  // --- Step 2: Soft Iron Correction ---
  // Transform ellipsoid to sphere using calibration matrix
  // B_cal = soft_iron_matrix * (B_raw - hard_iron_bias)
  eskf_scalar calibrated[3];
  applyMatrix3(cal.soft_iron_matrix, corrected, calibrated);

  // --- Step 2b: Mounting Rotation (Sensor -> Body) ---
  // Calibrated hard/soft-iron output remains in sensor frame until this step.
  eskf_scalar body_mag[3];
  applyMatrix3(cfg_.sensor_to_body, calibrated, body_mag);
  
  // --- Step 3a: Magnitude Validation ---
  // Reject if magnitude differs significantly from expected Earth field
  out.magnitude_ut = std::sqrt(body_mag[0] * body_mag[0] +
                               body_mag[1] * body_mag[1] +
                               body_mag[2] * body_mag[2]);
  
  if (cfg_.magnitude_check_enabled) {
    eskf_scalar mag_error = std::abs(out.magnitude_ut - cfg_.expected_magnitude_ut);
    if (mag_error > cfg_.magnitude_threshold_ut) {
      // Field distorted - likely near launch rail steel or deployment charge
      out.valid = false;
    }
  }
  
  // --- Step 3b: Dip Angle Validation ---
  // NOTE: Body-frame dip validation removed. The dip angle calculated here
  // is only valid when the rocket is level. At vertical orientations, the
  // Earth's field enters through body X/Y, making this check unreliable.
  // Callers with access to the attitude quaternion can perform Earth-frame
  // dip validation by rotating mag_calibrated to NED first.
  eskf_scalar B_horiz =
      std::sqrt(body_mag[0] * body_mag[0] + body_mag[1] * body_mag[1]);
  out.dip_angle_rad = std::atan2(body_mag[2], B_horiz);  // Informational only
  // Dip check disabled by design - cannot validate without attitude
  
  // --- Step 4: Output Calibrated Vector ---
  // Declination is NOT applied here because it's a 2D rotation that only
  // makes sense in the Earth's horizontal plane. Applying it to body X/Y
  // corrupts the vector when the rocket is tilted.
  // Instead, declination is applied in calculateTiltCompensatedHeading()
  // after rotating the vector to the Earth frame.
  out.mag_calibrated[0] = body_mag[0];
  out.mag_calibrated[1] = body_mag[1];
  out.mag_calibrated[2] = body_mag[2];
  
  // --- Step 5: Note on Heading ---
  // Heading is NOT computed here because it requires attitude (quaternion)
  // from the filter to perform tilt-compensation.
  // Use eskf::math::calculateTiltCompensatedHeading() with the calibrated
  // vector (out.mag_calibrated), attitude quaternion, and declination.
  out.heading_rad = 0;  // Deprecated: callers should compute externally
  
  return out;
}

} // namespace eskf

