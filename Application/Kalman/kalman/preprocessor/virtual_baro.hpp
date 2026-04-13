// Virtual Barometer Preprocessing Pipeline
// Part of Phase 2: ESKF Pre-Processing Layer
//
// Implements N-sensor fusion with voting-based outlier rejection for barometers:
// - Median-based voting for blocked port detection
// - Dynamic variance scaling: R = R_single / N_valid
//
// See Kalman_filter_planification.md Section 3.4

#pragma once
#include "../eskf_types.hpp"
#include "../eskf_config.hpp"
#include "sensor_calibration.hpp"
#include <cstddef>

namespace eskf {

// Forward declaration (SensorStatus defined in virtual_imu.hpp)
enum class SensorStatus : uint8_t;

/// Persistent barometer health lifecycle state.
enum class BaroHealthState : uint8_t {
  ACTIVE = 0,
  SUSPECT = 1,
  INHIBITED = 2,
  RECOVERING = 3,
};

// ============================================================
// Per-Baro Configuration
// ============================================================

/// Configuration for a single barometer sensor
struct BaroSensorConfig {
  /// Enable/disable this sensor
  bool enabled = false;
};

// ============================================================
// Virtual Baro Configuration
// ============================================================

/// Configuration for the Virtual Barometer preprocessing pipeline
struct VirtualBaroConfig {
  /// Per-sensor configurations (up to ESKF_MAX_BAROS)
  BaroSensorConfig baros[ESKF_MAX_BAROS];
  
  /// Number of configured barometers (0 to ESKF_MAX_BAROS)
  size_t baro_count = 0;
  
  /// Calibration data pointer (optional, null = no calibration)
  /// Pointer to array of BaroCalibration[ESKF_MAX_BAROS]
  const BaroCalibration* calibration = nullptr;
  
  /// Static pressure compensation settings
  StaticPressureCompensation static_pressure;
  
  /// Voting threshold (Pascals) - reject if |P_i - median| > this
  /// 500 Pa ≈ 40m altitude difference at sea level
  eskf_scalar voting_threshold_pa = 500.0;

  /// Hard-fault threshold (Pascals) - strict, persistent health candidate.
  eskf_scalar hard_fault_threshold_pa = 3000.0;

  /// Additional tolerance for stable calibration mismatch before hard fault.
  eskf_scalar calibration_mismatch_tolerance_pa = 600.0;

  /// Persistence / hysteresis for health lifecycle transitions.
  uint16_t hard_fault_suspect_samples = 2;
  uint16_t hard_fault_persistence_samples = 4;
  uint16_t recovery_cooldown_samples = 8;
  uint16_t recovery_confirm_samples = 8;

  /// Continuity clamp for inhibit/recover transitions.
  eskf_scalar continuity_max_step_pa = 200.0;

  /// Deterministic salvage when all sensors are soft-rejected.
  bool enable_all_soft_reject_salvage = true;

    /// Per-sensor stale/frozen sample detection.
    /// If both pressure and temperature remain unchanged (within thresholds) for
    /// stale_persistence_samples consecutive samples, the sensor contributes a
    /// hard-fault observation for persistent-health transitions.
    uint16_t stale_persistence_samples = 128;
    eskf_scalar stale_pressure_delta_threshold_pa =
      static_cast<eskf_scalar>(1e-6);
    eskf_scalar stale_temperature_delta_threshold_k =
      static_cast<eskf_scalar>(1e-6);
  
  /// Single sensor variance (m²) for altitude measurement
  eskf_scalar single_sensor_variance = 1.0;
  
  /// Enable voting (auto-disabled if only 1 sensor)
  bool voting_enabled = true;
};

// ============================================================
// Virtual Baro Output
// ============================================================

/// Fused barometer output
struct BaroOutput {
  /// Fused pressure (Pa)
  eskf_scalar pressure_pa;
  
  /// Fused temperature (K)
  eskf_scalar temperature_k;
  
  /// Measurement variance: R_single / N_valid (m²)
  eskf_scalar variance;
  
  /// Timestamp of measurement (trigger time + conversion/2)
  uint64_t timestamp_us;
  
  /// Per-sensor status after voting
  SensorStatus baro_status[ESKF_MAX_BAROS];
  
  /// Number of valid sensors used in fusion
  size_t valid_count;

  /// True when output is degraded-but-usable.
  bool degraded_output;

  /// True if deterministic all-soft-reject salvage was used.
  bool continuity_salvage_used;

  /// Per-sensor persistent health state.
  BaroHealthState baro_health[ESKF_MAX_BAROS];

  /// Per-sensor hard-fault persistence counters.
  uint16_t baro_hard_fault_counter[ESKF_MAX_BAROS];
};

// ============================================================
// Virtual Baro Class
// ============================================================

/// Virtual Barometer Preprocessing Pipeline
///
/// Fuses N barometers into a single high-fidelity virtual sensor:
/// 1. Check data validity (skip HARD_FAIL sensors)
/// 2. Apply tare offsets (if set) to normalize to virtual ground
/// 3. Median voting to detect blocked ports
/// 4. Average valid sensors
/// 5. Scale variance: R = R_single / N_valid
class VirtualBaro {
 public:
  VirtualBaro() = default;

  /// Configure the preprocessor
  void configure(const VirtualBaroConfig& cfg);

  /// Reset internal state
  void reset();
  
  /// Set per-sensor tare offsets for normalization.
  /// Applied AFTER calibration, BEFORE voting/averaging.
  /// Offset_i = virtual_ground - sensor_i_average (from pre-flight calibration)
  /// Corrected = raw + offset
  /// @param offsets_pa Per-sensor offset array [ESKF_MAX_BAROS]
  void setTareOffsets(const eskf_scalar offsets_pa[ESKF_MAX_BAROS]);
  
  /// Check if tare offsets are active
  bool hasTareOffsets() const { return tare_offsets_active_; }

  /// Process N raw baro readings, return fused output
  ///
  /// @param pressures   Raw pressure array [count] (Pa)
  /// @param temps       Raw temperature array [count] (K)
  /// @param statuses    Per-sensor status flags [count]
  /// @param trigger_us  Trigger timestamp (acquisition = trigger + conversion/2)
  /// @return            Fused output with variance scaling
  BaroOutput process(
      const eskf_sensor_t* pressures,
      const eskf_sensor_t* temps,
      const SensorStatus* statuses,
      uint64_t trigger_us);

  /// Process with static pressure compensation (Section 3.4.D)
  ///
  /// @param pressures        Raw pressure array [count] (Pa)
  /// @param temps            Raw temperature array [count] (K)
  /// @param statuses         Per-sensor status flags [count]
  /// @param trigger_us       Trigger timestamp
  /// @param velocity_mag     Velocity magnitude (m/s) for Cp correction
  /// @return                 Fused output with Cp compensation applied
  BaroOutput processWithVelocity(
      const eskf_sensor_t* pressures,
      const eskf_sensor_t* temps,
      const SensorStatus* statuses,
      uint64_t trigger_us,
      eskf_scalar velocity_mag);

 private:
  VirtualBaroConfig cfg_{};
  
  /// Per-sensor tare offsets for ground normalization
  eskf_scalar tare_offsets_[ESKF_MAX_BAROS] = {0};
  bool tare_offsets_active_ = false;

  // Persistent health lifecycle
  BaroHealthState health_state_[ESKF_MAX_BAROS] = {};
  uint16_t hard_fault_counter_[ESKF_MAX_BAROS] = {};
  uint16_t healthy_counter_[ESKF_MAX_BAROS] = {};
  uint16_t stale_counter_[ESKF_MAX_BAROS] = {};
  bool has_prev_sample_[ESKF_MAX_BAROS] = {};
  eskf_scalar prev_pressure_pa_[ESKF_MAX_BAROS] = {};
  eskf_scalar prev_temperature_k_[ESKF_MAX_BAROS] = {};

  // Last fused output for continuity shaping across inhibit/recover transitions
  bool has_last_fused_output_ = false;
  eskf_scalar last_fused_pressure_pa_ = 0;
  eskf_scalar last_fused_temperature_k_ = 0;

  /// Compute median of N scalar values
  eskf_scalar computeMedian(const eskf_scalar* data,
                            const bool* valid,
                            size_t count) const;
};

} // namespace eskf
