#pragma once
// ESKF Core Filter Class
// Part of Phase 1: Standalone Core ESKF Library
//
// Implements the Error-State Kalman Filter for INS/GPS fusion.
// Uses sequential scalar updates (no matrix inversion required).

#include <cmath>
#include "eskf_types.hpp"
#include "eskf_math.hpp"
#include "eskf_logger.hpp"
#include "eskf_tuning_config.hpp"

namespace eskf {

/// Error-State Kalman Filter core implementation.
///
/// This filter uses the ESKF formulation where:
/// - Nominal state is integrated non-linearly (large quantities)
/// - Error state is estimated linearly (small perturbations)
/// - After each correction, error is injected into nominal and reset
///
/// Key features:
/// - Sequential scalar updates (no matrix inversion)
/// - Joseph-form covariance update (numerically stable)
/// - Altitude-dependent gravity model
/// - NIS monitoring for divergence detection
class EskfCore {
 public:
  EskfCore() = default;

  // ============================================================
  // Initialization
  // ============================================================

  /// Initialize filter with tuning configuration.
  /// Must be called before any other operations.
  /// @param cfg Tuning configuration parameters
  void init(const TuningConfig& cfg);

  /// Reset filter to default state (identity quaternion, zero everything else).
  void reset();

  /// Initialize filter with given state and covariance.
  /// @param initial_state Starting nominal state
  /// @param P0 Initial covariance configuration
  /// @param Q Process noise configuration
  void initialize(const State& initial_state,
                  const InitialCovariance& P0,
                  const ProcessNoise& Q);

  // ============================================================
  // Prediction Step (IMU-driven, high-rate)
  // ============================================================

  /// Propagate state and covariance using IMU measurement.
  /// @param imu CG-corrected IMU frame (body frame)
  /// @param dt Time step in seconds
  void predict(const ImuFrame& imu, eskf_scalar dt);

  // ============================================================
  // Correction Steps (Sequential scalar updates)
  // ============================================================

  /// Correct with GPS position measurement (3 scalar updates).
  /// @param pos_ned Position in NED frame (m)
  /// @param R Measurement variance for each axis (m²)
  void correctGpsPosition(const eskf_scalar pos_ned[3], const eskf_scalar R[3]);

   /// Correct with GPS velocity measurement using antenna lever arm.
  /// @param vel_ned Velocity in NED frame (m/s)
  /// @param R Measurement variance for each axis ((m/s)²)
  /// @param lever_arm_body Vector from CG to Antenna in Body Frame (m)
  void correctGpsVelocity(const eskf_scalar vel_ned[3], const eskf_scalar R[3],
                          const eskf_scalar lever_arm_body[3]);

  // Overload for backward compatibility (assumes zero lever arm)
  void correctGpsVelocity(const eskf_scalar vel_ned[3], const eskf_scalar R[3]);

  /// Correct with GPS velocity using pre-computed averaged lever arm velocity.
  /// Use this when ESKF_USE_LEVER_ARM_AVERAGING is enabled.
  /// The averaged_lever_vel_ned should be computed by accumulating (ω × r) in NED
  /// frame over the GPS internal filter window and averaging.
  /// @param vel_ned Velocity in NED frame (m/s)
  /// @param R Measurement variance for each axis ((m/s)²)
  /// @param lever_arm_body Vector from CG to Antenna in Body Frame (m) - for Jacobian
  /// @param averaged_lever_vel_ned Pre-computed averaged lever arm velocity in NED (m/s)
  void correctGpsVelocityWithAveragedLeverArm(
      const eskf_scalar vel_ned[3], const eskf_scalar R[3],
      const eskf_scalar lever_arm_body[3],
      const eskf_scalar averaged_lever_vel_ned[3]);

  /// Correct with barometer altitude measurement (1 scalar update).
  /// @param alt_m Measured altitude (m, positive up)
  /// @param R Measurement variance (m²)
  void correctBaroAltitude(eskf_scalar alt_m, eskf_scalar R);

  /// Correct with magnetometer heading measurement (1 scalar update).
  /// @param heading_rad Measured heading in NED frame (rad)
  /// @param R Measurement variance (rad²)
  void correctMagHeading(eskf_scalar heading_rad, eskf_scalar R);

  /// Force specific heading (yaw) state with optional velocity override.
  /// Used for post-launch kinematic alignment ("Hard Snap").
  /// @param heading_rad New heading in radians (NED)
  /// @param reset_covariance If true, reset yaw covariance to small value (~1°)
  /// @param explicit_vel_ned Optional: GPS velocity to OVERWRITE state.v (nullptr = rotate existing)
  /// @param rotate_position If true, rotate N/E position with yaw snap
  /// @param rotate_velocity If true and explicit velocity is not provided,
  /// rotate N/E velocity with yaw snap
  void forceYaw(eskf_scalar heading_rad, bool reset_covariance = true,
                const eskf_scalar* explicit_vel_ned = nullptr,
                bool rotate_position = true,
                bool rotate_velocity = true);

  /// Correct with barometer altitude - auto-compute variance from velocity.
  /// Uses velocity-dependent R: R = (σ_base + K_aero·v²)² with transonic penalty.
  /// This overload computes R internally based on current velocity state.
  /// @param alt_m Measured altitude (m, positive up)
  void correctBaroAltitudeAuto(eskf_scalar alt_m);

  /// Correct with barometer using Innovation Transport (Section 3.4.A).
  /// Innovation is calculated using snapshot from trigger time, then applied to CURRENT state.
  /// @param measured_alt Measured altitude from sensor (m, positive up)
  /// @param predicted_alt_at_trigger Altitude estimate at trigger time (from snapshot)
  /// @param R Measurement variance (m²)
  void correctBaroWithSnapshot(eskf_scalar measured_alt, 
                                eskf_scalar predicted_alt_at_trigger,
                                eskf_scalar R);

  /// Attempt heading alignment using GPS velocity (one-shot).
  /// Checks alignment gate conditions (speed, pitch, GPS accuracy, rotation rate).
  /// If conditions met, performs forceYaw and returns true.
  /// @param gps_vel_ned GPS velocity in NED frame (m/s)
  /// @param gps_sAcc GPS speed accuracy estimate (m/s)
  /// @return true if alignment was performed, false if conditions not met
  bool attemptHeadingAlignment(const eskf_scalar gps_vel_ned[3], eskf_scalar gps_sAcc);

  /// Check if heading has been aligned (one-shot flag).
  bool isHeadingAligned() const { return heading_aligned_; }

  /// Get magnitude of previous bias-corrected gyro (rad/s).
  eskf_scalar prevGyroMagnitude() const {
    return std::sqrt(prev_gyro_(0)*prev_gyro_(0) +
                     prev_gyro_(1)*prev_gyro_(1) +
                     prev_gyro_(2)*prev_gyro_(2));
  }

  /// Zero P cross-terms between attitude/gyrbias and velocity/position.
  /// After calling this, GPS velocity corrections have K_att = 0 because
  /// K_att = P_att_vel * H_vel^T * S^{-1} = 0 when P_att_vel = 0.
  /// This is state-covariance consistent — no mismatch between P and state.
  void decoupleAttitudeFromVelocity();
  
  /// Set heading alignment state (for hardcoded heading mode).
  void setHeadingAligned(bool aligned) { heading_aligned_ = aligned; }

  /// Check if heading has been initialized (first valid measurement received).
  bool isHeadingInitialized() const { return heading_initialized_; }

  /// Set heading initialization state.
  void setHeadingInitialized(bool initialized) { heading_initialized_ = initialized; }

    /// Process heading update with gating and optional resurrection logic.
  /// First heading = SNAP. Subsequent = EKF update with outlier rejection.
  /// @param heading_rad Measured heading (radians)
  /// @param R Measurement variance (radians²)
  /// @param fused_correction_event Event type used when an inlier heading is
  /// fused
    /// @param allow_resurrection If false, outliers are rejected but do not
    /// trigger hard-snap resurrection.
  /// @return Result of the update attempt
  HeadingUpdateResult processHeadingUpdate(
      eskf_scalar heading_rad, eskf_scalar R,
      EskfEventType fused_correction_event =
        EskfEventType::MagCorrection,
      bool allow_resurrection = true);

  /// Continuous GPS velocity heading correction.
  /// Identical math to correctMagHeading but with different variance.
  /// @param heading_rad GPS course (radians)
  /// @param R Measurement variance (radians²)
  void correctVelocityHeading(eskf_scalar heading_rad, eskf_scalar R);

  /// Reset yaw covariance to small value after a hard snap.
  void resetYawCovariance();

  /// Reacquire vertical channel from barometer with explicit covariance contract.
  /// Sets p_D from baro model, decorrelates p_D from all other states,
  /// and assigns P_DD = R_baro + P_baroBias.
  /// @param altitude_isa_m Barometric ISA/MSL altitude measurement (m, positive up)
  /// @param baro_measurement_variance Measurement variance used for reacquisition (m²)
  void resetVerticalChannelFromBaro(eskf_scalar altitude_isa_m,
                                    eskf_scalar baro_measurement_variance);

  /// Set the local gravity magnitude (Somigliana constant).
  /// @param g Local gravity at sea level (m/s²)
  void setLocalGravity(eskf_scalar g) { g_local_ = g; }

#if ESKF_ENABLE_SIDESLIP
  /// Correct with sideslip constraint (aerodynamic velocity constraint).
  /// Assumes lateral body velocity ≈ 0 during flight.
  /// @param R_lateral Measurement variance for lateral velocity ((m/s)²)
  void correctSideslip(eskf_scalar R_lateral);
#endif

  // ============================================================
  // Static Constraint Updates (ZUPT/ZARU)
  // ============================================================
  // These are synthetic measurements applied when the rocket is known to be
  // stationary on the pad. The application layer decides WHEN to call them.

  /// Apply Zero Velocity Update (ZUPT).
  /// Constrains velocity to zero when stationary on pad.
  /// @param R Measurement variance for each axis ((m/s)²)
  void applyZupt(const eskf_scalar R[3]);

  /// Apply Zero Angular Rate Update (ZARU).
  /// Estimates gyroscope bias when stationary (assumes ω_true = 0).
  /// @param R Measurement variance for each axis ((rad/s)²)
  void applyZaru(const eskf_scalar R[3]);

  // ============================================================
  // Accessors
  // ============================================================

  /// Get current nominal state.
  const State& state() const { return state_; }

  /// Get current error-state covariance.
  const Covariance& covariance() const { return cov_; }

  /// Get last computed NIS (Normalized Innovation Squared).
  eskf_scalar lastNIS() const { return last_nis_; }

  /// Count of consecutive high-NIS updates.
  uint8_t consecutiveHighNisCount() const {
    return consecutive_high_nis_count_;
  }

  /// Get last computed innovation (for debugging/tuning).
  eskf_scalar lastInnovation() const { return last_innovation_; }

  /// Check if filter has diverged (detected NaN or excessive NIS).
  bool hasDiverged() const { return diverged_; }

  /// Number of prediction steps where dt was clamped.
  uint32_t predictDtClampedCount() const { return predict_dt_clamped_count_; }

  /// Get current filter mode.
  FilterMode mode() const { return mode_; }

  /// Reset integration history (coning/sculling/trapezoidal).
  /// Call this after state injection (e.g., liftoff snap).
  void resetIntegrationState();

  /// Set filter mode directly (e.g., Settling → Flight transition).
  /// Logs the mode change event.
  void setMode(FilterMode m);

  // ============================================================
  // State Injection (for GPS rewind)
  // ============================================================

  /// Set nominal state directly.
  void setState(const State& s) { state_ = s; }

  /// Set covariance directly.
  void setCovariance(const Covariance& P) { cov_ = P; }

  /// Update process noise configuration (e.g. for GPS dropout safeguard).
  void updateProcessNoise(const ProcessNoise& Q) { Q_ = Q; }

  /// Modify process noise configuration by reference.
  ProcessNoise& processNoise() { return Q_; }

  /// Get current timestamp.
  uint64_t timestamp() const { return state_.timestamp_us; }

  /// Create a checkpoint of current filter state.
  Checkpoint createCheckpoint() const;

  /// Restore filter state from checkpoint.
  void restoreCheckpoint(const Checkpoint& cp);

  // ============================================================
  // Logging Helpers
  // ============================================================

  /// Create a state snapshot for logging.
  StateSnapshot createStateSnapshot() const;

  /// Create a covariance snapshot for logging.
  CovarianceSnapshot createCovarianceSnapshot() const;

  // ============================================================
  // GPS Packet Rejection (Innovation Gating)
  // ============================================================

  /// Compute GPS velocity innovation Chi-Squared (no state update).
  /// Used for packet rejection gating before updating state.
  /// @param vel_ned Measured velocity in NED (m/s)
  /// @param R Measurement variance per axis ((m/s)²)
  /// @param lever_arm_body GPS antenna lever arm from CG (m)
  /// @param innovations_out Output: innovation per axis (z - h)
  /// @param S_out Output: innovation variance per axis (H*P*H' + R)
  /// @return Total Chi-Squared: Σ(innovation² / S)
  eskf_scalar computeGpsVelocityInnovation(
      const eskf_scalar vel_ned[3], const eskf_scalar R[3],
      const eskf_scalar lever_arm_body[3],
      eskf_scalar innovations_out[3], eskf_scalar S_out[3]);

  /// Compute GPS position innovation Chi-Squared (no state update).
  /// @param pos_ned Measured position in NED (m)
  /// @param R Measurement variance per axis (m²)
  /// @param innovations_out Output: innovation per axis
  /// @param S_out Output: innovation variance per axis
  /// @return Total Chi-Squared: Σ(innovation² / S)
  eskf_scalar computeGpsPositionInnovation(
      const eskf_scalar pos_ned[3], const eskf_scalar R[3],
      eskf_scalar innovations_out[3], eskf_scalar S_out[3]);

  /// Inflate position covariance floor for recovery after prolonged rejection.
  /// Raises each position diagonal to at least the requested variance while
  /// preserving existing cross-covariances.
  void inflatePositionCovariance(eskf_scalar variance);

  /// Inflate velocity covariance floor for recovery after prolonged rejection.
  /// Raises each velocity diagonal to at least the requested variance while
  /// preserving existing cross-covariances.
  void inflateVelocityCovariance(eskf_scalar variance);

  /// Inflate barometer-bias covariance floor.
  /// Raises baro-bias diagonal to at least the requested variance.
  void inflateBaroBiasCovariance(eskf_scalar variance);

  /// Hard-reset full position covariance block after a position state teleport.
  /// Clears all position cross-terms and sets position diagonals to variance.
  void resetPositionCovariance(eskf_scalar variance);

  /// Hard-reset full velocity covariance block after a velocity state overwrite.
  /// Clears all velocity cross-terms and sets velocity diagonals to variance.
  void resetVelocityCovariance(eskf_scalar variance);

  /// Record latest GPS velocity gating diagnostics for state logging.
  void setGpsVelocityGateDebug(eskf_scalar chi2, eskf_scalar threshold,
                               bool accepted);

  /// Apply the cumulative heading-snap offset to a GPS NED position.
  /// Only active when the GPS origin was set during flight (late fix),
  /// where the origin is anchored to a position with heading error.
  /// forceYaw rotates the ESKF position but GPS NED values (computed from
  /// the pre-rotation origin) remain in the original frame. This offset
  /// shifts them into the post-rotation ESKF frame. Unlike a full rotation,
  /// a constant offset preserves the velocity direction.
  /// When the origin was set pre-flight (position near zero, no heading
  /// error), GPS NED values are already correct and no offset is applied.
  void correctGpsNed(eskf_scalar pos[3]) const {
    if (!late_gps_origin_) return;
    pos[0] += gps_ned_offset_n_;
    pos[1] += gps_ned_offset_e_;
  }

  /// Mark the GPS origin as having been set during flight (late fix).
  /// When true, forceYaw heading snaps will accumulate position offsets
  /// so that GPS NED values can be corrected for the origin's heading error.
  void setLateGpsOrigin(bool late) { late_gps_origin_ = late; }

 private:
  State state_{};
  Covariance cov_{};
  ProcessNoise Q_{};
  eskf_scalar last_nis_ = 0;
  eskf_scalar last_innovation_ = 0;           // Last scalar innovation (for debugging)
  uint8_t consecutive_high_nis_count_ = 0;    // Count of consecutive high NIS values
  uint8_t consecutive_low_nis_count_ = 0;     // Count of consecutive low NIS values (for recovery)
  uint32_t predict_dt_clamped_count_ = 0;
  bool diverged_ = false;
  bool nis_soft_diverged_ = false;            // NIS-based soft divergence (baro continues with inflated R)
  FilterMode mode_ = FilterMode::Settling;

  // Runtime Configuration
  TuningConfig cfg_;  // Algorithm tuning parameters
  eskf_scalar g_local_ = constants::kGravityLocal; // Local gravity (from config or defaults)

  // Coning & Integration State
  math::Vector3 prev_gyro_ = math::Vector3::Zero();      // For coning
  math::Vector3 prev_accel_body_ = math::Vector3::Zero(); // For sculling
  math::Vector3 prev_vel_ = math::Vector3::Zero();       // For trapezoidal position
  bool first_run_ = true;                    // To init previous values

  // Sparse Updates
  uint32_t cov_decimation_counter_ = 0;      // Counter for P update
  uint16_t imu_dynamics_log_counter_ = 0;    // Decimation for IMU_DYNAMICS logging
#if ESKF_COVARIANCE_DECIMATION > 1
  eskf_scalar cov_decimation_dt_acc_s_ = 0;  // Accumulated dt for deferred covariance propagation
  eskf_scalar cov_decimation_F_acc_[kDimError][kDimError] = {{0}};
  eskf_scalar cov_decimation_Q_acc_[kDimError][kDimError] = {{0}};
#endif
  
  // Heading Alignment State
  bool heading_aligned_ = false;             // One-shot flag for GPS-based alignment
  bool heading_initialized_ = false;         // First valid heading received
  int consecutive_heading_rejects_ = 0;      // Counter for resurrection logic

  // Cumulative position offset from forceYaw snaps (for GPS NED correction).
  // Only accumulated/applied when late_gps_origin_ is true.
  eskf_scalar gps_ned_offset_n_ = 0;
  eskf_scalar gps_ned_offset_e_ = 0;

  // True when the GPS origin was set during flight (late fix), meaning the
  // origin is anchored to a position with accumulated heading error.
  // NOT checkpointed — set once by the app layer and persists through rewinds.
  bool late_gps_origin_ = false;
  HeadingUpdateResult last_heading_update_result_ = HeadingUpdateResult::Ignored;
  eskf_scalar last_heading_meas_rad_ = 0;
  eskf_scalar last_heading_innovation_rad_ = 0;
  eskf_scalar last_heading_gate_threshold_ = 0;
  eskf_scalar last_gps_vel_chi2_ = 0;
  eskf_scalar last_gps_vel_chi2_threshold_ = 0;
  bool last_gps_vel_accepted_ = false;

  // ============================================================
  // Internal Methods
  // ============================================================

  /// Inject error state into nominal state and reset error to zero.
  /// @param dx Error state vector (16×1)
  void injectErrorState(const eskf_scalar dx[kDimError]);

  /// Compute state transition Jacobian F (15×15).
  /// @param F Output Jacobian matrix
  /// @param accel_body Specific force in body frame (m/s²)
  /// @param dt Time step (s)
  void computeF(eskf_scalar F[kDimError][kDimError],
                const eskf_scalar accel_body[3],
                eskf_scalar dt);

  /// Propagate covariance: P = F*P*F' + G*Q*G'*dt
  void propagateCovariance(const eskf_scalar F[kDimError][kDimError],
                           eskf_scalar dt);

  /// True when accelerometer bias state is frozen in current mode.
  bool freezeAccelBiasInFlight() const;

  /// True when gyroscope bias state is frozen in current mode.
  bool freezeGyroBiasInFlight() const;

  /// Zero selected IMU-bias covariance rows/cols when entering flight mode.
  void freezeFlightBiasCovariance();

  /// Check for NaN/Inf in state and covariance.
  /// Sets diverged_ flag if detected.
  void checkNumericalHealth();

  /// Flush deferred covariance propagation before asynchronous corrections.
  /// Needed when covariance decimation is enabled and updates arrive between
  /// decimation boundaries.
  void flushDeferredCovariancePropagation();

  /// Get gravity magnitude at current altitude.
  eskf_scalar gravityMagnitude() const;

  /// Get gravity vector in NED frame [0, 0, g].
  void gravityNed(eskf_scalar g_ned[3]) const;

  /// Shared heading scalar-update implementation with explicit event source.
  void correctHeadingWithEvent(eskf_scalar heading_rad, eskf_scalar R,
                               EskfEventType correction_event);
};

} // namespace eskf
