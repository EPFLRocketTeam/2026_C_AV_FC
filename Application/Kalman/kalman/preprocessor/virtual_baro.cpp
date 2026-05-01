// Virtual Barometer Preprocessing Pipeline Implementation
// Part of Phase 2: ESKF Pre-Processing Layer
//
// See Kalman_filter_planification.md Section 3.4 for algorithm details.

#include "virtual_baro.hpp"
#include "virtual_imu.hpp"  // For SensorStatus enum
#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>

namespace eskf {

void VirtualBaro::configure(const VirtualBaroConfig& cfg) {
  cfg_ = cfg;
  reset();
}

void VirtualBaro::reset() {
  // Clear tare offsets
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    tare_offsets_[i] = 0;
    health_state_[i] = BaroHealthState::ACTIVE;
    hard_fault_counter_[i] = 0;
    healthy_counter_[i] = 0;
    stale_counter_[i] = 0;
    has_prev_sample_[i] = false;
    prev_pressure_pa_[i] = 0;
    prev_temperature_k_[i] = 0;
  }
  tare_offsets_active_ = false;
  has_last_fused_output_ = false;
  last_fused_pressure_pa_ = 0;
  last_fused_temperature_k_ = 0;
}

void VirtualBaro::setTareOffsets(const eskf_scalar offsets_pa[ESKF_MAX_BAROS]) {
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    tare_offsets_[i] = offsets_pa[i];
  }
  tare_offsets_active_ = true;
}

eskf_scalar VirtualBaro::computeMedian(const eskf_scalar* data,
                                        const bool* valid,
                                        size_t count) const {
  eskf_scalar values[ESKF_MAX_BAROS];
  size_t n = 0;
  
  for (size_t i = 0; i < count; ++i) {
    if (valid[i]) {
      values[n++] = data[i];
    }
  }
  
  if (n == 0) {
    return 0;
  } else if (n == 1) {
    return values[0];
  }
  
  std::sort(values, values + n);
  if (n % 2 == 1) {
    return values[n / 2];
  } else {
    return (values[n / 2 - 1] + values[n / 2]) / 2;
  }
}

BaroOutput VirtualBaro::process(
    const eskf_sensor_t* pressures,
    const eskf_sensor_t* temps,
    const SensorStatus* statuses_in,
    uint64_t trigger_us) {
  
  BaroOutput out{};
  out.timestamp_us = trigger_us;
  out.degraded_output = false;
  out.continuity_salvage_used = false;
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    out.baro_health[i] = health_state_[i];
    out.baro_hard_fault_counter[i] = hard_fault_counter_[i];
  }
  
  if (cfg_.baro_count == 0) {
    out.valid_count = 0;
    out.variance = cfg_.single_sensor_variance;
    return out;
  }
  
  // Copy statuses (we modify during voting and health-state transitions)
  SensorStatus statuses[ESKF_MAX_BAROS];
  bool enabled_mask[ESKF_MAX_BAROS] = {};
  bool input_hard_mask[ESKF_MAX_BAROS] = {};
  bool soft_reject_mask[ESKF_MAX_BAROS] = {};
  bool hard_fault_mask[ESKF_MAX_BAROS] = {};
  bool has_data_mask[ESKF_MAX_BAROS] = {};

  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    enabled_mask[i] = cfg_.baros[i].enabled;
    input_hard_mask[i] = (statuses_in[i] != SensorStatus::OK);
    has_data_mask[i] = !input_hard_mask[i];
    statuses[i] = SensorStatus::HARD_FAIL;
  }
  for (size_t i = cfg_.baro_count; i < ESKF_MAX_BAROS; ++i) {
    statuses[i] = SensorStatus::HARD_FAIL;
  }
  
  // Build pre-vote valid mask from health state and input status.
  bool pre_vote_valid[ESKF_MAX_BAROS] = {};
  size_t pre_vote_count = 0;
  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    const bool blocked_by_health =
        (health_state_[i] == BaroHealthState::INHIBITED ||
         health_state_[i] == BaroHealthState::RECOVERING);
    pre_vote_valid[i] = enabled_mask[i] && !input_hard_mask[i] && !blocked_by_health;
    if (pre_vote_valid[i]) {
      statuses[i] = SensorStatus::OK;
      pre_vote_count++;
    } else if (health_state_[i] == BaroHealthState::INHIBITED) {
      statuses[i] = SensorStatus::INHIBITED;
    } else if (health_state_[i] == BaroHealthState::RECOVERING) {
      statuses[i] = SensorStatus::RECOVERING;
    }
  }
  
  // Extract pressure values for valid sensors, applying calibration
  eskf_scalar pressure_vals[ESKF_MAX_BAROS] = {};
  eskf_scalar temp_vals[ESKF_MAX_BAROS] = {};
  const eskf_scalar stale_pressure_eps =
      (std::isfinite(cfg_.stale_pressure_delta_threshold_pa) &&
       cfg_.stale_pressure_delta_threshold_pa > 0)
          ? cfg_.stale_pressure_delta_threshold_pa
          : static_cast<eskf_scalar>(0);
  const eskf_scalar stale_temp_eps =
      (std::isfinite(cfg_.stale_temperature_delta_threshold_k) &&
       cfg_.stale_temperature_delta_threshold_k > 0)
          ? cfg_.stale_temperature_delta_threshold_k
          : static_cast<eskf_scalar>(0);
  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    if (cfg_.calibration) {
      const BaroCalibration& cal = cfg_.calibration[i];
      // Apply calibration: p_cal = (p_raw - bias) * scale
      pressure_vals[i] = (pressures[i] - cal.pressure_bias_pa) * cal.pressure_scale;
      temp_vals[i] = temps[i] - cal.temperature_bias_k;
    } else {
      pressure_vals[i] = pressures[i];
      temp_vals[i] = temps[i];
    }
    
    // Apply tare offset (normalize to virtual ground)
    // This ensures sensor rejection doesn't cause altitude jumps
    if (tare_offsets_active_) {
      pressure_vals[i] += tare_offsets_[i];
    }

    if (cfg_.stale_persistence_samples == 0) {
      stale_counter_[i] = 0;
      continue;
    }

    if (!(enabled_mask[i] && has_data_mask[i])) {
      stale_counter_[i] = 0;
      has_prev_sample_[i] = false;
      continue;
    }

    bool stale_sample = false;
    if (has_prev_sample_[i]) {
      const eskf_scalar dp = std::abs(pressure_vals[i] - prev_pressure_pa_[i]);
      const eskf_scalar dt = std::abs(temp_vals[i] - prev_temperature_k_[i]);
      stale_sample = (dp <= stale_pressure_eps) && (dt <= stale_temp_eps);
    }

    prev_pressure_pa_[i] = pressure_vals[i];
    prev_temperature_k_[i] = temp_vals[i];
    has_prev_sample_[i] = true;

    if (stale_sample) {
      if (stale_counter_[i] < std::numeric_limits<uint16_t>::max()) {
        stale_counter_[i]++;
      }
      if (stale_counter_[i] >= cfg_.stale_persistence_samples) {
        hard_fault_mask[i] = true;
      }
    } else {
      stale_counter_[i] = 0;
    }
  }

  eskf_scalar median_p = 0;
  if (pre_vote_count > 0) {
    median_p = computeMedian(pressure_vals, pre_vote_valid, cfg_.baro_count);
  }

  // Soft vote and hard-fault candidate separation.
  if (pre_vote_count > 0) {
    for (size_t i = 0; i < cfg_.baro_count; ++i) {
      if (pre_vote_valid[i]) {
        eskf_scalar diff = std::abs(pressure_vals[i] - median_p);
        if (cfg_.voting_enabled && pre_vote_count > 1 &&
            diff > cfg_.voting_threshold_pa) {
          soft_reject_mask[i] = true;
        }
        const eskf_scalar hard_gate =
            cfg_.hard_fault_threshold_pa + cfg_.calibration_mismatch_tolerance_pa;
        if (hard_gate > 0 && diff > hard_gate) {
          hard_fault_mask[i] = true;
        }
      }
    }
  }

  // Persistent health-state transitions.
  bool state_transition = false;
  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    const BaroHealthState prev = health_state_[i];
    const bool hard_sample = input_hard_mask[i] || hard_fault_mask[i];
    const bool soft_sample = soft_reject_mask[i];
    BaroHealthState state = health_state_[i];

    switch (state) {
    case BaroHealthState::INHIBITED:
      if (!hard_sample && !soft_sample && enabled_mask[i]) {
        if (healthy_counter_[i] < std::numeric_limits<uint16_t>::max()) {
          healthy_counter_[i]++;
        }
        if (healthy_counter_[i] >= cfg_.recovery_cooldown_samples) {
          state = BaroHealthState::RECOVERING;
          healthy_counter_[i] = 0;
        }
      } else {
        healthy_counter_[i] = 0;
      }
      break;
    case BaroHealthState::RECOVERING:
      if (hard_sample) {
        state = BaroHealthState::INHIBITED;
        healthy_counter_[i] = 0;
      } else if (!soft_sample) {
        if (healthy_counter_[i] < std::numeric_limits<uint16_t>::max()) {
          healthy_counter_[i]++;
        }
        if (healthy_counter_[i] >= cfg_.recovery_confirm_samples) {
          state = BaroHealthState::ACTIVE;
          healthy_counter_[i] = 0;
          hard_fault_counter_[i] = 0;
        }
      } else {
        healthy_counter_[i] = 0;
      }
      break;
    case BaroHealthState::ACTIVE:
    case BaroHealthState::SUSPECT:
    default:
      if (hard_sample) {
        if (hard_fault_counter_[i] < std::numeric_limits<uint16_t>::max()) {
          hard_fault_counter_[i]++;
        }
        healthy_counter_[i] = 0;
      } else {
        if (hard_fault_counter_[i] > 0) {
          hard_fault_counter_[i]--;
        }
        if (state == BaroHealthState::SUSPECT && !soft_sample) {
          if (healthy_counter_[i] < std::numeric_limits<uint16_t>::max()) {
            healthy_counter_[i]++;
          }
          if (healthy_counter_[i] >= cfg_.recovery_confirm_samples) {
            state = BaroHealthState::ACTIVE;
            healthy_counter_[i] = 0;
          }
        } else {
          healthy_counter_[i] = 0;
        }
      }

      if (hard_fault_counter_[i] >= cfg_.hard_fault_persistence_samples) {
        state = BaroHealthState::INHIBITED;
      } else if (hard_fault_counter_[i] >= cfg_.hard_fault_suspect_samples) {
        state = BaroHealthState::SUSPECT;
      }
      break;
    }

    if (state != prev) {
      state_transition = true;
    }
    health_state_[i] = state;

    if (state == BaroHealthState::INHIBITED) {
      statuses[i] = SensorStatus::INHIBITED;
    } else if (state == BaroHealthState::RECOVERING) {
      statuses[i] = SensorStatus::RECOVERING;
    } else if (hard_sample) {
      statuses[i] = SensorStatus::HARD_FAIL;
    } else if (soft_reject_mask[i]) {
      statuses[i] = SensorStatus::SOFT_FAIL;
    } else if (enabled_mask[i] && !input_hard_mask[i]) {
      statuses[i] = SensorStatus::OK;
    } else {
      statuses[i] = SensorStatus::HARD_FAIL;
    }
  }

  // Fusion: average valid sensors
  eskf_scalar sum_p = 0;
  eskf_scalar sum_t = 0;
  size_t n_valid = 0;
  
  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    if (statuses[i] == SensorStatus::OK) {
      sum_p += pressure_vals[i];  // Use calibrated values
      sum_t += temp_vals[i];
      n_valid++;
    }
  }

  // Deterministic all-soft-reject salvage.
  if (n_valid == 0 && cfg_.enable_all_soft_reject_salvage && pre_vote_count > 0) {
    size_t best_idx = ESKF_MAX_BAROS;
    eskf_scalar best_diff = std::numeric_limits<eskf_scalar>::infinity();
    for (size_t i = 0; i < cfg_.baro_count; ++i) {
      const bool health_ok =
          (health_state_[i] == BaroHealthState::ACTIVE ||
           health_state_[i] == BaroHealthState::SUSPECT);
      if (!health_ok || !enabled_mask[i] || input_hard_mask[i] ||
          hard_fault_mask[i] || !soft_reject_mask[i]) {
        continue;
      }
      const eskf_scalar diff = std::abs(pressure_vals[i] - median_p);
      if (diff < best_diff ||
          (std::abs(diff - best_diff) < 1e-9 && i < best_idx)) {
        best_diff = diff;
        best_idx = i;
      }
    }
    if (best_idx < cfg_.baro_count) {
      statuses[best_idx] = SensorStatus::OK;
      sum_p = pressure_vals[best_idx];
      sum_t = temp_vals[best_idx];
      n_valid = 1;
      out.degraded_output = true;
      out.continuity_salvage_used = true;
    }
  }
  
  if (n_valid > 0) {
    out.pressure_pa = sum_p / n_valid;
    out.temperature_k = sum_t / n_valid;
    out.valid_count = n_valid;
    // Variance scaling: R = R_single / N_valid
    out.variance = cfg_.single_sensor_variance /
                   static_cast<eskf_scalar>(n_valid);

    // Continuity shaping when sensor-health state changes.
    if (state_transition && has_last_fused_output_) {
      const eskf_scalar delta = out.pressure_pa - last_fused_pressure_pa_;
      if (std::abs(delta) > cfg_.continuity_max_step_pa) {
        out.pressure_pa = last_fused_pressure_pa_ +
                          ((delta > 0) ? cfg_.continuity_max_step_pa
                                       : -cfg_.continuity_max_step_pa);
        out.degraded_output = true;
      }
    }

    has_last_fused_output_ = true;
    last_fused_pressure_pa_ = out.pressure_pa;
    last_fused_temperature_k_ = out.temperature_k;
  } else {
    // Preserve continuity by holding last fused value when all sensors are
    // unusable this frame.
    if (has_last_fused_output_) {
      out.pressure_pa = last_fused_pressure_pa_;
      out.temperature_k = last_fused_temperature_k_;
      out.degraded_output = true;
    } else {
      out.pressure_pa = 0;
      out.temperature_k = 0;
    }
    out.valid_count = 0;
    out.variance = cfg_.single_sensor_variance;
  }
  
  // Copy final statuses
  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    out.baro_status[i] = statuses[i];
  }
  for (size_t i = cfg_.baro_count; i < ESKF_MAX_BAROS; ++i) {
    out.baro_status[i] = SensorStatus::HARD_FAIL;
  }

  for (size_t i = 0; i < cfg_.baro_count; ++i) {
    out.baro_health[i] = health_state_[i];
    out.baro_hard_fault_counter[i] = hard_fault_counter_[i];
  }
  for (size_t i = cfg_.baro_count; i < ESKF_MAX_BAROS; ++i) {
    out.baro_health[i] = BaroHealthState::INHIBITED;
    out.baro_hard_fault_counter[i] = 0;
  }
  
  return out;
}

BaroOutput VirtualBaro::processWithVelocity(
    const eskf_sensor_t* pressures,
    const eskf_sensor_t* temps,
    const SensorStatus* statuses,
    uint64_t trigger_us,
    eskf_scalar velocity_mag) {
  
  // First, process normally with calibration
  BaroOutput out = process(pressures, temps, statuses, trigger_us);
  
  // Apply static pressure compensation (Section 3.4.D)
  // P_static = P_meas - 0.5 * rho * v^2 * Cp
  const auto& sp = cfg_.static_pressure;
  if (sp.cp_coefficient != 0.0 && velocity_mag > 1.0 && out.valid_count > 0) {
    eskf_scalar dynamic_correction = 0.5 * sp.air_density 
                                   * velocity_mag * velocity_mag 
                                   * sp.cp_coefficient;
    out.pressure_pa -= dynamic_correction;
  }
  
  return out;
}

} // namespace eskf
