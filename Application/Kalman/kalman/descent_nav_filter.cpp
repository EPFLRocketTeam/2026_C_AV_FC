#include "descent_nav_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace app {

namespace {

constexpr float kEps = 1e-9f;

inline float floorVariance(float value, float min_var) {
  if (!std::isfinite(value) || value < min_var) {
    return min_var;
  }
  return value;
}

}  // namespace

void DescentNavFilter::configure(const Config& cfg) { cfg_ = cfg; }

void DescentNavFilter::reset() {
  stats_ = {};
  active_ = false;
  timestamp_us_ = 0;
  baro_hold_until_us_ = 0;
  std::memset(x2_, 0, sizeof(x2_));
  std::memset(P2_, 0, sizeof(P2_));
  std::memset(x3_, 0, sizeof(x3_));
  std::memset(P3_, 0, sizeof(P3_));
}

void DescentNavFilter::enter(const InitData& init) {
  reset();
  active_ = true;
  timestamp_us_ = init.timestamp_us;

  for (size_t axis = 0; axis < 2; ++axis) {
    x2_[axis][0] = init.position_ned[axis];
    x2_[axis][1] = init.velocity_ned[axis];

    P2_[axis][0][0] = floorVariance(init.position_var_ned[axis], cfg_.min_variance);
    P2_[axis][1][1] = floorVariance(init.velocity_var_ned[axis], cfg_.min_variance);
    P2_[axis][0][1] = 0.0f;
    P2_[axis][1][0] = 0.0f;
  }

  x3_[0] = init.position_ned[2];
  x3_[1] = init.velocity_ned[2];
  x3_[2] = init.baro_bias_m;

  P3_[0][0] = floorVariance(init.position_var_ned[2], cfg_.min_variance);
  P3_[1][1] = floorVariance(init.velocity_var_ned[2], cfg_.min_variance);
  P3_[2][2] = floorVariance(init.baro_bias_var, cfg_.min_variance);
  P3_[0][1] = P3_[1][0] = 0.0f;
  P3_[0][2] = P3_[2][0] = 0.0f;
  P3_[1][2] = P3_[2][1] = 0.0f;
}

void DescentNavFilter::hardResetToGnss(uint64_t timestamp_us,
                                       const float position_ned[3],
                                       const float position_var_ned[3],
                                       const float velocity_ned[3],
                                       const float velocity_var_ned[3],
                                       bool reset_velocity) {
  if (!active_) {
    return;
  }

  const float prev_down = x3_[0];

  for (size_t axis = 0; axis < 2; ++axis) {
    x2_[axis][0] = position_ned[axis];
    if (reset_velocity) {
      x2_[axis][1] = velocity_ned[axis];
    }

    P2_[axis][0][0] = floorVariance(position_var_ned[axis], cfg_.min_variance);
    const float vel_var = floorVariance(velocity_var_ned[axis], cfg_.min_variance);
    P2_[axis][1][1] = reset_velocity ? vel_var : std::max(P2_[axis][1][1], vel_var);
    P2_[axis][0][1] = 0.0f;
    P2_[axis][1][0] = 0.0f;
  }

  x3_[0] = position_ned[2];
  if (reset_velocity) {
    x3_[1] = velocity_ned[2];
  }
  // Preserve h = -p_d + b_baro continuity across hard position snaps.
  x3_[2] += (x3_[0] - prev_down);

  P3_[0][0] = floorVariance(position_var_ned[2], cfg_.min_variance);
  const float down_vel_var = floorVariance(velocity_var_ned[2], cfg_.min_variance);
  P3_[1][1] = reset_velocity ? down_vel_var : std::max(P3_[1][1], down_vel_var);
  P3_[0][1] = P3_[1][0] = 0.0f;
  P3_[0][2] = P3_[2][0] = 0.0f;
  P3_[1][2] = P3_[2][1] = 0.0f;
  P3_[2][2] = floorVariance(P3_[2][2], cfg_.min_variance);

  if (timestamp_us > timestamp_us_) {
    timestamp_us_ = timestamp_us;
  }
}

float DescentNavFilter::clampAbs(float value, float limit, bool& clamped) {
  if (limit <= 0.0f) {
    return value;
  }
  if (value > limit) {
    clamped = true;
    return limit;
  }
  if (value < -limit) {
    clamped = true;
    return -limit;
  }
  return value;
}

void DescentNavFilter::predict(uint64_t timestamp_us) {
  if (!active_) {
    return;
  }
  if (timestamp_us <= timestamp_us_) {
    return;
  }

  const float dt = std::min(2.0f, static_cast<float>(timestamp_us - timestamp_us_) * 1e-6f);
  if (dt <= 0.0f) {
    return;
  }

  const float qa = cfg_.accel_process_sigma * cfg_.accel_process_sigma;
  const float qpp = 0.25f * qa * dt * dt * dt * dt;
  const float qpv = 0.5f * qa * dt * dt * dt;
  const float qvv = qa * dt * dt;

  for (size_t axis = 0; axis < 2; ++axis) {
    const float decay = std::exp(-std::max(0.0f, cfg_.horizontal_velocity_decay_per_s) * dt);

    // State prediction
    x2_[axis][0] += x2_[axis][1] * dt;
    x2_[axis][1] *= decay;

    // Covariance prediction for [p, v]
    const float p00 = P2_[axis][0][0];
    const float p01 = P2_[axis][0][1];
    const float p10 = P2_[axis][1][0];
    const float p11 = P2_[axis][1][1];

    const float F00 = 1.0f;
    const float F01 = dt;
    const float F10 = 0.0f;
    const float F11 = decay;

    const float FP00 = F00 * p00 + F01 * p10;
    const float FP01 = F00 * p01 + F01 * p11;
    const float FP10 = F10 * p00 + F11 * p10;
    const float FP11 = F10 * p01 + F11 * p11;

    P2_[axis][0][0] = FP00 * F00 + FP01 * F01 + qpp;
    P2_[axis][0][1] = FP00 * F10 + FP01 * F11 + qpv;
    P2_[axis][1][0] = FP10 * F00 + FP11 * F01 + qpv;
    P2_[axis][1][1] = FP10 * F10 + FP11 * F11 + qvv;

    P2_[axis][0][0] = floorVariance(P2_[axis][0][0], cfg_.min_variance);
    P2_[axis][1][1] = floorVariance(P2_[axis][1][1], cfg_.min_variance);
  }

  // Down-axis and baro-bias state prediction: x = [p_d, v_d, b_baro]
  x3_[0] += x3_[1] * dt;

  const float F[3][3] = {
      {1.0f, dt, 0.0f},
      {0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 1.0f},
  };

  float FP[3][3] = {{0}};
  float Pn[3][3] = {{0}};

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        FP[r][c] += F[r][k] * P3_[k][c];
      }
    }
  }

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        Pn[r][c] += FP[r][k] * F[c][k];
      }
    }
  }

  const float qb = cfg_.baro_bias_rw_sigma * cfg_.baro_bias_rw_sigma;
  Pn[0][0] += qpp;
  Pn[0][1] += qpv;
  Pn[1][0] += qpv;
  Pn[1][1] += qvv;
  Pn[2][2] += qb * dt;

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      P3_[r][c] = Pn[r][c];
    }
  }

  P3_[0][0] = floorVariance(P3_[0][0], cfg_.min_variance);
  P3_[1][1] = floorVariance(P3_[1][1], cfg_.min_variance);
  P3_[2][2] = floorVariance(P3_[2][2], cfg_.min_variance);

  timestamp_us_ = timestamp_us;
}

bool DescentNavFilter::fuseGnssPosition(uint64_t timestamp_us,
                                        const float position_ned[3],
                                        const float variance_ned[3]) {
  if (!active_) {
    return false;
  }

  predict(timestamp_us);

  bool accepted = false;
  accepted |= updateAxisPos(0, position_ned[0], variance_ned[0]);
  accepted |= updateAxisPos(1, position_ned[1], variance_ned[1]);
  accepted |= updateAxisPos(2, position_ned[2], variance_ned[2]);

  if (accepted) {
    stats_.gnss_pos_updates++;
  }

  return accepted;
}

bool DescentNavFilter::fuseGnssVelocity(uint64_t timestamp_us,
                                        const float velocity_ned[3],
                                        const float variance_ned[3]) {
  if (!active_) {
    return false;
  }

  predict(timestamp_us);

  bool accepted = false;
  accepted |= updateAxisVel(0, velocity_ned[0], variance_ned[0]);
  accepted |= updateAxisVel(1, velocity_ned[1], variance_ned[1]);
  accepted |= updateAxisVel(2, velocity_ned[2], variance_ned[2]);

  if (accepted) {
    stats_.gnss_vel_updates++;
  }

  return accepted;
}

bool DescentNavFilter::fuseBaroDown(uint64_t timestamp_us,
                                    float down_position_from_baro,
                                    float variance) {
  if (!active_) {
    return false;
  }

  predict(timestamp_us);

  if (timestamp_us < baro_hold_until_us_) {
    stats_.baro_hold_skips++;
    return false;
  }

  const bool accepted = updateDownWithBaro(down_position_from_baro, variance);
  if (accepted) {
    stats_.baro_updates++;
  }

  return accepted;
}

bool DescentNavFilter::updateAxisPos(size_t axis,
                                     float measurement,
                                     float variance) {
  variance = floorVariance(variance, cfg_.min_variance);

  if (axis < 2) {
    float& p = x2_[axis][0];
    float& v = x2_[axis][1];

    float& P00 = P2_[axis][0][0];
    float& P01 = P2_[axis][0][1];
    float& P10 = P2_[axis][1][0];
    float& P11 = P2_[axis][1][1];

    const float innovation = measurement - p;
    const float hPh = P00;
    float S = hPh + variance;
    if (!(S > kEps)) {
      stats_.gnss_pos_rejects++;
      return false;
    }

    const float norm = std::fabs(innovation) / std::sqrt(S);
    const float hard_gate_sigma = std::max(cfg_.hard_gate_sigma, cfg_.gate_sigma);
    if (norm > hard_gate_sigma) {
      stats_.gnss_pos_rejects++;
      return false;
    }

    const float weight = (norm > cfg_.huber_k && norm > kEps) ? (cfg_.huber_k / norm) : 1.0f;
    const float R_eff = variance / std::max(weight, 0.05f);
    S = hPh + R_eff;

    const float ph0 = P00;
    const float ph1 = P10;
    const float K0 = ph0 / S;
    const float K1 = ph1 / S;

    bool clamped = false;
    float dp = clampAbs(K0 * innovation, cfg_.max_position_correction_m, clamped);
    float dv = clampAbs(K1 * innovation, cfg_.max_velocity_correction_mps, clamped);

    p += dp;
    v += dv;

    P00 -= K0 * ph0;
    P01 -= K0 * ph1;
    P10 -= K1 * ph0;
    P11 -= K1 * ph1;

    const float sym = 0.5f * (P01 + P10);
    P01 = sym;
    P10 = sym;

    P00 = floorVariance(P00, cfg_.min_variance);
    P11 = floorVariance(P11, cfg_.min_variance);

    if (clamped) {
      stats_.gnss_pos_clamps++;
    }
    return true;
  }

  // Down-axis position update on 3-state model: h = [1, 0, 0]
  float h[3] = {1.0f, 0.0f, 0.0f};
  float ph[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ph[i] += P3_[i][j] * h[j];
    }
  }

  float hPh = 0.0f;
  for (int i = 0; i < 3; ++i) {
    hPh += h[i] * ph[i];
  }

  float S = hPh + variance;
  if (!(S > kEps)) {
    stats_.gnss_pos_rejects++;
    return false;
  }

  const float innovation = measurement - x3_[0];
  const float norm = std::fabs(innovation) / std::sqrt(S);
  const float hard_gate_sigma = std::max(cfg_.hard_gate_sigma, cfg_.gate_sigma);
  if (norm > hard_gate_sigma) {
    stats_.gnss_pos_rejects++;
    return false;
  }

  const float weight = (norm > cfg_.huber_k && norm > kEps) ? (cfg_.huber_k / norm) : 1.0f;
  const float R_eff = variance / std::max(weight, 0.05f);
  S = hPh + R_eff;

  float K[3] = {ph[0] / S, ph[1] / S, ph[2] / S};

  bool clamped = false;
  float dx0 = clampAbs(K[0] * innovation, cfg_.max_position_correction_m, clamped);
  float dx1 = clampAbs(K[1] * innovation, cfg_.max_velocity_correction_mps, clamped);
  float dx2 = clampAbs(K[2] * innovation, cfg_.max_baro_bias_correction_m, clamped);

  x3_[0] += dx0;
  x3_[1] += dx1;
  x3_[2] += dx2;

  float Pnew[3][3] = {{0}};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Pnew[r][c] = P3_[r][c] - K[r] * ph[c];
    }
  }

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      P3_[r][c] = 0.5f * (Pnew[r][c] + Pnew[c][r]);
    }
  }

  P3_[0][0] = floorVariance(P3_[0][0], cfg_.min_variance);
  P3_[1][1] = floorVariance(P3_[1][1], cfg_.min_variance);
  P3_[2][2] = floorVariance(P3_[2][2], cfg_.min_variance);

  if (clamped) {
    stats_.gnss_pos_clamps++;
  }
  return true;
}

bool DescentNavFilter::updateAxisVel(size_t axis,
                                     float measurement,
                                     float variance) {
  variance = floorVariance(variance, cfg_.min_variance);

  if (axis < 2) {
    float& p = x2_[axis][0];
    float& v = x2_[axis][1];

    float& P00 = P2_[axis][0][0];
    float& P01 = P2_[axis][0][1];
    float& P10 = P2_[axis][1][0];
    float& P11 = P2_[axis][1][1];

    const float innovation = measurement - v;
    const float hPh = P11;
    float S = hPh + variance;
    if (!(S > kEps)) {
      stats_.gnss_vel_rejects++;
      return false;
    }

    const float norm = std::fabs(innovation) / std::sqrt(S);
    const float hard_gate_sigma = std::max(cfg_.hard_gate_sigma, cfg_.gate_sigma);
    if (norm > hard_gate_sigma) {
      stats_.gnss_vel_rejects++;
      return false;
    }

    const float weight = (norm > cfg_.huber_k && norm > kEps) ? (cfg_.huber_k / norm) : 1.0f;
    const float R_eff = variance / std::max(weight, 0.05f);
    S = hPh + R_eff;

    const float ph0 = P01;
    const float ph1 = P11;
    const float K0 = ph0 / S;
    const float K1 = ph1 / S;

    bool clamped = false;
    float dp = clampAbs(K0 * innovation, cfg_.max_position_correction_m, clamped);
    float dv = clampAbs(K1 * innovation, cfg_.max_velocity_correction_mps, clamped);

    p += dp;
    v += dv;

    P00 -= K0 * ph0;
    P01 -= K0 * ph1;
    P10 -= K1 * ph0;
    P11 -= K1 * ph1;

    const float sym = 0.5f * (P01 + P10);
    P01 = sym;
    P10 = sym;

    P00 = floorVariance(P00, cfg_.min_variance);
    P11 = floorVariance(P11, cfg_.min_variance);

    if (clamped) {
      stats_.gnss_vel_clamps++;
    }
    return true;
  }

  // Down-axis velocity update on 3-state model: h = [0, 1, 0]
  float h[3] = {0.0f, 1.0f, 0.0f};
  float ph[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ph[i] += P3_[i][j] * h[j];
    }
  }

  float hPh = 0.0f;
  for (int i = 0; i < 3; ++i) {
    hPh += h[i] * ph[i];
  }

  float S = hPh + variance;
  if (!(S > kEps)) {
    stats_.gnss_vel_rejects++;
    return false;
  }

  const float innovation = measurement - x3_[1];
  const float norm = std::fabs(innovation) / std::sqrt(S);
  const float hard_gate_sigma = std::max(cfg_.hard_gate_sigma, cfg_.gate_sigma);
  if (norm > hard_gate_sigma) {
    stats_.gnss_vel_rejects++;
    return false;
  }

  const float weight = (norm > cfg_.huber_k && norm > kEps) ? (cfg_.huber_k / norm) : 1.0f;
  const float R_eff = variance / std::max(weight, 0.05f);
  S = hPh + R_eff;

  float K[3] = {ph[0] / S, ph[1] / S, ph[2] / S};

  bool clamped = false;
  float dx0 = clampAbs(K[0] * innovation, cfg_.max_position_correction_m, clamped);
  float dx1 = clampAbs(K[1] * innovation, cfg_.max_velocity_correction_mps, clamped);
  float dx2 = clampAbs(K[2] * innovation, cfg_.max_baro_bias_correction_m, clamped);

  x3_[0] += dx0;
  x3_[1] += dx1;
  x3_[2] += dx2;

  float Pnew[3][3] = {{0}};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Pnew[r][c] = P3_[r][c] - K[r] * ph[c];
    }
  }

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      P3_[r][c] = 0.5f * (Pnew[r][c] + Pnew[c][r]);
    }
  }

  P3_[0][0] = floorVariance(P3_[0][0], cfg_.min_variance);
  P3_[1][1] = floorVariance(P3_[1][1], cfg_.min_variance);
  P3_[2][2] = floorVariance(P3_[2][2], cfg_.min_variance);

  if (clamped) {
    stats_.gnss_vel_clamps++;
  }
  return true;
}

bool DescentNavFilter::updateDownWithBaro(float measurement, float variance) {
  variance = floorVariance(variance, cfg_.min_variance);

  // z_baro = p_d + b_baro + n
  const float h[3] = {1.0f, 0.0f, 1.0f};

  float ph[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ph[i] += P3_[i][j] * h[j];
    }
  }

  float hPh = 0.0f;
  for (int i = 0; i < 3; ++i) {
    hPh += h[i] * ph[i];
  }

  float S = hPh + variance;
  if (!(S > kEps)) {
    stats_.baro_rejects++;
    return false;
  }

  const float expected = x3_[0] + x3_[2];
  const float innovation = measurement - expected;
  const float norm = std::fabs(innovation) / std::sqrt(S);
  if (norm > cfg_.gate_sigma) {
    stats_.baro_rejects++;
    return false;
  }

  const float weight = (norm > cfg_.huber_k && norm > kEps) ? (cfg_.huber_k / norm) : 1.0f;
  const float R_eff = variance / std::max(weight, 0.05f);
  S = hPh + R_eff;

  float K[3] = {ph[0] / S, ph[1] / S, ph[2] / S};

  bool clamped = false;
  float dx0 = clampAbs(K[0] * innovation, cfg_.max_position_correction_m, clamped);
  float dx1 = clampAbs(K[1] * innovation, cfg_.max_velocity_correction_mps, clamped);
  float dx2 = clampAbs(K[2] * innovation, cfg_.max_baro_bias_correction_m, clamped);

  x3_[0] += dx0;
  x3_[1] += dx1;
  x3_[2] += dx2;

  float Pnew[3][3] = {{0}};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Pnew[r][c] = P3_[r][c] - K[r] * ph[c];
    }
  }

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      P3_[r][c] = 0.5f * (Pnew[r][c] + Pnew[c][r]);
    }
  }

  P3_[0][0] = floorVariance(P3_[0][0], cfg_.min_variance);
  P3_[1][1] = floorVariance(P3_[1][1], cfg_.min_variance);
  P3_[2][2] = floorVariance(P3_[2][2], cfg_.min_variance);

  if (clamped) {
    stats_.baro_clamps++;
  }
  return true;
}

DescentNavFilter::State DescentNavFilter::state() const {
  State out{};
  out.active = active_;
  out.timestamp_us = timestamp_us_;

  out.position_ned[0] = x2_[0][0];
  out.position_ned[1] = x2_[1][0];
  out.position_ned[2] = x3_[0];

  out.velocity_ned[0] = x2_[0][1];
  out.velocity_ned[1] = x2_[1][1];
  out.velocity_ned[2] = x3_[1];

  out.baro_bias_m = x3_[2];

  out.position_var_ned[0] = floorVariance(P2_[0][0][0], cfg_.min_variance);
  out.position_var_ned[1] = floorVariance(P2_[1][0][0], cfg_.min_variance);
  out.position_var_ned[2] = floorVariance(P3_[0][0], cfg_.min_variance);

  out.velocity_var_ned[0] = floorVariance(P2_[0][1][1], cfg_.min_variance);
  out.velocity_var_ned[1] = floorVariance(P2_[1][1][1], cfg_.min_variance);
  out.velocity_var_ned[2] = floorVariance(P3_[1][1], cfg_.min_variance);

  out.baro_bias_var = floorVariance(P3_[2][2], cfg_.min_variance);

  return out;
}

}  // namespace app
