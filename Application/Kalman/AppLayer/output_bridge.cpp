#include "Application/Kalman/AppLayer/output_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kRadToDeg = 57.29577951308232;

double clampAsinInput(double x) {
  return std::max(-1.0, std::min(1.0, x));
}

}  // namespace

namespace app {

flight_computer::NavigationData mapEstimatorToNavigation(
    const EstimatorOutput& output,
    const flight_computer::bmp3_data& baro,
    const flight_computer::Vector3& body_accel_mps2) {
  flight_computer::NavigationData nav{};

  nav.position_kalman = {
      output.position_ned[0], output.position_ned[1], output.position_ned[2]};
    nav.speed = {output.velocity_ned[0], output.velocity_ned[1], output.velocity_ned[2]};
  nav.accel = body_accel_mps2;

    const double qw = output.quaternion[0];
    const double qx = output.quaternion[1];
    const double qy = output.quaternion[2];
    const double qz = output.quaternion[3];

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll_deg = std::atan2(sinr_cosp, cosr_cosp) * kRadToDeg;

  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch_deg = std::asin(clampAsinInput(sinp)) * kRadToDeg;

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw_deg = std::atan2(siny_cosp, cosy_cosp) * kRadToDeg;

  nav.attitude = {roll_deg, pitch_deg, yaw_deg};
  // NavigationData::course is heading (yaw) in degrees, not track-over-ground.
  nav.course = yaw_deg;
  nav.altitude = output.altitude_m;
  nav.baro = baro;
  return nav;
}

ApogeeInput buildApogeeInput(const EstimatorOutput& output,
                             bool eskf_diverged,
                             bool is_coast_phase,
                             uint32_t liftoff_ms,
                             uint32_t now_ms) {
  ApogeeInput input{};
  input.altitude_m = output.altitude_m;
  input.eskf_velocity_down_mps = output.velocity_ned[2];
  input.shadow_velocity_down_mps = output.shadow_velocity_down_mps;
  input.eskf_valid = output.eskf_valid;
  input.shadow_valid = output.shadow_valid;
  input.eskf_diverged = eskf_diverged;
  input.body_accel_x_mps2 = output.body_accel_x_mps2;

  input.is_coast_phase = is_coast_phase;

  if (now_ms >= liftoff_ms) {
    input.time_since_liftoff_ms = now_ms - liftoff_ms;
  } else {
    input.time_since_liftoff_ms = 0;
  }

  return input;
}

}  // namespace app
