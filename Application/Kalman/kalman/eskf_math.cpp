// ESKF Math Implementation
// Part of Phase 1: Standalone Core ESKF Library
//
// Implements quaternion operations and custom backend math functions.

#include "eskf_math.hpp"
#include <cmath>

namespace eskf {
namespace math {

// ============================================================
// QUATERNION OPERATIONS (Always compiled)
// ============================================================

void quatMultiply(eskf_scalar q_out[4],
                  const eskf_scalar q_a[4],
                  const eskf_scalar q_b[4]) {
  // Hamilton convention: q = w + xi + yj + zk
  // q_a ⊗ q_b multiplication
  const eskf_scalar aw = q_a[0], ax = q_a[1], ay = q_a[2], az = q_a[3];
  const eskf_scalar bw = q_b[0], bx = q_b[1], by = q_b[2], bz = q_b[3];

  q_out[0] = aw*bw - ax*bx - ay*by - az*bz;  // w
  q_out[1] = aw*bx + ax*bw + ay*bz - az*by;  // x
  q_out[2] = aw*by - ax*bz + ay*bw + az*bx;  // y
  q_out[3] = aw*bz + ax*by - ay*bx + az*bw;  // z
}

void quatNormalize(eskf_scalar q[4]) {
  eskf_scalar norm = quatNorm(q);
  if (norm > static_cast<eskf_scalar>(1e-10)) {
    eskf_scalar inv_norm = static_cast<eskf_scalar>(1) / norm;
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
  } else {
    // Degenerate case: reset to identity
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  }
}

void quatToDcm(eskf_scalar R[3][3], const eskf_scalar q[4]) {
  const eskf_scalar w = q[0], x = q[1], y = q[2], z = q[3];

  // Precompute products
  const eskf_scalar xx = x*x, yy = y*y, zz = z*z;
  const eskf_scalar xy = x*y, xz = x*z, yz = y*z;
  const eskf_scalar wx = w*x, wy = w*y, wz = w*z;

  // DCM from quaternion (body→NED)
  R[0][0] = 1 - 2*(yy + zz);  R[0][1] = 2*(xy - wz);      R[0][2] = 2*(xz + wy);
  R[1][0] = 2*(xy + wz);      R[1][1] = 1 - 2*(xx + zz);  R[1][2] = 2*(yz - wx);
  R[2][0] = 2*(xz - wy);      R[2][1] = 2*(yz + wx);      R[2][2] = 1 - 2*(xx + yy);
}

void quatFromAxisAngle(eskf_scalar q[4],
                       const eskf_scalar axis[3],
                       eskf_scalar angle) {
  eskf_scalar half_angle = angle * static_cast<eskf_scalar>(0.5);
  eskf_scalar s = std::sin(half_angle);
  eskf_scalar c = std::cos(half_angle);

  q[0] = c;
  q[1] = axis[0] * s;
  q[2] = axis[1] * s;
  q[3] = axis[2] * s;

  // Ensure unit norm
  quatNormalize(q);
}

void quatFromRotationVector(eskf_scalar q[4], const eskf_scalar dtheta[3]) {
  // Compute rotation angle (magnitude of dtheta)
  eskf_scalar angle_sq = dtheta[0]*dtheta[0] + dtheta[1]*dtheta[1] + dtheta[2]*dtheta[2];

  if (angle_sq < static_cast<eskf_scalar>(1e-12)) {
    // Very small rotation: first-order approximation
    // q ≈ [1, δθ/2]
    q[0] = 1;
    q[1] = dtheta[0] * static_cast<eskf_scalar>(0.5);
    q[2] = dtheta[1] * static_cast<eskf_scalar>(0.5);
    q[3] = dtheta[2] * static_cast<eskf_scalar>(0.5);
  } else {
    // Full computation
    eskf_scalar angle = std::sqrt(angle_sq);
    eskf_scalar half_angle = angle * static_cast<eskf_scalar>(0.5);
    eskf_scalar s = std::sin(half_angle) / angle;
    eskf_scalar c = std::cos(half_angle);

    q[0] = c;
    q[1] = dtheta[0] * s;
    q[2] = dtheta[1] * s;
    q[3] = dtheta[2] * s;
  }

  quatNormalize(q);
}

void quatRotateVector(eskf_scalar v_out[3],
                      const eskf_scalar q[4],
                      const eskf_scalar v_in[3]) {
  // v' = q ⊗ v ⊗ q*
  // Optimized form (avoids full quaternion multiply):
  // v' = v + 2*w*(q_v × v) + 2*(q_v × (q_v × v))
  // where q = [w, q_v]

  const eskf_scalar w = q[0];
  const eskf_scalar qx = q[1], qy = q[2], qz = q[3];

  // t = 2 * (q_v × v)
  eskf_scalar tx = 2 * (qy*v_in[2] - qz*v_in[1]);
  eskf_scalar ty = 2 * (qz*v_in[0] - qx*v_in[2]);
  eskf_scalar tz = 2 * (qx*v_in[1] - qy*v_in[0]);

  // v' = v + w*t + q_v × t
  v_out[0] = v_in[0] + w*tx + (qy*tz - qz*ty);
  v_out[1] = v_in[1] + w*ty + (qz*tx - qx*tz);
  v_out[2] = v_in[2] + w*tz + (qx*ty - qy*tx);
}

void quatRotateVectorInverse(eskf_scalar v_out[3],
                             const eskf_scalar q[4],
                             const eskf_scalar v_in[3]) {
  // v' = q* ⊗ v ⊗ q (inverse rotation)
  // Using conjugate: q* = [w, -x, -y, -z]
  eskf_scalar q_conj[4];
  quatConjugate(q_conj, q);
  quatRotateVector(v_out, q_conj, v_in);
}

void quatToRollPitchYaw(eskf_scalar& roll, eskf_scalar& pitch, eskf_scalar& yaw,
                        const eskf_scalar q[4]) {
  // ZYX (Yaw-Pitch-Roll) Euler angle extraction from quaternion
  const eskf_scalar w = q[0], x = q[1], y = q[2], z = q[3];

  // Roll (rotation about X-axis)
  eskf_scalar sinr_cosp = 2 * (w * x + y * z);
  eskf_scalar cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (rotation about Y-axis)
  eskf_scalar sinp = 2 * (w * y - z * x);
  // Handle gimbal lock
  if (std::abs(sinp) >= 1) {
    pitch = std::copysign(constants::kPi / 2, sinp);  // Clamp to ±90°
  } else {
    pitch = std::asin(sinp);
  }

  // Yaw (rotation about Z-axis)
  eskf_scalar siny_cosp = 2 * (w * z + x * y);
  eskf_scalar cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

eskf_scalar calculateTiltCompensatedHeading(const eskf_scalar mag_body[3],
                                             const eskf_scalar q[4],
                                             eskf_scalar declination_rad) {
  // Robust Vector-Based Heading Calculation (Gimbal-Lock Free)
  //
  // Instead of extracting Roll/Pitch Euler angles (which fail at 90° pitch),
  // we rotate the body-frame magnetic vector directly to the Earth frame
  // using the quaternion, then project onto the horizontal plane.
  //
  // This works at any orientation, including vertical rockets on the rail.

  // 1. Rotate body magnetic vector to estimated NED frame
  //    The quaternion q represents body→NED rotation
  eskf_scalar mag_earth[3];
  quatRotateVector(mag_earth, q, mag_body);

  // 2. Calculate heading from horizontal projection
  //    In NED: X=North, Y=East, Z=Down
  //    atan2(East, North) gives heading CW from North
  //    Note: No need to explicitly remove Z component - atan2 ignores it
  eskf_scalar heading = std::atan2(mag_earth[1], mag_earth[0]);

  // 3. Apply magnetic declination
  //    True North = Magnetic North + Declination (East positive)
  heading += declination_rad;

  // 4. Wrap to [-π, π]
  return wrapPi(heading);
}


// ============================================================
// CUSTOM BACKEND IMPLEMENTATIONS (Only when needed)
// ============================================================

#if ESKF_USE_CUSTOM_LINALG

void josephUpdate(Matrix15& P, const Vector15& K,
                  const eskf_scalar H[kDimError], eskf_scalar R) {
  // I_KH = I - K*H
  Matrix15 I_KH = Matrix15::Identity();
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      I_KH.data[i][j] -= K.data[i][0] * H[j];
    }
  }

  // I_KH_t = (I - K*H)'
  Matrix15 I_KH_t;
  transpose(I_KH_t, I_KH);

  // temp1 = (I - K*H) * P
  Matrix15 temp1;
  multiply(temp1, I_KH, P);

  // temp2 = (I - K*H) * P * (I - K*H)'
  Matrix15 temp2;
  multiply(temp2, temp1, I_KH_t);

  // Add K*R*K'
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      P.data[i][j] = temp2.data[i][j] + K.data[i][0] * R * K.data[j][0];
    }
  }
}

eskf_scalar scalarUpdate(Vector15& dx, Matrix15& P,
                         eskf_scalar z, eskf_scalar h,
                         const eskf_scalar H[kDimError], eskf_scalar R) {
  // S = H*P*H' + R (scalar)
  eskf_scalar S = R;
  for (int i = 0; i < kDimError; ++i) {
    eskf_scalar temp = 0;
    for (int j = 0; j < kDimError; ++j) {
      temp += P.data[i][j] * H[j];
    }
    S += H[i] * temp;
  }

  // Avoid division by zero
  if (S < static_cast<eskf_scalar>(1e-12)) {
    S = static_cast<eskf_scalar>(1e-12);
  }

  // K = P*H'/S (15×1)
  Vector15 K;
  for (int i = 0; i < kDimError; ++i) {
    eskf_scalar sum = 0;
    for (int j = 0; j < kDimError; ++j) {
      sum += P.data[i][j] * H[j];
    }
    K.data[i][0] = sum / S;
  }

  // Innovation
  eskf_scalar y = z - h;

  // Update error state
  for (int i = 0; i < kDimError; ++i) {
    dx.data[i][0] += K.data[i][0] * y;
  }

  // Joseph update
  josephUpdate(P, K, H, R);

  // Return NIS
  return y * y / S;
}

#endif // ESKF_USE_CUSTOM_LINALG

} // namespace math
} // namespace eskf
