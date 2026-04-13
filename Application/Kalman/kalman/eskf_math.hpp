#pragma once
// ESKF Linear Algebra Abstraction Layer
// Part of Phase 1: Standalone Core ESKF Library
//
// Provides matrix/vector operations with two backends:
// - Eigen (default): Full-featured, optimized, recommended for Teensy 4.1
// - Custom (ESKF_USE_CUSTOM_LINALG=1): Minimal, for constrained MCUs
//
// Quaternion operations are always custom (simple enough, avoid Eigen dependency).

#include "eskf_config.hpp"
#include <cmath>

#if !ESKF_USE_CUSTOM_LINALG
// ============================================================
// EIGEN BACKEND (Default)
// ============================================================

// Platform-specific Eigen includes:
// - Teensy: Use ArduinoEigen wrapper (handles Arduino macro conflicts)
// - Native: Direct include from ArduinoEigen's bundled Eigen headers
#if defined(HAL_PLATFORM_TEENSY) || defined(ARDUINO)
  #include <ArduinoEigenDense.h>
#else
  // Native build: directly include Eigen from ArduinoEigen library
  // Define EIGEN_MPL2_ONLY for license compliance
  #define EIGEN_MPL2_ONLY
  #include <ArduinoEigen/Eigen/Dense>
#endif

namespace eskf {
namespace math {

// Type aliases for common matrix/vector sizes
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<eskf_scalar, Rows, Cols>;

template<int Size>
using Vector = Eigen::Matrix<eskf_scalar, Size, 1>;

// Dimension-agnostic aliases for error-state matrices/vectors
using MatrixError = Matrix<kDimError, kDimError>;
using VectorError = Vector<kDimError>;
using RowVectorError = Eigen::Matrix<eskf_scalar, 1, kDimError>;
using Matrix3 = Matrix<3, 3>;
using Vector3 = Vector<3>;

// Legacy aliases for compatibility (deprecated)
using Matrix15 = MatrixError;
using Vector15 = VectorError;
using RowVector15 = RowVectorError;

/// Joseph-form covariance update (numerically stable).
/// P = (I - K*H) * P * (I - K*H)' + K*R*K'
/// Includes symmetry enforcement to prevent floating-point asymmetry accumulation.
inline void josephUpdate(MatrixError& P, const VectorError& K,
                         const RowVectorError& H, eskf_scalar R) {
  MatrixError I_KH = MatrixError::Identity() - K * H;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
  
  // Enforce symmetry (small floating-point errors can accumulate)
  P = static_cast<eskf_scalar>(0.5) * (P + P.transpose());
}

/// Scalar measurement update with Joseph form.
/// Updates error state dx and covariance P.
/// Returns NIS (Normalized Innovation Squared) contribution.
/// @param dx Error state accumulator (kDimError×1)
/// @param P Covariance matrix (kDimError×kDimError)
/// @param z Measurement value
/// @param h Predicted measurement (from current state)
/// @param H Measurement Jacobian row (1×kDimError)
/// @param R Measurement variance (scalar)
/// @return NIS = y²/S where y = z - h, S = H*P*H' + R
inline eskf_scalar scalarUpdate(VectorError& dx, MatrixError& P,
                                eskf_scalar z, eskf_scalar h,
                                const RowVectorError& H, eskf_scalar R) {
  // 1. Calculate Innovation Covariance
  // S = Uncertainty of State Projected + Uncertainty of Sensor
  eskf_scalar P_projected = (H * P * H.transpose())(0, 0);
  eskf_scalar S = P_projected + R;

  // 2. Apply Numerical Safety Floor
  // We use the larger of:
  // a) The sensor noise floor R (S mathematically cannot be < R)
  // b) A numerical epsilon (1e-8) to prevent division by near-zero if R is 0
  constexpr eskf_scalar MIN_S_EPSILON = 1e-8; // Safe for double precision
  
  if (S < R) S = R; 
  if (S < MIN_S_EPSILON) S = MIN_S_EPSILON;

  // 3. Compute Kalman Gain
  VectorError K = P * H.transpose() / S;

  // 4. Compute Innovation
  eskf_scalar y = z - h;

  // 5. Update State
  dx += K * y;

  // 6. Update Covariance (Joseph Form)
  josephUpdate(P, K, H, R);

  // Return NIS
  return y * y / S;
}

} // namespace math
} // namespace eskf

#else
// ============================================================
// CUSTOM BACKEND (For constrained MCUs)
// ============================================================

namespace eskf {
namespace math {

/// Simple fixed-size matrix (no dynamic allocation).
template<int Rows, int Cols>
struct Matrix {
  eskf_scalar data[Rows][Cols] = {};

  eskf_scalar& operator()(int r, int c) { return data[r][c]; }
  eskf_scalar operator()(int r, int c) const { return data[r][c]; }

  // Vector access (assuming column vector)
  eskf_scalar& operator()(int i) { return data[i][0]; }
  eskf_scalar operator()(int i) const { return data[i][0]; }

  void setIdentity() {
      *this = Identity();
  }

  void setZero() {
      *this = Zero();
  }

  void setDiagonal(const eskf_scalar* diag) {
      *this = Zero();
      int minDim = (Rows < Cols) ? Rows : Cols;
      for (int i = 0; i < minDim; ++i) {
          data[i][i] = diag[i];
      }
  }

  static Matrix Identity() {
    Matrix m;
    int minDim = (Rows < Cols) ? Rows : Cols;
    for (int i = 0; i < minDim; ++i) {
      m.data[i][i] = static_cast<eskf_scalar>(1);
    }
    return m;
  }

  static Matrix Zero() {
    Matrix m;
    return m; // Already zero-initialized
  }

  Matrix& operator+=(const Matrix& other) {
    for (int i = 0; i < Rows; ++i) {
      for (int j = 0; j < Cols; ++j) {
        data[i][j] += other.data[i][j];
      }
    }
    return *this;
  }

  Matrix& operator-=(const Matrix& other) {
    for (int i = 0; i < Rows; ++i) {
      for (int j = 0; j < Cols; ++j) {
        data[i][j] -= other.data[i][j];
      }
    }
    return *this;
  }

  Matrix& operator*=(eskf_scalar s) {
    for (int i = 0; i < Rows; ++i) {
      for (int j = 0; j < Cols; ++j) {
        data[i][j] *= s;
      }
    }
    return *this;
  }

  // Friend operators for convenient math
  friend Matrix operator+(Matrix lhs, const Matrix& rhs) {
      lhs += rhs;
      return lhs;
  }

  friend Matrix operator*(eskf_scalar s, Matrix rhs) {
      rhs *= s;
      return rhs;
  }

  friend Matrix operator*(Matrix lhs, eskf_scalar s) {
      lhs *= s;
      return lhs;
  }
  
  // Cross product (only valid for 3x1 vectors)
  Matrix cross(const Matrix& other) const {
      static_assert(Rows == 3 && Cols == 1, "cross product requires 3x1 vectors");
      Matrix res;
      // res = this x other
      // [y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2]
      res(0) = data[1][0] * other.data[2][0] - data[2][0] * other.data[1][0];
      res(1) = data[2][0] * other.data[0][0] - data[0][0] * other.data[2][0];
      res(2) = data[0][0] * other.data[1][0] - data[1][0] * other.data[0][0];
      return res;
  }
};

template<int Size>
using Vector = Matrix<Size, 1>;

// Dimension-agnostic aliases for error-state matrices/vectors
using MatrixError = Matrix<kDimError, kDimError>;
using VectorError = Vector<kDimError>;
using Matrix3 = Matrix<3, 3>;
using Vector3 = Vector<3>;

// Legacy aliases for compatibility (deprecated)
using Matrix15 = MatrixError;
using Vector15 = VectorError;

/// Matrix multiply: C = A * B
template<int M, int N, int P>
void multiply(Matrix<M, P>& C, const Matrix<M, N>& A, const Matrix<N, P>& B) {
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < P; ++j) {
      eskf_scalar sum = 0;
      for (int k = 0; k < N; ++k) {
        sum += A.data[i][k] * B.data[k][j];
      }
      C.data[i][j] = sum;
    }
  }
}

/// Transpose: At = A'
template<int M, int N>
void transpose(Matrix<N, M>& At, const Matrix<M, N>& A) {
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      At.data[j][i] = A.data[i][j];
    }
  }
}

/// Joseph-form covariance update (custom backend).
/// P = (I - K*H) * P * (I - K*H)' + K*R*K'
/// H is a row vector stored as 1D array.
void josephUpdate(Matrix15& P, const Vector15& K,
                  const eskf_scalar H[kDimError], eskf_scalar R);

/// Scalar measurement update (custom backend).
/// Returns NIS contribution.
eskf_scalar scalarUpdate(Vector15& dx, Matrix15& P,
                         eskf_scalar z, eskf_scalar h,
                         const eskf_scalar H[kDimError], eskf_scalar R);

} // namespace math
} // namespace eskf

#endif // ESKF_USE_CUSTOM_LINALG

// ============================================================
// QUATERNION OPERATIONS (Always custom implementation)
// ============================================================

namespace eskf {
namespace math {

/// Quaternion multiply: q_out = q_a ⊗ q_b (Hamilton convention)
/// Quaternion format: [w, x, y, z] where w is scalar part.
void quatMultiply(eskf_scalar q_out[4],
                  const eskf_scalar q_a[4],
                  const eskf_scalar q_b[4]);

/// Normalize quaternion in-place to unit norm.
void quatNormalize(eskf_scalar q[4]);

/// Quaternion to Direction Cosine Matrix (rotation matrix).
/// R transforms vectors from body frame to navigation frame.
void quatToDcm(eskf_scalar R[3][3], const eskf_scalar q[4]);

/// Create quaternion from axis-angle representation.
/// axis: unit vector, angle: rotation angle in radians.
void quatFromAxisAngle(eskf_scalar q[4],
                       const eskf_scalar axis[3],
                       eskf_scalar angle);

/// Create quaternion from small-angle rotation vector.
/// For small δθ: q ≈ [1, δθ/2] (first-order approximation).
void quatFromRotationVector(eskf_scalar q[4], const eskf_scalar dtheta[3]);

/// Rotate vector by quaternion: v_out = q ⊗ v ⊗ q*
/// Rotates v_in from body frame to navigation frame using quaternion q.
void quatRotateVector(eskf_scalar v_out[3],
                      const eskf_scalar q[4],
                      const eskf_scalar v_in[3]);

/// Rotate vector by quaternion conjugate (inverse rotation).
/// Rotates v_in from navigation frame to body frame.
void quatRotateVectorInverse(eskf_scalar v_out[3],
                             const eskf_scalar q[4],
                             const eskf_scalar v_in[3]);

/// Get quaternion conjugate: q_conj = [w, -x, -y, -z]
inline void quatConjugate(eskf_scalar q_conj[4], const eskf_scalar q[4]) {
  q_conj[0] = q[0];
  q_conj[1] = -q[1];
  q_conj[2] = -q[2];
  q_conj[3] = -q[3];
}

/// Compute quaternion norm squared.
inline eskf_scalar quatNormSquared(const eskf_scalar q[4]) {
  return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
}

/// Compute quaternion norm.
inline eskf_scalar quatNorm(const eskf_scalar q[4]) {
  return std::sqrt(quatNormSquared(q));
}

/// Check if value is finite (not NaN or Inf).
inline bool isFinite(eskf_scalar x) {
  return std::isfinite(x);
}

/// Check if all elements of a quaternion are finite.
inline bool quatIsFinite(const eskf_scalar q[4]) {
  return isFinite(q[0]) && isFinite(q[1]) && isFinite(q[2]) && isFinite(q[3]);
}

/// Altitude-dependent gravity magnitude (simple inverse-square model).
/// g(z) = g_local * (Rₑ / (Rₑ + z))²
/// @param altitude_m Altitude above sea level in meters
/// @param g_local Local gravity at sea level (default: standard 9.80665)
/// @return Gravity magnitude (m/s²)
inline eskf_scalar gravityMagnitude(eskf_scalar altitude_m, eskf_scalar g_local = constants::kGravityLocal) {
  eskf_scalar ratio = constants::kEarthRadius / (constants::kEarthRadius + altitude_m);
  return g_local * ratio * ratio;
}

// WGS-84 Ellipsoid Constants for Somigliana Gravity
static constexpr double WGS84_GRAVITY_EQUATOR = 9.7803253359;       // gamma_e (m/s²)
static constexpr double WGS84_GRAVITY_CONSTANT = 0.00193185265241;  // k
static constexpr double WGS84_ECCENTRICITY_SQ = 0.00669437999014;   // e²

/**
 * @brief Computes Exact Somigliana Gravity for WGS84 Ellipsoid
 * 
 * The Somigliana formula provides the normal gravity on the surface of
 * the WGS84 reference ellipsoid as a function of geodetic latitude.
 * This is the canonical single-source-of-truth for runtime gravity calculation.
 * 
 * @param latitude_rad Geodetic latitude in Radians
 * @return Gravity magnitude in m/s²
 */
static inline double compute_somigliana_gravity(double latitude_rad) {
    double sin_phi = std::sin(latitude_rad);
    double sin2_phi = sin_phi * sin_phi;

    double numerator = WGS84_GRAVITY_EQUATOR * (1.0 + WGS84_GRAVITY_CONSTANT * sin2_phi);
    double denominator = std::sqrt(1.0 - WGS84_ECCENTRICITY_SQ * sin2_phi);

    return numerator / denominator;
}

/// @deprecated Use compute_somigliana_gravity() instead
inline eskf_scalar localGravityFromLatitude(eskf_scalar latitude_rad) {
    return static_cast<eskf_scalar>(compute_somigliana_gravity(static_cast<double>(latitude_rad)));
}

/// Extract Roll, Pitch, Yaw (Euler angles) from quaternion.
/// Uses ZYX (Yaw-Pitch-Roll) convention, suitable for aerospace.
/// @param roll Output roll angle in radians (rotation about body X)
/// @param pitch Output pitch angle in radians (rotation about body Y)
/// @param yaw Output yaw angle in radians (rotation about body Z, heading)
/// @param q Input quaternion [w, x, y, z]
void quatToRollPitchYaw(eskf_scalar& roll, eskf_scalar& pitch, eskf_scalar& yaw,
                        const eskf_scalar q[4]);

/// Calculate tilt-compensated heading from magnetometer (Design Doc 5.4.F).
/// Uses current attitude to de-rotate the mag vector to the horizontal plane.
/// @param mag_body Calibrated magnetometer vector in body frame (μT)
/// @param q Current attitude quaternion [w, x, y, z]
/// @param declination_rad Magnetic declination correction (radians, East positive)
/// @return Heading in radians, clockwise from True North [0, 2π) or wrapped to [-π, π]
eskf_scalar calculateTiltCompensatedHeading(const eskf_scalar mag_body[3],
                                             const eskf_scalar q[4],
                                             eskf_scalar declination_rad);

/// Wrap angle to [-π, π] range.
inline eskf_scalar wrapPi(eskf_scalar angle) {
  while (angle > constants::kPi) angle -= 2 * constants::kPi;
  while (angle < -constants::kPi) angle += 2 * constants::kPi;
  return angle;
}

/// Skew-symmetric matrix from vector (for cross product).
/// [v]× such that [v]× * u = v × u
inline void skewSymmetric(eskf_scalar S[3][3], const eskf_scalar v[3]) {
  S[0][0] = 0;      S[0][1] = -v[2];  S[0][2] = v[1];
  S[1][0] = v[2];   S[1][1] = 0;      S[1][2] = -v[0];
  S[2][0] = -v[1];  S[2][1] = v[0];   S[2][2] = 0;
}

/// Apply 3x3 matrix to 3-vector: out = M * v
/// @param out Output vector (3 elements)
/// @param M 3x3 matrix (row-major)
/// @param v Input vector (3 elements)
inline void mat3Vec3Multiply(eskf_scalar out[3],
                             const eskf_scalar M[3][3],
                             const eskf_scalar v[3]) {
  for (int i = 0; i < 3; ++i) {
    out[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
  }
}

} // namespace math
} // namespace eskf
