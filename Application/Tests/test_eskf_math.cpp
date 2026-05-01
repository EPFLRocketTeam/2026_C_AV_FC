// ESKF Math Unit Tests
// Tests quaternion operations and linear algebra functions

#include "Application/Tests/unity_gtest_compat.hpp"

#include <cmath>

#include "Application/Kalman/kalman/eskf_config.hpp"
#include "Application/Kalman/kalman/eskf_math.hpp"

using namespace eskf;
using namespace eskf::math;

// Tolerance for floating-point comparisons
constexpr eskf_scalar kTol = 1e-9;
constexpr eskf_scalar kTolF = 1e-5;  // Looser for intermediate results

// ============================================================
// Quaternion Tests
// ============================================================

static void test_quatNormalize_identity() {
  eskf_scalar q[4] = {1, 0, 0, 0};
  quatNormalize(q);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, q[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, q[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, q[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.0, q[3]);
}

static void test_quatNormalize_unit() {
  eskf_scalar q[4] = {2, 0, 0, 0};
  quatNormalize(q);
  eskf_scalar norm = quatNorm(q);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);
}

static void test_quatNormalize_arbitrary() {
  eskf_scalar q[4] = {1, 2, 3, 4};
  quatNormalize(q);
  eskf_scalar norm = quatNorm(q);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);
}

static void test_quatMultiply_identity_left() {
  eskf_scalar q_id[4] = {1, 0, 0, 0};
  eskf_scalar q_in[4] = {0.5, 0.5, 0.5, 0.5};
  eskf_scalar q_out[4];

  quatMultiply(q_out, q_id, q_in);

  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[0], q_out[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[1], q_out[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[2], q_out[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[3], q_out[3]);
}

static void test_quatMultiply_identity_right() {
  eskf_scalar q_id[4] = {1, 0, 0, 0};
  eskf_scalar q_in[4] = {0.5, 0.5, 0.5, 0.5};
  eskf_scalar q_out[4];

  quatMultiply(q_out, q_in, q_id);

  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[0], q_out[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[1], q_out[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[2], q_out[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, q_in[3], q_out[3]);
}

static void test_quatMultiply_90deg_x() {
  // 90-degree rotation about X: q = [cos(45°), sin(45°), 0, 0]
  eskf_scalar c = std::cos(M_PI / 4);
  eskf_scalar s = std::sin(M_PI / 4);
  eskf_scalar q_x90[4] = {c, s, 0, 0};
  eskf_scalar q_out[4];

  // Two 90° rotations should give 180° rotation
  quatMultiply(q_out, q_x90, q_x90);
  quatNormalize(q_out);

  // 180° about X: q = [0, 1, 0, 0]
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, std::abs(q_out[0]));
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 1.0, std::abs(q_out[1]));
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, std::abs(q_out[2]));
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, std::abs(q_out[3]));
}

static void test_quatRotateVector_identity() {
  eskf_scalar q_id[4] = {1, 0, 0, 0};
  eskf_scalar v_in[3] = {1, 2, 3};
  eskf_scalar v_out[3];

  quatRotateVector(v_out, q_id, v_in);

  TEST_ASSERT_DOUBLE_WITHIN(kTol, v_in[0], v_out[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, v_in[1], v_out[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, v_in[2], v_out[2]);
}

static void test_quatRotateVector_90deg_z() {
  // 90-degree rotation about Z: q = [cos(45°), 0, 0, sin(45°)]
  eskf_scalar c = std::cos(M_PI / 4);
  eskf_scalar s = std::sin(M_PI / 4);
  eskf_scalar q_z90[4] = {c, 0, 0, s};
  eskf_scalar v_in[3] = {1, 0, 0};  // X-axis unit vector
  eskf_scalar v_out[3];

  quatRotateVector(v_out, q_z90, v_in);

  // X-axis rotated 90° about Z should become Y-axis
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, v_out[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 1.0, v_out[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, v_out[2]);
}

static void test_quatRotateVector_inverse() {
  // Rotating and inverse rotating should return original
  eskf_scalar q[4] = {0.5, 0.5, 0.5, 0.5};
  quatNormalize(q);

  eskf_scalar v_in[3] = {1, 2, 3};
  eskf_scalar v_rot[3];
  eskf_scalar v_back[3];

  quatRotateVector(v_rot, q, v_in);
  quatRotateVectorInverse(v_back, q, v_rot);

  TEST_ASSERT_DOUBLE_WITHIN(kTolF, v_in[0], v_back[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, v_in[1], v_back[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, v_in[2], v_back[2]);
}

static void test_quatFromRotationVector_small() {
  // Very small rotation
  eskf_scalar dtheta[3] = {0.001, 0.002, 0.003};
  eskf_scalar q[4];

  quatFromRotationVector(q, dtheta);

  // Should be approximately [1, dtheta/2]
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 1.0, q[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0005, q[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.001, q[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0015, q[3]);

  // Norm should be 1
  eskf_scalar norm = quatNorm(q);
  TEST_ASSERT_DOUBLE_WITHIN(kTol, 1.0, norm);
}

static void test_quatFromRotationVector_90deg() {
  // 90-degree rotation about Z
  eskf_scalar dtheta[3] = {0, 0, M_PI / 2};
  eskf_scalar q[4];

  quatFromRotationVector(q, dtheta);
  quatNormalize(q);

  // Should be [cos(45°), 0, 0, sin(45°)]
  eskf_scalar c = std::cos(M_PI / 4);
  eskf_scalar s = std::sin(M_PI / 4);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, c, q[0]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, q[1]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, 0.0, q[2]);
  TEST_ASSERT_DOUBLE_WITHIN(kTolF, s, q[3]);
}

static void test_quatToDcm_identity() {
  eskf_scalar q_id[4] = {1, 0, 0, 0};
  eskf_scalar R[3][3];

  quatToDcm(R, q_id);

  // Should be identity matrix
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      eskf_scalar expected = (i == j) ? 1.0 : 0.0;
      TEST_ASSERT_DOUBLE_WITHIN(kTol, expected, R[i][j]);
    }
  }
}

static void test_quatToDcm_orthogonal() {
  // Arbitrary quaternion
  eskf_scalar q[4] = {0.5, 0.5, 0.5, 0.5};
  quatNormalize(q);
  eskf_scalar R[3][3];

  quatToDcm(R, q);

  // Check orthogonality: R * R' = I
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      eskf_scalar sum = 0;
      for (int k = 0; k < 3; ++k) {
        sum += R[i][k] * R[j][k];  // R * R'
      }
      eskf_scalar expected = (i == j) ? 1.0 : 0.0;
      TEST_ASSERT_DOUBLE_WITHIN(kTolF, expected, sum);
    }
  }
}

// ============================================================
// Gravity Model Tests
// ============================================================

static void test_gravityMagnitude_sealevel() {
  eskf_scalar g = gravityMagnitude(0);
  TEST_ASSERT_DOUBLE_WITHIN(0.001, 9.80665, g);
}

static void test_gravityMagnitude_10km() {
  // At 10km, gravity should be slightly less
  eskf_scalar g = gravityMagnitude(10000);
  TEST_ASSERT_TRUE(g < 9.80665);
  TEST_ASSERT_TRUE(g > 9.75);  // Reasonable range
}

// ============================================================
// Joseph Update Tests
// ============================================================

#if !ESKF_USE_CUSTOM_LINALG

static void test_josephUpdate_preserves_symmetry() {
  Matrix15 P = Matrix15::Identity() * 10;
  Vector15 K = Vector15::Zero();
  K(0) = 0.5;
  K(3) = 0.3;
  RowVector15 H = RowVector15::Zero();
  H(0, 0) = 1;
  eskf_scalar R = 1.0;

  josephUpdate(P, K, H, R);

  // Check symmetry: P(i,j) == P(j,i)
  for (int i = 0; i < kDimError; ++i) {
    for (int j = i + 1; j < kDimError; ++j) {
      TEST_ASSERT_DOUBLE_WITHIN(kTol, P(i, j), P(j, i));
    }
  }
}

static void test_josephUpdate_positive_definite() {
  Matrix15 P = Matrix15::Identity() * 10;
  Vector15 K = Vector15::Zero();
  K(0) = 0.9;  // Large gain
  RowVector15 H = RowVector15::Zero();
  H(0, 0) = 1;
  eskf_scalar R = 0.1;

  josephUpdate(P, K, H, R);

  // Check diagonal elements are positive
  for (int i = 0; i < kDimError; ++i) {
    TEST_ASSERT_TRUE(P(i, i) > 0);
  }
}

static void test_scalarUpdate_reduces_uncertainty() {
  Matrix15 P = Matrix15::Identity() * 10;
  Vector15 dx = Vector15::Zero();
  RowVector15 H = RowVector15::Zero();
  H(0, 0) = 1;

  eskf_scalar initial_variance = P(0, 0);
  eskf_scalar z = 5.0;
  eskf_scalar h = 4.5;
  eskf_scalar R = 1.0;

  eskf_scalar nis = scalarUpdate(dx, P, z, h, H, R);

  // Variance should decrease after update
  TEST_ASSERT_TRUE(P(0, 0) < initial_variance);

  // NIS should be positive
  TEST_ASSERT_TRUE(nis > 0);
}

#endif // !ESKF_USE_CUSTOM_LINALG

// ============================================================
// GTest Wrapper
// ============================================================

#define WRAP_TEST(test_fn) TEST(KalmanEskfMathSuite, test_fn) { test_fn(); }

WRAP_TEST(test_quatNormalize_identity);
WRAP_TEST(test_quatNormalize_unit);
WRAP_TEST(test_quatNormalize_arbitrary);
WRAP_TEST(test_quatMultiply_identity_left);
WRAP_TEST(test_quatMultiply_identity_right);
WRAP_TEST(test_quatMultiply_90deg_x);
WRAP_TEST(test_quatRotateVector_identity);
WRAP_TEST(test_quatRotateVector_90deg_z);
WRAP_TEST(test_quatRotateVector_inverse);
WRAP_TEST(test_quatFromRotationVector_small);
WRAP_TEST(test_quatFromRotationVector_90deg);
WRAP_TEST(test_quatToDcm_identity);
WRAP_TEST(test_quatToDcm_orthogonal);

WRAP_TEST(test_gravityMagnitude_sealevel);
WRAP_TEST(test_gravityMagnitude_10km);

#if !ESKF_USE_CUSTOM_LINALG
WRAP_TEST(test_josephUpdate_preserves_symmetry);
WRAP_TEST(test_josephUpdate_positive_definite);
WRAP_TEST(test_scalarUpdate_reduces_uncertainty);
#endif

#undef WRAP_TEST
