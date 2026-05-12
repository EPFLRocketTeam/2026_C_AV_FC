#pragma once
// ESKF Buffer Sizes and Structural Constants
// Split from eskf_config.hpp for organization

#ifndef ESKF_IMU_PRIMARY_ODR_HZ
#define ESKF_IMU_PRIMARY_ODR_HZ 6400
#endif

#ifndef ESKF_BARO_ODR_HZ
#define ESKF_BARO_ODR_HZ 100
#endif

// ============================================================
// Buffer Sizes (500ms history at target rates)
// ============================================================

#ifndef ESKF_GPS_REWIND_DEPTH_MS
#define ESKF_GPS_REWIND_DEPTH_MS 500
#endif

#ifndef ESKF_IMU_BUFFER_SIZE
#define ESKF_IMU_BUFFER_SIZE                                                  \
  ((ESKF_GPS_REWIND_DEPTH_MS * ESKF_IMU_PRIMARY_ODR_HZ + 999) / 1000)
#endif

#ifndef ESKF_BARO_BUFFER_SIZE
#define ESKF_BARO_BUFFER_SIZE                                                 \
  ((ESKF_GPS_REWIND_DEPTH_MS * ESKF_BARO_ODR_HZ + 999) / 1000)
#endif

#ifndef ESKF_EVENT_BUFFER_SIZE
#define ESKF_EVENT_BUFFER_SIZE 256   // Extra headroom for rewind replay bursts
#endif

#ifndef ESKF_CHECKPOINT_INTERVAL
#define ESKF_CHECKPOINT_INTERVAL 128    // Every 128 IMU samples @ 6.4kHz = 20ms
#endif

// --- Periodic Checkpoints for Efficient Rewind ---
#ifndef ESKF_CHECKPOINT_BUFFER_SIZE
#define ESKF_CHECKPOINT_BUFFER_SIZE                                           \
  ((ESKF_GPS_REWIND_DEPTH_MS * ESKF_IMU_PRIMARY_ODR_HZ +                 \
    (ESKF_CHECKPOINT_INTERVAL * 1000 - 1)) /                            \
   (ESKF_CHECKPOINT_INTERVAL * 1000))
#endif

#ifndef ESKF_RAIL_CHECKPOINT_BUFFER_SIZE
#define ESKF_RAIL_CHECKPOINT_BUFFER_SIZE 50
#endif

// ============================================================
// Multi-sensor support
// ============================================================

#ifndef ESKF_MAX_IMUS
#define ESKF_MAX_IMUS 4              // Maximum simultaneous IMU sensors
#endif

#ifndef ESKF_MAX_BAROS
#define ESKF_MAX_BAROS 4             // Maximum simultaneous barometer sensors
#endif

// ============================================================
// Precision Configuration
// ============================================================

#ifndef ESKF_FORCE_FLOAT32
#define ESKF_FORCE_FLOAT32 0
#endif

#if ESKF_FORCE_FLOAT32
  using eskf_scalar = float;
#else
  using eskf_scalar = double;
#endif

// Always use float for sensor I/O and logging
using eskf_sensor_t = float;

// ============================================================
// Linear Algebra Backend
// ============================================================

#ifndef ESKF_USE_CUSTOM_LINALG
#define ESKF_USE_CUSTOM_LINALG 1
#endif

// ============================================================
// Dimensions and Index Offsets
// ============================================================

namespace eskf {

/// Nominal state dimension: p(3), v(3), q(4), b_a(3), b_g(3), b_h(1)
[[maybe_unused]] constexpr int kDimState = 17;

/// Error state dimension: δp(3), δv(3), δθ(3), δb_a(3), δb_g(3), δb_h(1)
constexpr int kDimError = 16; // we use euler angles for error state instead of quaternions

// Index offsets for error state vector
namespace idx {
  constexpr int kPos = 0;      // δp: position error (3)
  constexpr int kVel = 3;      // δv: velocity error (3)
  constexpr int kAtt = 6;      // δθ: attitude error (3)
  constexpr int kAccBias = 9;  // δb_a: accelerometer bias error (3)
  constexpr int kGyrBias = 12; // δb_g: gyroscope bias error (3)
  constexpr int kBarBias = 15; // δb_h: barometer bias error (1)
}

// Physical constants
namespace constants {
  constexpr eskf_scalar kGravity0 = 9.80665;      // Standard gravity (m/s²)
  constexpr eskf_scalar kEarthRadius = 6371000.0; // Earth radius (m)
  constexpr eskf_scalar kPi = 3.14159265358979323846;

  // WGS84 parameters for Somigliana gravity model (Section 1.3)
  constexpr eskf_scalar kGravityEquator = 9.7803253359;  // Equatorial gravity (m/s²)
  constexpr eskf_scalar kGravityPole = 9.8321849378;     // Polar gravity (m/s²)
  constexpr eskf_scalar kFlattening = 1.0 / 298.257223563; // WGS84 flattening

  // Constexpr sin approximation using Taylor series (accurate to ~1e-6 for |x| < π)
  constexpr eskf_scalar constexprSin(eskf_scalar x) {
    // Reduce to [-π, π] not needed for typical latitude values
    const eskf_scalar x2 = x * x;
    const eskf_scalar x3 = x2 * x;
    const eskf_scalar x5 = x3 * x2;
    const eskf_scalar x7 = x5 * x2;
    const eskf_scalar x9 = x7 * x2;
    // Taylor: sin(x) ≈ x - x³/6 + x⁵/120 - x⁷/5040 + x⁹/362880
    return x - x3 / 6.0 + x5 / 120.0 - x7 / 5040.0 + x9 / 362880.0;
  }

  // Compute local gravity from latitude (Somigliana formula)
  // g_local = g_eq * (1 + 0.0053024*sin²(φ) - 0.0000058*sin²(2φ))
  constexpr eskf_scalar computeLocalGravity(eskf_scalar latitude_deg) {
    const eskf_scalar phi = latitude_deg * kPi / 180.0;
    const eskf_scalar sin_phi = constexprSin(phi);
    const eskf_scalar sin_2phi = constexprSin(2.0 * phi);
    const eskf_scalar sin2_phi = sin_phi * sin_phi;
    const eskf_scalar sin2_2phi = sin_2phi * sin_2phi;
    return kGravityEquator * (1.0 + 0.0053024 * sin2_phi - 0.0000058 * sin2_2phi);
  }

  // Local gravity requires ESKF_LAUNCH_SITE_LATITUDE_DEG from tuning header
  // This is defined in eskf_config.hpp after including both headers
}

} // namespace eskf
