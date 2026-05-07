#pragma once
// ============================================================================
// Kalman Debug Console Output
// ============================================================================
//
// Compile-time flag to enable verbose, human-readable Kalman filter
// diagnostics on the serial console.  Intended for first-on-board sanity
// tests where a teammate reads the print output to verify the filter is
// alive and producing reasonable values.
//
// Enable by defining KALMAN_DEBUG_PRINT=1 in the build system
// (e.g. -DKALMAN_DEBUG_PRINT=1 in the STM32CubeIDE preprocessor symbols).
//
// KALMAN_DEBUG_FORCE_FLIGHT:
//   When set to 1, the Kalman subsystem will auto-inject a synthetic
//   BURN state + liftoff event shortly after boot, so the ESKF
//   initialises and starts tracking attitude even without going through
//   the normal IGNITION→BURN flight sequence.  Use for indoor bench tests.
//
// ============================================================================

#ifndef KALMAN_DEBUG_PRINT
#define KALMAN_DEBUG_PRINT 1
#endif

#ifndef KALMAN_DEBUG_FORCE_FLIGHT
#define KALMAN_DEBUG_FORCE_FLIGHT 1
#endif

/// Print rate decimation: console output happens every N super-loop ticks.
/// At ~1 kHz loop rate, 500 → ~2 Hz.  Tune if output is too fast/slow.
#ifndef KALMAN_DEBUG_PRINT_DECIMATION
#define KALMAN_DEBUG_PRINT_DECIMATION 15
#endif

/// Delay (super-loop ticks) before force-flight auto-transitions.
/// Gives the Rail Shadow time to converge on gravity before seeding ESKF.
#ifndef KALMAN_DEBUG_FORCE_FLIGHT_DELAY_TICKS
#define KALMAN_DEBUG_FORCE_FLIGHT_DELAY_TICKS 1000
#endif

#if KALMAN_DEBUG_PRINT

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "Application/Kalman/kalman_health.hpp"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Data/fsm.hpp"

// ---------------------------------------------------------------------------
// Helpers (quaternion → Euler, FSM name, etc.)
// ---------------------------------------------------------------------------
namespace kalman_debug {

struct EulerDeg {
    double roll;
    double pitch;
    double yaw;
};

inline EulerDeg quaternionToEuler(const double q[4]) {
    const double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    const double roll = std::atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    const double sinp_c = std::max(-1.0, std::min(1.0, sinp));
    const double pitch = std::asin(sinp_c) * (180.0 / M_PI);

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double yaw = std::atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

    return {roll, pitch, yaw};
}

inline EulerDeg quaternionToEulerF(const float q[4]) {
    double qd[4] = {q[0], q[1], q[2], q[3]};
    return quaternionToEuler(qd);
}

inline const char* fsmStateName(flight_computer::State s) {
    switch (s) {
        case flight_computer::State::INIT:            return "INIT";
        case flight_computer::State::CALIBRATION:     return "CALIB";
        case flight_computer::State::FILLING:         return "FILL";
        case flight_computer::State::ARMED:           return "ARMED";
        case flight_computer::State::PRESSURIZATION:  return "PRESS";
        case flight_computer::State::IGNITION:        return "IGN";
        case flight_computer::State::BURN:            return "BURN";
        case flight_computer::State::ASCENT:          return "ASC";
        case flight_computer::State::DESCENT:         return "DESC";
        case flight_computer::State::LANDED:          return "LAND";
        case flight_computer::State::ABORT_ON_GROUND: return "ABT_G";
        case flight_computer::State::ABORT_IN_FLIGHT: return "ABT_F";
        default:                                      return "???";
    }
}

// ---------------------------------------------------------------------------
// Main debug print — call once per kalman_loop() iteration.
//
// Parameters from kalman_process.cpp that aren't reachable through the
// estimator public API are passed explicitly (raw sensor, drained count, …).
// ---------------------------------------------------------------------------
struct RawSensorSnapshot {
    float ax, ay, az;  // Last raw accel sample (body, m/s²)
    float gx, gy, gz;  // Last raw gyro sample (body, rad/s)
    float baro_pa;     // Last baro pressure seen (Pa), 0 if none
    float baro_tempC;  // Last baro temperature (°C), 0 if none
};

inline void printDebugLine(
    const app::EskfEstimator& estimator,
    const KalmanHealthSnapshot& health,
    flight_computer::State fsm_state,
    uint32_t kalman_loop_us,
    uint32_t imu_drained,
    const RawSensorSnapshot& raw)
{
    static uint32_t decimation_counter = 0;
    if (++decimation_counter < KALMAN_DEBUG_PRINT_DECIMATION) {
        return;
    }
    decimation_counter = 0;

    // ── Rail Shadow attitude (always running pre-flight) ─────────────
    const auto& rail = estimator.railShadow();
    double rail_q[4];
    rail.getQuaternion(rail_q);
    const auto rail_euler = quaternionToEuler(rail_q);

    // ── ESKF attitude (valid after liftoff / force-flight) ───────────
    const auto output = estimator.output();
    const auto eskf_euler = quaternionToEulerF(output.quaternion);

    // ── Rewind / buffer stats ────────────────────────────────────────
    const auto& stats = estimator.rewindStats();

    // ── Health summary word ──────────────────────────────────────────
    // Bit 0 = diverged, 1 = alt_valid, 2 = vel_valid,
    // 3 = eskf_valid, 4 = att_valid
    uint8_t hb = 0;
    if (health.diverged)        hb |= 0x01;
    if (health.altitude_valid)  hb |= 0x02;
    if (health.velocity_valid)  hb |= 0x04;
    if (output.eskf_valid)      hb |= 0x08;
    if (output.attitude_valid)  hb |= 0x10;

    // ── Gyro bias from rail shadow (sanity check) ────────────────────
    const auto* gb = rail.gyroBias();

    // ── Print block ──────────────────────────────────────────────────

    // Line 1: Attitude — tilt the board, pitch/roll should follow.
    printf("[KAL] FSM=%-5s  "
           "Rail(p=%+6.1f r=%+6.1f)  "
           "ESKF(p=%+6.1f r=%+6.1f y=%+6.1f)  "
           "alt=%+.1fm vD=%+.1f\n",
           fsmStateName(fsm_state),
           rail_euler.pitch, rail_euler.roll,
           eskf_euler.pitch, eskf_euler.roll, eskf_euler.yaw,
           static_cast<double>(output.altitude_m),
           static_cast<double>(output.velocity_ned[2]));

    // Line 2: Health + timing
    printf("[KAL] hlth=0x%02X  "
           "loop=%luus(max %lu)  "
           "mainLoop=%luus(max %lu)  "
           "NIS=%.1f  hiNIS=%u  "
           "yields=%lu\n",
           hb,
           static_cast<unsigned long>(health.last_kalman_loop_us),
           static_cast<unsigned long>(health.max_kalman_loop_us),
           static_cast<unsigned long>(health.last_main_loop_iteration_us),
           static_cast<unsigned long>(health.max_main_loop_iteration_us),
           static_cast<double>(estimator.eskfLastNis()),
           static_cast<unsigned>(estimator.eskfConsecutiveHighNisCount()),
           static_cast<unsigned long>(stats.catchup_budget_yields));

    // Line 3: Buffer / overflow
    printf("[KAL] IMU=%lu  baro=%lu  gps=%lu  "
           "imuDrop=%lu  baroDrop=%lu  evtDrop=%lu  "
           "imuBuf=%zu  baroBuf=%zu  evtBuf=%zu  "
           "rwd=%lu  drained=%lu\n",
           static_cast<unsigned long>(health.imu_samples_consumed),
           static_cast<unsigned long>(health.baro_updates),
           static_cast<unsigned long>(health.gps_updates),
           static_cast<unsigned long>(stats.imu_drops),
           static_cast<unsigned long>(stats.baro_drops),
           static_cast<unsigned long>(stats.event_drops),
           estimator.filter().imuBufferCount(),
           estimator.filter().baroBufferCount(),
           estimator.filter().eventBufferCount(),
           static_cast<unsigned long>(stats.rewind_count),
           static_cast<unsigned long>(imu_drained));

    // Line 4: Ring HWMs
    printf("[KAL] ring_hwm=[%lu,%lu,%lu]  "
           "gate=%s  grnd=%s  hdg=%s\n",
           static_cast<unsigned long>(health.imu_ring_hwm[0]),
           static_cast<unsigned long>(health.imu_ring_hwm[1]),
           static_cast<unsigned long>(health.imu_ring_hwm[2]),
           rail.isGateOpen() ? "OPEN" : "SHUT",
           rail.isGroundReferenceValid() ? "OK" : "NO",
           rail.isHeadingInitialized() ? "OK" : "NO");

    // Line 5: Raw sensor values (last sample this tick)
    printf("[KAL] raw: ax=%+7.2f ay=%+7.2f az=%+7.2f  "
           "gx=%+6.3f gy=%+6.3f gz=%+6.3f  "
           "P=%.0fPa T=%.1fC\n",
           static_cast<double>(raw.ax), static_cast<double>(raw.ay),
           static_cast<double>(raw.az),
           static_cast<double>(raw.gx), static_cast<double>(raw.gy),
           static_cast<double>(raw.gz),
           static_cast<double>(raw.baro_pa),
           static_cast<double>(raw.baro_tempC));

    // Line 6: Gyro bias from rail shadow (should converge to ~0 when still)
    printf("[KAL] gyroBias: %+.5f %+.5f %+.5f (rad/s)\n",
           static_cast<double>(gb[0]),
           static_cast<double>(gb[1]),
           static_cast<double>(gb[2]));

    // Separator for readability
    printf("---\n");
}

} // namespace kalman_debug

#endif // KALMAN_DEBUG_PRINT
