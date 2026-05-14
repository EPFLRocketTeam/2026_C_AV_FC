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
#define KALMAN_DEBUG_PRINT 0
#endif

#ifndef KALMAN_DEBUG_FORCE_FLIGHT
#define KALMAN_DEBUG_FORCE_FLIGHT 0
#endif

/// Print rate decimation: console output happens every N super-loop ticks.
/// At ~1 kHz loop rate, 500 → ~2 Hz.  Tune if output is too fast/slow.
#ifndef KALMAN_DEBUG_PRINT_DECIMATION
#define KALMAN_DEBUG_PRINT_DECIMATION 1000
#endif

/// Delay (super-loop ticks) before force-flight auto-transitions.
/// Gives the Rail Shadow time to converge on gravity before seeding ESKF.
#ifndef KALMAN_DEBUG_FORCE_FLIGHT_DELAY_TICKS
#define KALMAN_DEBUG_FORCE_FLIGHT_DELAY_TICKS 5000
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
    // Per-sensor raw baro values (up to 4)
    float baro_per_sensor_pa[4];     // Raw pressure per baro source (Pa)
    float baro_per_sensor_tempC[4];  // Raw temperature per baro source (°C)
    uint8_t baro_per_sensor_alive;   // Bitmask of sensors that produced data
    // Per-IMU raw accel magnitude
    float imu_per_sensor_amag[3];    // |a| per IMU source (m/s²)
    uint8_t imu_per_sensor_alive;    // Bitmask of IMUs that produced data
    uint32_t frame_h78;  // Count of 0x78 frames (normal hires)
    uint32_t frame_hF0;  // Count of 0xF0 frames (ext_header)
    uint32_t frame_other; // Count of other/unknown frames
    uint32_t imu_status_flags; // IMU driver status flags (SPI errors etc)
    uint32_t imu_drop_count;   // IMU ring buffer overflow count
    // Timing breakdown (filled by kalman_loop)
    uint32_t t_imu_drain_us;   // Time spent draining IMU ring buffers
    uint32_t t_imu_process_us; // Time spent in IMU ingestion + predict
    uint32_t t_aiding_us;      // Time spent in baro/GPS ingestion
    uint32_t t_tick_us;        // Time spent in onTick (catchUp)
    uint32_t t_output_us;      // Time spent in output bridge + apogee
};

inline void printDebugLine(
    const app::EskfEstimator& estimator,
    const KalmanHealthSnapshot& health,
    flight_computer::State fsm_state,
    uint32_t kalman_loop_us,
    uint32_t imu_drained,
    RawSensorSnapshot& raw)
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
           "alt=%+.1fm vD=%+.1f\r\n",
           fsmStateName(fsm_state),
           rail_euler.pitch, rail_euler.roll,
           eskf_euler.pitch, eskf_euler.roll, eskf_euler.yaw,
           static_cast<double>(output.altitude_m),
           static_cast<double>(output.velocity_ned[2]));

    // Line 2: Health + timing
    printf("[KAL] hlth=0x%02X  "
           "loop=%luus(max %lu)  "
           "mainLoop=%luus(max %lu)  "
           "NIS=%.1f  hiNIS=%u  div=%d  "
           "yields=%lu\r\n",
           hb,
           static_cast<unsigned long>(health.last_kalman_loop_us),
           static_cast<unsigned long>(health.max_kalman_loop_us),
           static_cast<unsigned long>(health.last_main_loop_iteration_us),
           static_cast<unsigned long>(health.max_main_loop_iteration_us),
           static_cast<double>(estimator.eskfLastNis()),
           static_cast<unsigned>(estimator.eskfConsecutiveHighNisCount()),
           static_cast<int>(estimator.eskfHasDiverged()),
           static_cast<unsigned long>(stats.catchup_budget_yields));

    // Line 3: Buffer / overflow
    printf("[KAL] IMU=%lu  baro=%lu  gps=%lu  "
           "imuDrop=%lu  baroDrop=%lu  evtDrop=%lu  staleSkip=%lu  "
           "imuBuf=%lu  baroBuf=%lu  evtBuf=%lu  "
           "rwd=%lu  drained=%lu\r\n",
           static_cast<unsigned long>(health.imu_samples_consumed),
           static_cast<unsigned long>(health.baro_updates),
           static_cast<unsigned long>(health.gps_updates),
           static_cast<unsigned long>(stats.imu_drops),
           static_cast<unsigned long>(stats.baro_drops),
           static_cast<unsigned long>(stats.event_drops),
           static_cast<unsigned long>(stats.stale_skips),
           static_cast<unsigned long>(estimator.filter().imuBufferCount()),
           static_cast<unsigned long>(estimator.filter().baroBufferCount()),
           static_cast<unsigned long>(estimator.filter().eventBufferCount()),
           static_cast<unsigned long>(stats.rewind_count),
           static_cast<unsigned long>(imu_drained));

    // Line 4: Ring HWMs
    printf("[KAL] ring_hwm=[%lu,%lu,%lu]  imuRdIdx=%lu  "
           "gate=%s  grnd=%s  hdg=%s\r\n",
           static_cast<unsigned long>(health.imu_ring_hwm[0]),
           static_cast<unsigned long>(health.imu_ring_hwm[1]),
           static_cast<unsigned long>(health.imu_ring_hwm[2]),
           static_cast<unsigned long>(estimator.filter().imuReadIdx()),
           rail.isGateOpen() ? "OPEN" : "SHUT",
           rail.isGroundReferenceValid() ? "OK" : "NO",
           rail.isHeadingInitialized() ? "OK" : "NO");

    // Line 5: Raw sensor values (last sample this tick)
    const double amag = std::sqrt(
        static_cast<double>(raw.ax) * raw.ax +
        static_cast<double>(raw.ay) * raw.ay +
        static_cast<double>(raw.az) * raw.az);
    printf("[KAL] raw: ax=%+7.2f ay=%+7.2f az=%+7.2f  "
           "gx=%+6.3f gy=%+6.3f gz=%+6.3f  "
           "|a|=%.2f  "
           "P=%.0fPa T=%.1fC\r\n",
           static_cast<double>(raw.ax), static_cast<double>(raw.ay),
           static_cast<double>(raw.az),
           static_cast<double>(raw.gx), static_cast<double>(raw.gy),
           static_cast<double>(raw.gz),
           amag,
           static_cast<double>(raw.baro_pa),
           static_cast<double>(raw.baro_tempC));

    // Line 5b: Per-sensor raw baro values
    printf("[KAL] baroRaw:");
    for (int i = 0; i < 4; ++i) {
      if (raw.baro_per_sensor_alive & (1u << i)) {
        printf(" [%d]=%.0fPa/%.1fC", i,
               static_cast<double>(raw.baro_per_sensor_pa[i]),
               static_cast<double>(raw.baro_per_sensor_tempC[i]));
      } else {
        printf(" [%d]=--", i);
      }
    }
    printf("  alive=0x%X\r\n", raw.baro_per_sensor_alive);

    // Line 5c: Per-IMU raw accel magnitudes
    printf("[KAL] imuRaw:");
    for (int i = 0; i < 3; ++i) {
      if (raw.imu_per_sensor_alive & (1u << i)) {
        printf(" [%d]|a|=%.2f", i,
               static_cast<double>(raw.imu_per_sensor_amag[i]));
      } else {
        printf(" [%d]=--", i);
      }
    }
    printf("  alive=0x%X\r\n", raw.imu_per_sensor_alive);

    // Line 6: Gyro bias from rail shadow (should converge to ~0 when still)
    printf("[KAL] gyroBias: %+.5f %+.5f %+.5f (rad/s)\r\n",
           static_cast<double>(gb[0]),
           static_cast<double>(gb[1]),
           static_cast<double>(gb[2]));

    // Line 7: IMU FIFO frame header statistics
    printf("[KAL] fifoHdr: 0x78=%lu  0xF0=%lu  other=%lu  "
           "imuFlags=0x%lX  imuDrvDrop=%lu\r\n",
           static_cast<unsigned long>(raw.frame_h78),
           static_cast<unsigned long>(raw.frame_hF0),
           static_cast<unsigned long>(raw.frame_other),
           static_cast<unsigned long>(raw.imu_status_flags),
           static_cast<unsigned long>(raw.imu_drop_count));

    // Line 8: CatchUp budget details
    printf("[KAL] catchUp: budget=%luus  last=%luus  events=%lu  "
           "dtClamp=%lu\r\n",
           static_cast<unsigned long>(estimator.catchupBudgetUs()),
           static_cast<unsigned long>(estimator.lastCatchupDurationUs()),
           static_cast<unsigned long>(estimator.lastCatchupEventsProcessed()),
           static_cast<unsigned long>(estimator.predictDtClampedCount()));

    // Line 9: Timing breakdown (where time is spent in kalman_loop)
    printf("[KAL] timing: drain=%luus  imuProc=%luus  aiding=%luus  "
           "tick=%luus  output=%luus\r\n",
           static_cast<unsigned long>(raw.t_imu_drain_us),
           static_cast<unsigned long>(raw.t_imu_process_us),
           static_cast<unsigned long>(raw.t_aiding_us),
           static_cast<unsigned long>(raw.t_tick_us),
           static_cast<unsigned long>(raw.t_output_us));

    // Line 9b: VirtualIMU vs post-processing breakdown inside imuProc
    {
        uint32_t vimu_us = 0, post_us = 0, vimu_samples = 0;
        estimator.getImuProcBreakdown(vimu_us, post_us, vimu_samples);
        const uint32_t vimu_per = vimu_samples > 0 ? vimu_us / vimu_samples : 0;
        const uint32_t post_per = vimu_samples > 0 ? post_us / vimu_samples : 0;
        printf("[KAL] imuBreak: vimu=%luus  post=%luus  n=%lu  "
               "vimu/s=%luus  post/s=%luus\r\n",
               static_cast<unsigned long>(vimu_us),
               static_cast<unsigned long>(post_us),
               static_cast<unsigned long>(vimu_samples),
               static_cast<unsigned long>(vimu_per),
               static_cast<unsigned long>(post_per));
        const_cast<app::EskfEstimator&>(estimator).resetImuProcBreakdown();
    }

    // Line 10: BURN-phase diagnostics (ESKF state, aero-blind, corrections)
    if (fsm_state == flight_computer::State::BURN ||
        fsm_state == flight_computer::State::ASCENT ||
        estimator.inFlight()) {
        const auto& core = estimator.filter().core();
        const auto& P = core.covariance();
        const auto& st = core.state();
        const bool aero_blind = estimator.flightShadow().isAeroBlind();

        // Position and velocity covariance diagonal (σ in meters)
        printf("[KAL] Pcov: σp=%.2f,%.2f,%.2f  σv=%.2f,%.2f,%.2f  "
               "σθ=%.3f  σba=%.4f  σbg=%.6f  σbb=%.2f\r\n",
               std::sqrt(static_cast<double>(P.diag(0))),
               std::sqrt(static_cast<double>(P.diag(1))),
               std::sqrt(static_cast<double>(P.diag(2))),
               std::sqrt(static_cast<double>(P.diag(3))),
               std::sqrt(static_cast<double>(P.diag(4))),
               std::sqrt(static_cast<double>(P.diag(5))),
               std::sqrt(static_cast<double>(P.diag(6))),
               std::sqrt(static_cast<double>(P.diag(9))),
               std::sqrt(static_cast<double>(P.diag(12))),
               std::sqrt(static_cast<double>(P.diag(15))));

        // ESKF position (NED), velocity, and baro bias
        printf("[KAL] eskf: pN=%+.2f pE=%+.2f pD=%+.2f  "
               "vN=%+.2f vE=%+.2f vD=%+.2f  bBaro=%+.2f  "
               "aeroBlind=%d  inn=%.2f  NIS=%.2f\r\n",
               static_cast<double>(st.p[0]),
               static_cast<double>(st.p[1]),
               static_cast<double>(st.p[2]),
               static_cast<double>(st.v[0]),
               static_cast<double>(st.v[1]),
               static_cast<double>(st.v[2]),
               static_cast<double>(st.b_baro),
               static_cast<int>(aero_blind),
               static_cast<double>(core.lastInnovation()),
               static_cast<double>(core.lastNIS()));
    }

    // Reset per-sensor alive bitmasks after printing so they accumulate
    // across all kalman_loop() calls until the next decimated print.
    raw.baro_per_sensor_alive = 0;
    raw.imu_per_sensor_alive = 0;

    // Separator for readability
    printf("---\r\n");
}

} // namespace kalman_debug

#endif // KALMAN_DEBUG_PRINT
