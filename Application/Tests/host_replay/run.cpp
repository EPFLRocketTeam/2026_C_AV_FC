// Replay recorded IMU/baro data through the firmware kalman_loop() with a
// scripted FSM that makes the same lifecycle calls as AvState::update().
//
// Usage: harness imu=<csv> baro=<csv> out=<csv> [key=value ...]
//   mode=flown      INIT -> BURN at t_burn (as in the test flight)
//   mode=ignition   INIT -> IGNITION at t_ign; BURN on cable (t_cable) or
//                   vertical_acc_hold == DID_HOLD; ABORT on DID_NOT_HOLD
//                   without cable (FSM spec, no valve bypass)
//   t_burn, t_ign, t_cable : absolute us (0 = never)
//   burn_to_ascent_ms      : BURN -> ASCENT delay (default 2000)
//   detector=0             : never allow IMU liftoff detection
//   reset_at=<us>          : call kalman_request_reset() (new code only)
//   reset_at2=<us>         : second reset request
//   fake_gps=1             : inject bogus GNSS fixes at 5 Hz (must be ignored)
//   drop_imu=<i>@<us>      : mark IMU i unhealthy from time us
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "Application/Data/data.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/Data/ring_buffer.hpp"
#include "Application/Kalman/kalman/eskf_logger.hpp"
#include "Application/Kalman/kalman_lifecycle.h"
extern "C" {
#include "Application/Kalman/kalman_process.h"
}
#include "Drivers/BMP390/BMP390.h"
#include "Drivers/InvIMU/InvIMU.h"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

using Drivers::InvIMU::IMUData;
extern uint64_t g_sim_us;
extern bool g_printf_enabled;
extern bool g_liftoff_detection_allowed;
extern "C" bool g_imu_healthy_sim[4];
extern RingBuffer<IMUData, 128u> imuData1, imuData2, imuData3, imuData4;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData1, baroData2, baroData3, baroData4;
extern RingBuffer<GpsBasicFixData, 100> gpsData;
#ifdef HARNESS_HAS_RESET
extern "C" uint8_t kalman_request_reset(void);
#endif

namespace {

struct ImuRow { uint64_t ts; uint8_t idx; float a[3], g[3], temp; };
struct BaroRow { uint64_t deliver_us, trigger_us; uint8_t idx; float p, t; };

struct CaptureLogger : eskf::IEskfLogger {
  eskf::StateSnapshot last_state{}; bool has_state = false;
  eskf::FlightShadowSnapshot last_fs{}; bool has_fs = false;
  uint32_t diverged = 0;
  void logState(const eskf::StateSnapshot &s) override { last_state = s; has_state = true; }
  void logCovariance(const eskf::CovarianceSnapshot &) override {}
  void logEvent(eskf::EskfEventType e, uint64_t, float) override {
    if (e == eskf::EskfEventType::FilterDiverged) ++diverged;
  }
  void logGpsRejection(eskf::EskfEventType, uint64_t, const eskf::GpsRejectionInfo &) override {}
  void logRewind(eskf::EskfEventType, uint64_t, const eskf::RewindInfo &) override {}
  void logRailShadow(const eskf::RailShadowSnapshot &) override {}
  void logFlightShadow(const eskf::FlightShadowSnapshot &s) override { last_fs = s; has_fs = true; }
};
CaptureLogger g_logger;

uint64_t argU(int argc, char **argv, const char *k, uint64_t def) {
  const size_t n = std::strlen(k);
  for (int i = 1; i < argc; ++i)
    if (!std::strncmp(argv[i], k, n) && argv[i][n] == '=') return std::strtoull(argv[i] + n + 1, nullptr, 10);
  return def;
}
std::string argS(int argc, char **argv, const char *k, const char *def) {
  const size_t n = std::strlen(k);
  for (int i = 1; i < argc; ++i)
    if (!std::strncmp(argv[i], k, n) && argv[i][n] == '=') return argv[i] + n + 1;
  return def;
}

const char *stateName(uint32_t s) {
  static const char *n[] = {"INIT", "CALIBRATION", "FILLING", "ARMED", "PRESSURIZATION", "IGNITION",
                            "BURN", "ASCENT", "DESCENT", "LANDED", "ABORT_ON_GROUND", "ABORT_IN_FLIGHT"};
  return s < 12 ? n[s] : "?";
}

uint32_t g_state = flight_computer::State::INIT;
void transition(uint32_t to) {
  std::printf("[t=%.6f] FSM %s -> %s\n", g_sim_us / 1e6, stateName(g_state), stateName(to));
  g_state = to;
  flight_computer::GOATStore::get_instance().stateStore.set(static_cast<flight_computer::State>(to));
  kalman_on_state_change(to);
  if (to == flight_computer::State::BURN) kalman_on_liftoff(static_cast<uint32_t>(g_sim_us / 1000u));
}

}  // namespace

int main(int argc, char **argv) {
  const std::string imu_path = argS(argc, argv, "imu", ""), baro_path = argS(argc, argv, "baro", "");
  const std::string out_path = argS(argc, argv, "out", "out.csv"), mode = argS(argc, argv, "mode", "flown");
  const uint64_t t_burn = argU(argc, argv, "t_burn", 0), t_ign = argU(argc, argv, "t_ign", 0);
  const uint64_t t_cable = argU(argc, argv, "t_cable", 0);
  const uint64_t burn_to_ascent_us = argU(argc, argv, "burn_to_ascent_ms", 2000) * 1000u;
  const bool detector = argU(argc, argv, "detector", 1) != 0, fake_gps = argU(argc, argv, "fake_gps", 0) != 0;
  const uint64_t reset_at = argU(argc, argv, "reset_at", 0), reset_at2 = argU(argc, argv, "reset_at2", 0);
  const std::string drop = argS(argc, argv, "drop_imu", "");
  int drop_idx = -1; uint64_t drop_us = 0;
  if (!drop.empty()) { drop_idx = std::atoi(drop.c_str()); drop_us = std::strtoull(drop.c_str() + drop.find('@') + 1, nullptr, 10); }
  g_printf_enabled = argU(argc, argv, "verbose", 0) != 0;

  // ---- Load data ------------------------------------------------------
  std::vector<ImuRow> imu; imu.reserve(7000000);
  { FILE *f = std::fopen(imu_path.c_str(), "r"); if (!f) { std::perror("imu"); return 1; }
    unsigned long long ts, ts2; unsigned idx; ImuRow r{};
    while (std::fscanf(f, "%llu,%u,%f,%f,%f,%f,%f,%f,%f,%llu", &ts, &idx, &r.a[0], &r.a[1], &r.a[2],
                       &r.g[0], &r.g[1], &r.g[2], &r.temp, &ts2) == 10) { r.ts = ts2; r.idx = idx; imu.push_back(r); }
    std::fclose(f); }
  std::stable_sort(imu.begin(), imu.end(), [](const ImuRow &x, const ImuRow &y) { return x.ts < y.ts; });
  std::vector<BaroRow> baro;
  { FILE *f = std::fopen(baro_path.c_str(), "r"); if (!f) { std::perror("baro"); return 1; }
    unsigned long long d, tr; unsigned idx; BaroRow r{};
    while (std::fscanf(f, "%llu,%u,%f,%f,%llu", &d, &idx, &r.p, &r.t, &tr) == 5) { r.deliver_us = d; r.trigger_us = tr; r.idx = idx; baro.push_back(r); }
    std::fclose(f); }
  std::stable_sort(baro.begin(), baro.end(), [](const BaroRow &x, const BaroRow &y) { return x.deliver_us < y.deliver_us; });
  std::vector<uint64_t> triggers;
  for (const auto &b : baro) triggers.push_back(b.trigger_us);
  std::sort(triggers.begin(), triggers.end());
  triggers.erase(std::unique(triggers.begin(), triggers.end()), triggers.end());
  std::fprintf(stderr, "loaded imu=%zu baro=%zu\n", imu.size(), baro.size());

  eskf::setEskfLogger(&g_logger);
  RingBuffer<IMUData, 128u> *imu_rings[4] = {&imuData1, &imuData2, &imuData3, &imuData4};
  RingBuffer<Drivers::BMP390::BaroData, 100> *baro_rings[4] = {&baroData1, &baroData2, &baroData3, &baroData4};

  FILE *out = std::fopen(out_path.c_str(), "w");
  if (!out) { std::perror(out_path.c_str()); return 1; }
  std::fprintf(out, "t_us,state,nav_alt_up,nav_vz_up,eskf_alt_up,eskf_vz_up,fs_alt_up,fs_vz_up,apogee,imu_liftoff,acc_hold\n");

  const uint64_t t0 = imu.front().ts, t_end = imu.back().ts;
  const uint64_t tick_us = 500;
  size_t ii = 0, bi = 0, ti = 0;
  uint64_t burn_entry = 0, next_rec = 0, next_gps = t0 + 1000000;
  bool reset_done = false, reset2_done = false;
  uint8_t prev_acc = 0; bool prev_apogee = false, prev_lift = false;
  auto &goat = flight_computer::GOATStore::get_instance();

  for (g_sim_us = t0; g_sim_us <= t_end; g_sim_us += tick_us) {
    if (detector && g_sim_us >= t0 + 3000000) g_liftoff_detection_allowed = true;
    if (drop_idx >= 0 && g_sim_us >= drop_us) g_imu_healthy_sim[drop_idx] = false;
    while (ii < imu.size() && imu[ii].ts <= g_sim_us) {
      const ImuRow &r = imu[ii++];
      if (r.idx < 4 && g_imu_healthy_sim[r.idx]) {
        IMUData d{r.a[0], r.a[1], r.a[2], r.g[0], r.g[1], r.g[2], r.temp, r.ts};
        imu_rings[r.idx]->append(d);
      }
    }
    while (ti < triggers.size() && triggers[ti] <= g_sim_us) kalman_note_baro_trigger(triggers[ti++]);
    while (bi < baro.size() && baro[bi].deliver_us <= g_sim_us) {
      const BaroRow &r = baro[bi++];
      Drivers::BMP390::BaroData d{r.p, r.t, r.trigger_us};
      if (r.idx < 4) baro_rings[r.idx]->append(d);
    }
    if (fake_gps && g_sim_us >= next_gps) {  // bogus fix 5 km away, moving 50 m/s
      next_gps = g_sim_us + 200000;
      GpsBasicFixData fx{};
      fx.timestamp_us = g_sim_us; fx.lat = 465000000; fx.lon = 65000000; fx.hMSL = 5000000;
      fx.velN = 50000; fx.velD = -20000; fx.fixType = GpsFixType::FIX_3D; fx.flags.gnssFixOK = true;
      fx.numSV = 12; fx.hAcc = 1000; fx.vAcc = 1500; fx.sAcc = 200; fx.pDOP = 100;
      gpsData.append(fx);
    }
#ifdef HARNESS_HAS_RESET
    if (reset_at && !reset_done && g_sim_us >= reset_at) {
      reset_done = true;
      std::printf("[t=%.6f] kalman_request_reset() -> %u\n", g_sim_us / 1e6, (unsigned)kalman_request_reset());
    }
    if (reset_at2 && !reset2_done && g_sim_us >= reset_at2) {
      reset2_done = true;
      std::printf("[t=%.6f] kalman_request_reset() -> %u\n", g_sim_us / 1e6, (unsigned)kalman_request_reset());
    }
#endif

    kalman_loop();

    // ---- scripted FSM (runs after kalman_loop like fsm_tick) ----
    const auto ev = goat.eventStore.get();
    using flight_computer::State;
    if (mode == "flown") {
      if (g_state == State::INIT && t_burn && g_sim_us >= t_burn) { transition(State::BURN); burn_entry = g_sim_us; }
    } else {
      if (g_state == State::INIT && t_ign && g_sim_us >= t_ign) transition(State::IGNITION);
      else if (g_state == State::IGNITION) {
        const bool cable_lost = t_cable && g_sim_us >= t_cable;
        if (cable_lost || ev.vertical_acc_hold == flight_computer::ACC_HOLD_DID_HOLD) { transition(State::BURN); burn_entry = g_sim_us; }
        else if (ev.vertical_acc_hold == flight_computer::ACC_HOLD_DID_NOT_HOLD) transition(State::ABORT_ON_GROUND);
      }
    }
    if (g_state == State::BURN && g_sim_us - burn_entry >= burn_to_ascent_us) transition(State::ASCENT);
    else if (g_state == State::ASCENT && ev.apogee_detected) transition(State::DESCENT);

    if (ev.vertical_acc_hold != prev_acc) { std::printf("[t=%.6f] vertical_acc_hold=%u\n", g_sim_us / 1e6, ev.vertical_acc_hold); prev_acc = ev.vertical_acc_hold; }
    if (ev.imu_liftoff_detected != prev_lift) { std::printf("[t=%.6f] imu_liftoff_detected=%d\n", g_sim_us / 1e6, ev.imu_liftoff_detected); prev_lift = ev.imu_liftoff_detected; }
    if (ev.apogee_detected != prev_apogee) { std::printf("[t=%.6f] apogee_detected=%d\n", g_sim_us / 1e6, ev.apogee_detected); prev_apogee = ev.apogee_detected; }

    if (g_sim_us >= next_rec) {
      next_rec = g_sim_us + 10000;
      const auto nav = goat.navigationDataStore.get();
      std::fprintf(out, "%llu,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%u\n", (unsigned long long)g_sim_us, stateName(g_state),
                   nav.altitude, -nav.speed.z,
                   g_logger.has_state ? -g_logger.last_state.p[2] : 0.0, g_logger.has_state ? -g_logger.last_state.v[2] : 0.0,
                   g_logger.has_fs ? g_logger.last_fs.altitude_m : 0.0, g_logger.has_fs ? -g_logger.last_fs.velocity_mps : 0.0,
                   ev.apogee_detected, ev.imu_liftoff_detected, ev.vertical_acc_hold);
    }
  }
  std::fclose(out);
  std::printf("diverged_events=%u\n", g_logger.diverged);
  return 0;
}
