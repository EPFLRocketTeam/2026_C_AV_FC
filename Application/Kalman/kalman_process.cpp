
#include "Core/Inc/main.h"
// #include "cmsis_os.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Kalman/AppLayer/apogee_hub.hpp"
#include "Application/Kalman/AppLayer/apogee_factory.hpp"
#include "Application/Kalman/AppLayer/hw_config.hpp"
#include "Application/Kalman/AppLayer/hw_calibration_data.hpp"
#include "Application/Kalman/AppLayer/output_bridge.hpp"
#include "Application/Kalman/kalman_health.hpp"
#include "Application/Kalman/kalman_debug.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/Data/data.hpp"
#include "Application/FlightControl/threshold.h"
#include "Application/FlightControl/liftoff_detector.hpp"
#include "Application/Modules/baro_module.hpp"
#include "Application/Modules/imu_modlue.hpp"

extern bool g_liftoff_detection_allowed;

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Application/app_timebase.h"
#include "Application/main.h"
#include "Drivers/InvIMU/InvIMU.h"
#include "Application/FlightControl/uart_cmd.h"
}
#include "Application/app_printf.h"
#include "Drivers/InvIMU/InvIMU.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"
// After ubx_gps_interface.h: FlightParams.hpp defines a FIXED macro that
// collides with GpsCarrierPhaseStatus::FIXED.
#include "Application/Config/config.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

// extern osMutexId_t eventStoreMutexHandle;
// extern osMutexId_t navigationDataMutexHandle;


extern AppImuRingBuffer imuData1;
extern AppImuRingBuffer imuData2;
extern AppImuRingBuffer imuData3;
extern AppImuRingBuffer imuData4;

extern RingBuffer<GpsBasicFixData, 100> gpsData;


namespace {

using BaroData = Drivers::BMP390::BaroData;

constexpr size_t kMaxImuSamplesPerSourcePerRun = 32u;  // was 128; lower cap
    // prevents the vicious drain-accumulate cycle (draining 100+ takes 17ms,
    // which causes 100+ more to accumulate). With push decimation at 4x,
    // 32 drained → 8 pushed to ESKF → catchUp ~1.8ms.  Steady state converges
    // to ~4 samples/tick at ~624µs.  Excess samples stay in the ring buffer
    // and are overwritten by newer data (ring buffer discards oldest).
constexpr size_t kMaxImuSamplesPerEstimatorBatch = 16u;
constexpr size_t kMaxBaroSamplesPerSourcePerRun = 8u;
constexpr size_t kMaxGpsSamplesPerRun = 8u;

// GNSS fusion policy. Default is log-only: fixes are still received, logged
// (raw UBX via the GPS driver callback) and drained here, but never fed to
// the estimator. GNSS fusion was never flight-tested, and apogee detection
// does not need it. The logs allow replaying the flight offline with fusion
// enabled. Build with -DKALMAN_GNSS_FUSION_ENABLE=1 to fuse GNSS.
#ifndef KALMAN_GNSS_FUSION_ENABLE
#define KALMAN_GNSS_FUSION_ENABLE 0
#endif
#if APP_IMU_PRIMARY_ODR_HZ > 0
constexpr uint32_t kNominalImuDtUs =
	static_cast<uint32_t>(1000000ULL / APP_IMU_PRIMARY_ODR_HZ);
#else
constexpr uint32_t kNominalImuDtUs = 1000U;
#endif

std::atomic<bool> g_reset_requested{false};

/// States in which a requested Kalman reset is honoured: on the ground, with
/// enough time before launch for the preflight estimators to reconverge
/// (tens of seconds to ~2 minutes of stationary data).
bool kalmanResetAllowedIn(uint32_t raw_state) {
	switch (raw_state) {
	case flight_computer::State::INIT:
	case flight_computer::State::CALIBRATION:
	case flight_computer::State::FILLING:
	case flight_computer::State::ARMED:
	case flight_computer::State::ABORT_ON_GROUND:
	case flight_computer::State::LANDED:
		return true;
	default:
		return false;
	}
}

std::atomic<uint32_t> g_last_main_loop_iteration_us{0u};
std::atomic<uint32_t> g_max_main_loop_iteration_us{0u};
volatile uint64_t g_pending_baro_trigger_us = 0u;

// ---------------------------------------------------------------
// Liftoff acceleration-hold evaluator (FSM IGNITION -> BURN/ABORT).
//
// After the FSM enters IGNITION, this waits
// Ignition.TotalTimeUntilHoldDownMs() (prechill + igniter + delay +
// ramp-up + hold-down), then averages the thrust-axis acceleration over
// Ignition.LiftoffAccelDurationMs and compares the mean to
// Ignition.LiftoffAccelThreshold. The result is written as a tri-state
// AccHoldStatus into the GOATStore eventStore so that the FSM can decide
// between BURN and ABORT_ON_GROUND.
//
// Input is every raw IMU sample (independent of the estimator's
// preprocessing) rotated by the fixed sensor-to-body mounting matrix,
// body +X = nose. It is specific force, so it reads ~+9.81 m/s^2 on the
// pad and the threshold applies to that value. Samples are selected by
// their own timestamps; the decision uses the median of the per-IMU
// means so that a single failed IMU cannot flip it.
// ---------------------------------------------------------------
struct LiftoffAccHold {
	static constexpr size_t kMaxSources = 4u;
	/// Grace period after the window end for samples still in the rings.
	static constexpr uint64_t kLateSampleMarginUs = 10000u;

	bool active = false;             ///< Evaluation in progress
	uint64_t window_start_us = 0;    ///< Evaluation window [start, end)
	uint64_t window_end_us = 0;
	double accel_sum[kMaxSources] = {};
	uint32_t accel_count[kMaxSources] = {};

	void reset() {
		active = false;
		window_start_us = 0;
		window_end_us = 0;
		for (size_t i = 0; i < kMaxSources; ++i) {
			accel_sum[i] = 0.0;
			accel_count[i] = 0;
		}
	}

	void onIgnitionEnter(uint64_t now_us) {
		reset();
		const auto &ignition = config::get().Ignition;
		const uint64_t hold_down_us = static_cast<uint64_t>(
			ignition.TotalTimeUntilHoldDownMs() * 1000.0f);
		const uint64_t window_us =
			static_cast<uint64_t>(ignition.LiftoffAccelDurationMs) * 1000ULL;
		active = true;
		window_start_us = now_us + hold_down_us;
		window_end_us = window_start_us + window_us;
	}

	/// Feed one raw IMU sample.
	/// @param thrust_axis_accel_mps2  Body +X specific force (m/s^2).
	void addSample(size_t source, float thrust_axis_accel_mps2,
				   uint64_t sample_ts_us) {
		if (!active || source >= kMaxSources ||
			sample_ts_us < window_start_us || sample_ts_us >= window_end_us) {
			return;
		}
		accel_sum[source] += static_cast<double>(thrust_axis_accel_mps2);
		accel_count[source]++;
	}

	/// Call every Kalman tick while active.
	/// @return ACC_HOLD_NOT_ELAPSED while still evaluating,
	///         ACC_HOLD_DID_HOLD / ACC_HOLD_DID_NOT_HOLD when done.
	flight_computer::AccHoldStatus evaluate(uint64_t now_us) {
		using flight_computer::ACC_HOLD_NOT_ELAPSED;
		using flight_computer::ACC_HOLD_DID_HOLD;
		using flight_computer::ACC_HOLD_DID_NOT_HOLD;

		if (!active || now_us < window_end_us + kLateSampleMarginUs) {
			return ACC_HOLD_NOT_ELAPSED;
		}
		active = false;

		double means[kMaxSources] = {};
		size_t n = 0;
		for (size_t i = 0; i < kMaxSources; ++i) {
			if (accel_count[i] > 0) {
				means[n++] = accel_sum[i] / static_cast<double>(accel_count[i]);
			}
		}
		if (n == 0) {
			app_printf("[ACC-HOLD] No IMU samples in window -> DID_NOT_HOLD\r\n");
			return ACC_HOLD_DID_NOT_HOLD;
		}
		std::sort(means, means + n);
		const double median = (n % 2u == 1u)
			? means[n / 2u]
			: 0.5 * (means[n / 2u - 1u] + means[n / 2u]);
		const double threshold =
			static_cast<double>(config::get().Ignition.LiftoffAccelThreshold);
		const bool held = median > threshold;
		app_printf("[ACC-HOLD] median=%.2f m/s2 (n_imu=%u) threshold=%.2f -> %s\r\n",
		       median, static_cast<unsigned>(n), threshold,
		       held ? "DID_HOLD" : "DID_NOT_HOLD");
		return held ? ACC_HOLD_DID_HOLD : ACC_HOLD_DID_NOT_HOLD;
	}
};

// ---------------------------------------------------------------
// Touchdown detector (FSM DESCENT -> LANDED).
//
// Baro-only, so it depends neither on GNSS nor on the navigation filters:
// the least-squares slope of the fused barometer altitude over the last
// kWindowUs must stay below kMaxSpeedMps (FSM spec SPEED_ZERO) for
// kConfirmUs. Under parachute the descent rate is several m/s, so this only
// holds on the ground (or if the rocket hangs somewhere). Runs in DESCENT
// and LANDED. The flag is live, not latched; the FSM additionally requires
// Descent.MaxDurationMs.
// ---------------------------------------------------------------
struct TouchdownDetector {
	static constexpr size_t kCapacity = 256u;
	static constexpr uint64_t kWindowUs = 2500000u;
	static constexpr uint64_t kConfirmUs = 3000000u;
	static constexpr uint64_t kStaleUs = 2000000u;
	static constexpr size_t kMinSamples = 10u;
	static constexpr double kMaxSpeedMps = 0.5;

	uint64_t ts[kCapacity] = {};
	float alt[kCapacity] = {};
	size_t head = 0;   ///< Oldest sample
	size_t count = 0;
	uint64_t still_since_us = 0;
	bool detected = false;

	void reset() {
		head = 0;
		count = 0;
		still_since_us = 0;
		detected = false;
	}

	void addSample(uint64_t sample_ts_us, float altitude_m) {
		if (count > 0 && sample_ts_us <= ts[(head + count - 1u) % kCapacity]) {
			return;  // Same fused sample as last tick
		}
		if (count == kCapacity) {
			head = (head + 1u) % kCapacity;
			--count;
		}
		ts[(head + count) % kCapacity] = sample_ts_us;
		alt[(head + count) % kCapacity] = altitude_m;
		++count;
		while (count > 0 && ts[head] + kWindowUs < sample_ts_us) {
			head = (head + 1u) % kCapacity;
			--count;
		}
	}

	void update(uint64_t now_us) {
		const uint64_t newest = count ? ts[(head + count - 1u) % kCapacity] : 0u;
		if (count < kMinSamples || now_us > newest + kStaleUs ||
			newest - ts[head] < (kWindowUs * 8u) / 10u) {
			still_since_us = 0;
			detected = false;
			return;
		}
		double mt = 0.0, ma = 0.0;
		for (size_t i = 0; i < count; ++i) {
			const size_t k = (head + i) % kCapacity;
			mt += static_cast<double>(ts[k] - ts[head]) * 1e-6;
			ma += static_cast<double>(alt[k]);
		}
		mt /= static_cast<double>(count);
		ma /= static_cast<double>(count);
		double cov = 0.0, var = 0.0;
		for (size_t i = 0; i < count; ++i) {
			const size_t k = (head + i) % kCapacity;
			const double dt = static_cast<double>(ts[k] - ts[head]) * 1e-6 - mt;
			cov += dt * (static_cast<double>(alt[k]) - ma);
			var += dt * dt;
		}
		const double slope_mps = (var > 0.0) ? cov / var : 0.0;
		if (std::fabs(slope_mps) < kMaxSpeedMps) {
			if (still_since_us == 0) {
				still_since_us = newest;
			}
			detected = (newest - still_since_us) >= kConfirmUs;
		} else {
			still_since_us = 0;
			detected = false;
		}
	}
};

struct KalmanRuntime {
	app::EskfEstimator estimator;
	app::ApogeeHub apogee_hub;
	bool apogee_ready = false;
	bool apogee_detected = false;
	uint32_t liftoff_ms = 0;
	bool initialized = false;
	uint64_t last_imu_ts_us[4] = {0, 0, 0, 0};
	bool has_prev_imu_ts[4] = {false, false, false, false};
	flight_computer::State last_state = flight_computer::State::INIT;
	flight_computer::Vector3 last_body_accel_mps2{};
	KalmanHealthSnapshot health{};
	size_t active_imu_sources = 4u;
	static constexpr size_t kActiveBaroSources = 4u;
	LiftoffAccHold acc_hold;  ///< Liftoff acceleration-hold evaluator
	LiftoffDetector liftoff_detector;  ///< IMU dual-window liftoff detector
	uint32_t imu_liftoff_detect_ms = 0;  ///< Timestamp of the detecting sample
	TouchdownDetector touchdown_detector;  ///< Baro stillness in DESCENT
	bool touchdown_published = false;

#if KALMAN_DEBUG_PRINT
	kalman_debug::RawSensorSnapshot debug_raw_sensor_{};
#endif

#if KALMAN_DEBUG_FORCE_FLIGHT
	uint32_t force_flight_start_ms_ = 0;
	bool force_flight_done_ = false;
#endif

	void initIfNeeded() {
		if (initialized) {
			return;
		}

		estimator.reset();
		estimator.configureReplaySensorCounts(4, kActiveBaroSources);
		if (!apogee_ready) {
			const int idx = apogee_hub.addAlgorithm(app::createConsensusApogeeDetector());
			apogee_hub.setPrimary(idx);
			apogee_ready = true;
		}
		KalmanHealthStore::instance().reset();
		health = KalmanHealthSnapshot{};
		liftoff_ms = 0;
		apogee_detected = false;
		last_body_accel_mps2 = {};
		last_state = flight_computer::State::INIT;
		initialized = true;

#if KALMAN_DEBUG_PRINT
		// Print compile-time configuration at first init
		app_printf("[KAL-CFG] ESKF_FORCE_FLOAT32=%d  sizeof(eskf_scalar)=%lu  "
		       "COV_DECIM=%d  IMU_ODR=%d  BARO_ODR=%d\r\n",
		       ESKF_FORCE_FLOAT32,
		       static_cast<unsigned long>(sizeof(eskf_scalar)),
		       ESKF_COVARIANCE_DECIMATION,
		       ESKF_IMU_PRIMARY_ODR_HZ,
		       ESKF_BARO_ODR_HZ);
		app_printf("[KAL-CFG] activeBaroSources=%lu  "
		       "FORCE_FLIGHT=%d  PRINT_DECIM=%d  "
		       "FORCE_FLIGHT_DELAY_MS=%u  GNSS_FUSION=%d\r\n",
		       static_cast<unsigned long>(kActiveBaroSources),
		       KALMAN_DEBUG_FORCE_FLIGHT,
		       KALMAN_DEBUG_PRINT_DECIMATION,
		       (unsigned)KALMAN_DEBUG_FORCE_FLIGHT_DELAY_MS,
		       KALMAN_GNSS_FUSION_ENABLE);
#ifdef COMPILE_OPT_LEVEL
		app_printf("[KAL-CFG] Optimization: -O%d", COMPILE_OPT_LEVEL);
#elif defined(__OPTIMIZE__)
		app_printf("[KAL-CFG] Optimization: ON (unknown level)");
#else
		app_printf("[KAL-CFG] Optimization: OFF (-O0)");
#endif
		app_printf("  SYSCLK=%luMHz\r\n", SystemCoreClock / 1000000UL);
		app_timebase_print_init_diag();
		app_printf("[KAL-CFG] timebase_now: HAL=%lu  app=%lu  cycles_per_us=%lu\r\n",
		       static_cast<unsigned long>(HAL_GetTick()),
		       static_cast<unsigned long>(app_timebase_now_ms()),
		       SystemCoreClock / 1000000UL);
#endif
	}

	/// Reset every piece of Kalman runtime state that describes a flight
	/// sequence: estimator (incl. preflight tares, rail/flight shadows,
	/// ground references, turn-on bias, GNSS anchor, pending IMU groups),
	/// apogee hub, both liftoff detectors, IMU timestamp tracking and
	/// health counters. The FSM state seen by the runtime is unchanged.
	void resetRuntime() {
		estimator.reset();
		estimator.configureReplaySensorCounts(4u, kActiveBaroSources);
		active_imu_sources = 4u;
		apogee_hub.reset();
		for (size_t i = 0; i < 4u; ++i) {
			last_imu_ts_us[i] = 0;
			has_prev_imu_ts[i] = false;
		}
		liftoff_ms = 0;
		apogee_detected = false;
		last_body_accel_mps2 = {};
		acc_hold.reset();
		liftoff_detector.reset();
		imu_liftoff_detect_ms = 0;
		touchdown_detector.reset();
		touchdown_published = false;
		KalmanHealthStore::instance().reset();
	}

	void onStateChange(uint32_t raw_state) {
		initIfNeeded();
		if (raw_state > static_cast<uint32_t>(flight_computer::State::ABORT_IN_FLIGHT)) {
			return;
		}

		const auto state = static_cast<flight_computer::State>(raw_state);
		if (state == last_state) {
			return;
		}

		estimator.onFlightStateChange(state);
		last_state = state;

		if (state == flight_computer::State::INIT) {
			resetRuntime();
		}

		// Start liftoff acceleration-hold evaluation on IGNITION entry.
		if (state == flight_computer::State::IGNITION) {
			acc_hold.onIgnitionEnter(app_timebase_now_us());

			// A detection latched before IGNITION (e.g. the rocket was
			// bumped during FILLING) is not a launch. Discard it so only
			// detections after ignition can start the flight estimator.
			if (liftoff_detector.isDetected()) {
				app_printf("[LIFTOFF] Discarding pre-ignition IMU detection\r\n");
				liftoff_detector.reset();
				imu_liftoff_detect_ms = 0;
				auto &goat = flight_computer::GOATStore::get_instance();
				auto event = goat.eventStore.get();
				event.imu_liftoff_detected = false;
				goat.eventStore.set(event);
			}
		}
	}

	/// Kalman-side liftoff epoch. While the FSM is in IGNITION, the
	/// dual-window detector fires within tens of ms of motion onset, while
	/// the FSM may only declare BURN later (cable or end of the
	/// acceleration-hold window). The estimator freezes its preflight state
	/// at the liftoff epoch, so a late epoch degrades it; use the detection
	/// time instead. The FSM remains the authority on flight states, and its
	/// later BURN is then ignored by onLiftoff().
	void selfLatchLiftoffIfDetected() {
		if (liftoff_ms != 0 ||
			last_state != flight_computer::State::IGNITION ||
			!liftoff_detector.isDetected() || imu_liftoff_detect_ms == 0) {
			return;
		}
		app_printf("[LIFTOFF] Kalman liftoff epoch from IMU detection: t=%lu ms\r\n",
		       static_cast<unsigned long>(imu_liftoff_detect_ms));
		onLiftoff(imu_liftoff_detect_ms);
	}

	void onLiftoff(uint32_t liftoff_ms) {
		initIfNeeded();
		if (this->liftoff_ms != 0) {
			// Already in flight from an earlier (IMU) epoch: keep it.
			app_printf("[LIFTOFF] FSM liftoff at t=%lu ms ignored, epoch already t=%lu ms\r\n",
			       static_cast<unsigned long>(liftoff_ms),
			       static_cast<unsigned long>(this->liftoff_ms));
			return;
		}
		this->liftoff_ms = liftoff_ms;
		estimator.onLiftoff(liftoff_ms);
		apogee_hub.arm(liftoff_ms);
		apogee_detected = false;
	}

	void setActiveImuSources(size_t imu_sources) {
		initIfNeeded();
		if (imu_sources == active_imu_sources) {
			return;
		}
		estimator.configureReplaySensorCounts(imu_sources, kActiveBaroSources);
		active_imu_sources = imu_sources;
	}

	static app::ImuSample convertImuSample(const IMUData &sample) {
		// IMUData already contains float SI values (m/s², rad/s, Kelvin).
		// Pass them through directly to ImuSample; the estimator consumes
		// these via the APP_IMU_LOG_FORMAT == 1 (float) path, avoiding a
		// lossy reverse-scale/re-scale round-trip.
		constexpr float kTempScale = 1.0f / 2.07f;
		constexpr float kTempOffsetK = 25.0f + 273.15f;

		app::ImuSample out{};
		out.ax = sample.accel_x;
		out.ay = sample.accel_y;
		out.az = sample.accel_z;
		out.gx = sample.gyro_x;
		out.gy = sample.gyro_y;
		out.gz = sample.gyro_z;

		// Temperature: ICM FIFO convention is stored as int8_t in ImuSample
		// to match the kalman library's expected format.
		const float temp_raw = (sample.temperature - kTempOffsetK) / kTempScale;
		const int temp_i = static_cast<int>(std::lround(temp_raw));
		const int temp_clamped = std::max(-128, std::min(127, temp_i));
		out.temperature = static_cast<int8_t>(temp_clamped);

		out.internal_timestamp = static_cast<uint16_t>(sample.timestamp_us & 0xFFFFU);
		return out;
	}

	static app::sensors::gnss::GnssSample convertGpsFixSample(
		const GpsBasicFixData &fix,
		uint64_t fallback_timestamp_us) {
		app::sensors::gnss::GnssSample sample{};

		sample.timestamp_us =
			(fix.timestamp_us > 0u) ? fix.timestamp_us : fallback_timestamp_us;
		sample.pps_timestamp_us = fix.pps_timestamp_us;
		sample.lat_deg7 = fix.lat;
		sample.lon_deg7 = fix.lon;
		sample.alt_msl_mm = fix.hMSL;
		sample.alt_ellipsoid_mm = fix.height;
		sample.vel_n_mms = fix.velN;
		sample.vel_e_mms = fix.velE;
		sample.vel_d_mms = fix.velD;
		sample.ground_speed_mms = fix.gSpeed;
		sample.heading_deg5 = fix.headMot;
		sample.h_acc_mm = fix.hAcc;
		sample.v_acc_mm = fix.vAcc;
		sample.s_acc_mms = fix.sAcc;
		sample.head_acc_deg5 = fix.headAcc;
		sample.pdop = fix.pDOP;
		sample.fix_type = static_cast<uint8_t>(fix.fixType);
		sample.num_sv = fix.numSV;
		sample.flags = 0;
		if (fix.flags.gnssFixOK) {
			sample.flags |= 0x01u;
		}
		if (fix.flags.diffSoln) {
			sample.flags |= 0x02u;
		}

		sample.year = fix.year;
		sample.month = fix.month;
		sample.day = fix.day;
		sample.hour = fix.hour;
		sample.min = fix.min;
		sample.sec = fix.sec;
		sample.nano = fix.nano;
		sample.time_valid = 0;
		if (fix.valid.validDate) {
			sample.time_valid |= 0x01u;
		}
		if (fix.valid.validTime) {
			sample.time_valid |= 0x02u;
		}
		if (fix.valid.fullyResolved) {
			sample.time_valid |= 0x04u;
		}
		if (fix.valid.validMag) {
			sample.time_valid |= 0x08u;
		}
		sample.itow_ms = fix.iTOW;

		sample.valid =
			fix.flags.gnssFixOK &&
			(sample.fix_type >= static_cast<uint8_t>(GpsFixType::FIX_2D));
		return sample;
	}

	static app::BaroSample convertBaroSample(const BaroData &sample,
											 size_t source_index) {
		app::BaroSample out{};
		out.pressurePa = sample.pressure_pa;
		out.temperatureC = sample.temperature_c;
		out.timestamp_us = sample.timestamp_us;
		out.source = static_cast<uint8_t>(source_index);
		return out;
	}

	void ingestImuChunk(size_t source_index, const IMUData *samples, size_t count) {
		initIfNeeded();
		if (samples == nullptr || count == 0u) {
			return;
		}

#if KALMAN_DEBUG_PRINT
		{
			static uint32_t ingest_diag_counter = 0;
			if (ingest_diag_counter < 5) {
				app_printf("[INGEST-IMU] src=%u count=%u ts[0]=%u:%u",
					(unsigned)source_index, (unsigned)count,
					(unsigned)(samples[0].timestamp_us >> 32),
					(unsigned)samples[0].timestamp_us);
				if (count >= 2) {
					app_printf("  ts[1]=%u:%u  dt=%u",
						(unsigned)(samples[1].timestamp_us >> 32),
						(unsigned)samples[1].timestamp_us,
						(unsigned)(uint32_t)(samples[1].timestamp_us - samples[0].timestamp_us));
				}
				app_printf("  ts[last]=%u:%u\r\n",
					(unsigned)(samples[count-1].timestamp_us >> 32),
					(unsigned)samples[count-1].timestamp_us);
				ingest_diag_counter++;
			}
		}
#endif

		if (count > kMaxImuSamplesPerSourcePerRun) {
			count = kMaxImuSamplesPerSourcePerRun;
		}

		size_t offset = 0u;
		while (offset < count) {
			const size_t slice_count =
				std::min(kMaxImuSamplesPerEstimatorBatch, count - offset);
			const IMUData *slice = &samples[offset];

			const bool has_prev_ts = has_prev_imu_ts[source_index];
			const uint64_t prev_ts = last_imu_ts_us[source_index];
			const uint64_t first_ts = slice[0].timestamp_us;
			const uint64_t last_ts = slice[slice_count - 1u].timestamp_us;
			last_imu_ts_us[source_index] = last_ts;
			has_prev_imu_ts[source_index] = true;

			app::ImuSample converted[kMaxImuSamplesPerEstimatorBatch] = {};
			for (size_t i = 0; i < slice_count; ++i) {
				converted[i] = convertImuSample(slice[i]);
			}

			app::ImuBatch batch{};
			batch.data = converted;
			batch.count = slice_count;
			batch.t0_us = first_ts;
			if (slice_count >= 2u && slice[1].timestamp_us > first_ts) {
				batch.dt_us =
					static_cast<uint32_t>(slice[1].timestamp_us - first_ts);
			} else {
				batch.dt_us =
					(has_prev_ts && first_ts > prev_ts)
						? static_cast<uint32_t>(first_ts - prev_ts)
						: kNominalImuDtUs;
			}
			batch.source = static_cast<uint8_t>(source_index);
			batch.slot = 0xFF;

#if KALMAN_DEBUG_PRINT
			{
				static uint32_t batch_diag_counter = 0;
				if (batch_diag_counter < 10) {
					app_printf("[BATCH-DT] #%u  t0=%u:%u  dt_us=%u  count=%u  "
						"slice0_ts=%u:%u  slice1_ts=%u:%u\r\n",
						(unsigned)batch_diag_counter,
						(unsigned)(batch.t0_us >> 32), (unsigned)batch.t0_us,
						(unsigned)batch.dt_us, (unsigned)batch.count,
						(unsigned)(slice[0].timestamp_us >> 32),
						(unsigned)slice[0].timestamp_us,
						(unsigned)(slice_count >= 2 ? (slice[1].timestamp_us >> 32) : 0),
						(unsigned)(slice_count >= 2 ? (uint32_t)slice[1].timestamp_us : 0));
					batch_diag_counter++;
				}
			}
#endif

			estimator.processImuBatch(batch);

			// ── Feed liftoff detector with each sample in the slice ─────
			if (!liftoff_detector.isDetected() && g_liftoff_detection_allowed) {
				const bool gate = estimator.railShadow().isGateOpen();
				const bool was_armed = liftoff_detector.isArmed();
				for (size_t i = 0; i < slice_count; ++i) {
					liftoff_detector.update(
						slice[i].accel_x,
						slice[i].accel_y,
						slice[i].accel_z,
						gate);
					if (liftoff_detector.isDetected()) {
						imu_liftoff_detect_ms = static_cast<uint32_t>(
							slice[i].timestamp_us / 1000ULL);
						break;
					}
				}
				if (!was_armed && liftoff_detector.isArmed()) {
					app_printf("[LIFTOFF] Detector ARMED (pad stable for ~2s)\r\n");
				}
				if (liftoff_detector.isDetected()) {
					app_printf("[LIFTOFF] IMU liftoff DETECTED! t=%lu ms\r\n",
					       static_cast<unsigned long>(imu_liftoff_detect_ms));
				}
			}

			// Raw samples rotated by the fixed mounting matrix only (no
			// estimator preprocessing), body +X = nose.
			const eskf_scalar (*to_body)[3] = eskf::getImuSensorToBody(source_index);
			flight_computer::Vector3 body_accel{};
			for (size_t i = 0; i < slice_count; ++i) {
				const float s[3] = {slice[i].accel_x, slice[i].accel_y, slice[i].accel_z};
				float b[3];
				for (int r = 0; r < 3; ++r) {
					b[r] = static_cast<float>(to_body[r][0] * s[0] +
					                          to_body[r][1] * s[1] +
					                          to_body[r][2] * s[2]);
				}
				acc_hold.addSample(source_index, b[0], slice[i].timestamp_us);
				body_accel.x = b[0];
				body_accel.y = b[1];
				body_accel.z = b[2];
			}
			last_body_accel_mps2 = body_accel;
			health.imu_samples_consumed += static_cast<uint32_t>(slice_count);

#if KALMAN_DEBUG_PRINT
			// Capture last raw IMU sample for debug output
			const auto& last_raw = slice[slice_count - 1u];
			debug_raw_sensor_.ax = last_raw.accel_x;
			debug_raw_sensor_.ay = last_raw.accel_y;
			debug_raw_sensor_.az = last_raw.accel_z;
			debug_raw_sensor_.gx = last_raw.gyro_x;
			debug_raw_sensor_.gy = last_raw.gyro_y;
			debug_raw_sensor_.gz = last_raw.gyro_z;
			// Per-IMU accel magnitude and raw axes
			if (source_index < 4) {
				const float amag = std::sqrt(
					last_raw.accel_x * last_raw.accel_x +
					last_raw.accel_y * last_raw.accel_y +
					last_raw.accel_z * last_raw.accel_z);
				debug_raw_sensor_.imu_per_sensor_amag[source_index] = amag;
				debug_raw_sensor_.imu_per_sensor_ax[source_index] = last_raw.accel_x;
				debug_raw_sensor_.imu_per_sensor_ay[source_index] = last_raw.accel_y;
				debug_raw_sensor_.imu_per_sensor_az[source_index] = last_raw.accel_z;
				debug_raw_sensor_.imu_per_sensor_alive |= (1u << source_index);
			}
#endif
			offset += slice_count;
		}
	}

	void onTick(uint64_t now_us) {
		initIfNeeded();
		estimator.onTick(now_us);

		const app::EstimatorOutput output = estimator.output();
		const bool eskf_diverged = estimator.isEskfDiverged();
		const bool is_coast_phase = estimator.isCoastPhase();
		auto &goat = flight_computer::GOATStore::get_instance();
		flight_computer::bmp3_data latest_baro{};

//		if (navigationDataMutexHandle != nullptr) {
//			osMutexAcquire(navigationDataMutexHandle, osWaitForever);
//		}

		latest_baro = goat.navigationDataStore.get_baro();

		const auto nav = app::mapEstimatorToNavigation(
			output,
			latest_baro,
			last_body_accel_mps2);
		goat.navigationDataStore.set(nav);

//		if (navigationDataMutexHandle != nullptr) {
//			osMutexRelease(navigationDataMutexHandle);
//		}

		// --- Liftoff acceleration-hold evaluation ---
		// Samples are fed in ingestImuChunk(). When the evaluation window
		// completes, write the result to eventStore.
		flight_computer::AccHoldStatus acc_hold_result =
			flight_computer::ACC_HOLD_NOT_ELAPSED;
		bool acc_hold_terminal = false;
		if (acc_hold.active) {
			acc_hold_result = acc_hold.evaluate(now_us);
			acc_hold_terminal =
				(acc_hold_result != flight_computer::ACC_HOLD_NOT_ELAPSED);
		}

		const bool set_catastrophic_failure = false;  // ESKF divergence is NOT catastrophic;
		                                               // apogee detector gracefully falls back to shadow filter.
		bool set_apogee_detected = false;

		if (!apogee_detected && liftoff_ms > 0) {
			const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000ULL);
			const app::ApogeeInput input = app::buildApogeeInput(
				output,
				eskf_diverged,
				is_coast_phase,
				liftoff_ms,
				now_ms);

			const app::ApogeeDecision decision = apogee_hub.update(now_ms, input);
			if (decision.triggered) {
				apogee_detected = true;
				set_apogee_detected = true;
				app_printf("[APOGEE] Detected at t=%lu ms (liftoff+%lu ms)\r\n",
				       (unsigned long)now_ms,
				       (unsigned long)(now_ms - liftoff_ms));
			}
		}

		// TODO(parachute-trigger): Apogee detection is the Kalman subsystem's
		// responsibility and is now complete (ConsensusApogeeDetector + ApogeeHub).
		// The actual parachute deployment decision and FSM ASCENT->DESCENT
		// transition should be handled by a higher-level module that:
		//   1. Reads eventStore.apogee_detected (set above by kalman).
		//   2. Applies additional robustness checks before triggering deployment:
		//      - Minimum time-since-liftoff confirmation window.
		//      - Independent altitude/velocity sanity bound.
		//      - Redundant actuation confirmation (e.g. arm/fire sequencing).
		//   3. Only then transitions FSM to DESCENT and fires actuation.
		// Currently, av_state.cpp::fromAscent() reads dump.event.apogee_detected
		// and transitions to DESCENT. The robustness checks listed above are
		// NOT yet implemented there — this is a flight-safety TODO.

		// ── Touchdown detector → EventStore (live flag, DESCENT/LANDED) ──
		if (last_state == flight_computer::State::DESCENT ||
			last_state == flight_computer::State::LANDED) {
			float baro_alt_m = 0.0f;
			uint64_t baro_ts_us = 0u;
			if (estimator.latestBaroAltitude(baro_alt_m, baro_ts_us)) {
				touchdown_detector.addSample(baro_ts_us, baro_alt_m);
			}
			touchdown_detector.update(now_us);
		} else {
			touchdown_detector.reset();
		}
		if (touchdown_detector.detected != touchdown_published) {
			touchdown_published = touchdown_detector.detected;
			auto event = goat.eventStore.get();
			event.touchdown_detected = touchdown_published;
			goat.eventStore.set(event);
			app_printf("[TOUCHDOWN] %s at t=%lu ms\r\n",
			       touchdown_published ? "Detected" : "Cleared",
			       static_cast<unsigned long>(now_us / 1000ULL));
		}

		// ── IMU liftoff detector → EventStore ────────────────────────
		const bool imu_liftoff = liftoff_detector.isDetected();

		if (set_catastrophic_failure || set_apogee_detected || acc_hold_terminal || imu_liftoff) {

			auto event = goat.eventStore.get();
			if (set_catastrophic_failure) {
				event.catastrophic_failure = true;
			}
			if (set_apogee_detected) {
				event.apogee_detected = true;
			}
			if (acc_hold_terminal) {
				event.vertical_acc_hold = static_cast<uint8_t>(acc_hold_result);
			}
			if (imu_liftoff) {
				event.imu_liftoff_detected = true;
			}
			goat.eventStore.set(event);

//			if (eventStoreMutexHandle != nullptr) {
//				osMutexRelease(eventStoreMutexHandle);
//			}
		}

		health.diverged = eskf_diverged;
		health.altitude_valid = output.altitude_valid;
		health.velocity_valid = output.velocity_valid;
		health.yieldable_imu_drops = estimator.rewindStats().imu_drops;
		health.catchup_yield_count = estimator.catchupYieldCount();
		health.catchup_budget_yields = estimator.rewindStats().catchup_budget_yields;
		health.total_events_processed = static_cast<uint32_t>(estimator.totalCatchupEventsProcessed());
		health.baro_corrections = estimator.rewindStats().baro_corrections;
		// Compute ESKF lag: wall clock minus ESKF internal timestamp
		const uint64_t now = app_timebase_now_us();
		const uint64_t kal_ts = estimator.kalmanTimestamp();
		health.kalman_behind_us = (now > kal_ts) ? static_cast<uint32_t>(now - kal_ts) : 0;
	}

	void ingestAidingFromStore() {
		const uint32_t _primask = __get_PRIMASK();
		__disable_irq();
		const uint64_t baro_trigger_us = g_pending_baro_trigger_us;
		g_pending_baro_trigger_us = 0u;
		if ((_primask & 0x1u) == 0u) __enable_irq();
		if (baro_trigger_us > 0u) {
			estimator.onBaroTrigger(baro_trigger_us);
		}

		RingBuffer<BaroData, 100> *baro_buffers[] = {
			&baroData1, &baroData2, &baroData3, &baroData4};
		BaroData staged_baros[kActiveBaroSources][kMaxBaroSamplesPerSourcePerRun] = {};
		size_t staged_baro_count[kActiveBaroSources] = {};
		size_t staged_baro_index[kActiveBaroSources] = {};

		for (size_t source = 0; source < kActiveBaroSources; ++source) {
			while (staged_baro_count[source] < kMaxBaroSamplesPerSourcePerRun &&
				   baro_buffers[source]->pop(
					   staged_baros[source][staged_baro_count[source]])) {
				++staged_baro_count[source];
			}
		}

		for (;;) {
			size_t best_source = kActiveBaroSources;
			uint64_t best_ts = UINT64_MAX;
			for (size_t source = 0; source < kActiveBaroSources; ++source) {
				if (staged_baro_index[source] >= staged_baro_count[source]) {
					continue;
				}
				const uint64_t ts =
					staged_baros[source][staged_baro_index[source]].timestamp_us;
				if (ts < best_ts) {
					best_ts = ts;
					best_source = source;
				}
			}
			if (best_source >= kActiveBaroSources) {
				break;
			}

			const app::BaroSample sample = convertBaroSample(
				staged_baros[best_source][staged_baro_index[best_source]],
				best_source);
			estimator.processBaroSample(sample);
			health.baro_updates += 1;
			++staged_baro_index[best_source];

#if KALMAN_DEBUG_PRINT
			// Capture last baro sample for debug output
			debug_raw_sensor_.baro_pa = sample.pressurePa;
			debug_raw_sensor_.baro_tempC = sample.temperatureC;
			// Per-sensor tracking
			if (best_source < 4) {
				debug_raw_sensor_.baro_per_sensor_pa[best_source] = sample.pressurePa;
				debug_raw_sensor_.baro_per_sensor_tempC[best_source] = sample.temperatureC;
				debug_raw_sensor_.baro_per_sensor_alive |= (1u << best_source);
			}
#endif
		}

		GpsBasicFixData staged_fixes[kMaxGpsSamplesPerRun] = {};
		size_t fix_count = 0;

		while (fix_count < kMaxGpsSamplesPerRun &&
			   gpsData.pop(staged_fixes[fix_count])) {
			++fix_count;
		}

		const uint64_t fallback_timestamp_us =
			app_timebase_now_us();
		for (size_t i = 0; i < fix_count; ++i) {
			const app::sensors::gnss::GnssSample sample =
				convertGpsFixSample(staged_fixes[i], fallback_timestamp_us);
#if KALMAN_GNSS_FUSION_ENABLE
			// Forward ALL fixes to the estimator (including invalid ones) so
			// the estimator's own stale/frozen detection and fix-drop
			// diagnostics can observe the full GNSS health picture.
			// The estimator has its own usability gate:
			//   (sample.valid && sample.fix_type >= 2).
			estimator.processGpsSample(sample);
#endif
			if (sample.valid) {
				health.gps_updates += 1;
			}
		}
	}
};

KalmanRuntime &runtime() {
	static KalmanRuntime instance;
	return instance;
}

} // namespace

int kalman_loop() {
	KalmanRuntime &kalman = runtime();
	kalman.initIfNeeded();
	const uint64_t kalman_loop_start_us = app_timebase_now_us();

#if KALMAN_DEBUG_PRINT
	uint64_t t_imu_drain_start = kalman_loop_start_us;
#endif

	const uint32_t current_state = kalman_current_state();
	kalman.onStateChange(current_state);

	if (g_reset_requested.exchange(false)) {
		if (kalmanResetAllowedIn(current_state)) {
			kalman.resetRuntime();
			kalman_lifecycle_rearm();
			app_printf("[KAL] Reset done (state=%lu)\r\n",
			       static_cast<unsigned long>(current_state));
		} else {
			app_printf("[KAL] Reset refused (state=%lu)\r\n",
			       static_cast<unsigned long>(current_state));
		}
	}

	AppImuRingBuffer *buffers[] = {&imuData1, &imuData2, &imuData3, &imuData4};

	bool source_healthy[4] = {false, false, false, false};
	size_t healthy_source_count = 0;
	int drained = 0;
	int align_discarded = 0;
	for (size_t i = 0; i < 4; ++i) {
		source_healthy[i] =
			(app_imu_sensor_healthy(static_cast<uint8_t>(i)) != 0U);
		if (source_healthy[i]) {
			++healthy_source_count;
		}

		const uint32_t ring_depth = static_cast<uint32_t>(buffers[i]->size());
		kalman.health.imu_ring_hwm[i] =
			std::max(kalman.health.imu_ring_hwm[i], ring_depth);
	}
	kalman.setActiveImuSources(healthy_source_count);

#if KALMAN_DEBUG_PRINT
	const uint64_t t_imu_drain_end = app_timebase_now_us();
#endif

	/* ── Aligned round-robin drain ──────────────────────────────────
	 *
	 *  Problem: ring buffers can be deeply unbalanced (overflow discards
	 *  oldest samples unevenly). Source 1 may have data 17ms ahead of
	 *  source 0, breaking the 500μs sync tolerance in processImuBatch.
	 *
	 *  Solution:
	 *   1. Align all rings to the NEWEST oldest-sample within 200μs
	 *      (well inside the 500μs sync tolerance in processImuBatch).
	 *   2. Round-robin drain 1 sample per source per iteration.
	 *   3. Stop when ANY healthy ring empties (prevents partial groups).
	 *      Leftover samples in longer rings survive to next tick.
	 * ─────────────────────────────────────────────────────────────── */

	// Step 1: Find the latest front timestamp across all healthy rings.
	uint64_t align_ts = 0;
	for (size_t i = 0; i < 4; ++i) {
		if (!source_healthy[i]) continue;
		const IMUData *front = buffers[i]->get(0);
		if (front && front->timestamp_us > align_ts) {
			align_ts = front->timestamp_us;
		}
	}

	// Step 2: Discard old samples to align all rings.
	//         The inter-IMU timestamp offset from sequential SPI reads
	//         can be 100-170μs (varies by boot). Since this overlaps the
	//         156μs FSYNC period, a threshold < period cannot reliably
	//         distinguish same-edge from adjacent-edge. Use 200μs:
	//         empirically gives staleSkip=0, soloFlush=0 in steady state.
	static constexpr uint64_t kDrainAlignToleranceUs = 200;
	if (align_ts > 0) {
		const uint64_t align_floor = (align_ts > kDrainAlignToleranceUs) ? (align_ts - kDrainAlignToleranceUs) : 0u;
		for (size_t i = 0; i < 4; ++i) {
			if (!source_healthy[i]) continue;
			IMUData discard;
			while (buffers[i]->size() > 0) {
				const IMUData *front = buffers[i]->get(0);
				if (!front || front->timestamp_us >= align_floor) break;
				buffers[i]->pop(discard);
				drained++;
				align_discarded++;
			}
		}
		// If alignment discarded data, stale pending in the estimator
		// might reference old timestamps. Reset to prevent poisoning.
		if (align_discarded > 0) {
			kalman.estimator.resetPendingImuGroup();
		}
	}

	// Step 3: Round-robin drain — stop when ANY healthy ring empties.
	// TODO: guard added because healthy_source_count can be 0 on the first
	// tick(s) after boot (no IMU has produced a frame yet), which spun this
	// loop forever with the old unguarded for(;;). Not present upstream on
	// fix/fc-flight-test-plume since that branch never hit the race.

	//for (;;) {
	while (healthy_source_count > 0) {
		// Check all healthy sources still have data.
		bool all_have_data = true;
		for (size_t i = 0; i < 4; ++i) {
			if (!source_healthy[i]) continue;
			if (buffers[i]->size() == 0) { all_have_data = false; break; }
		}
		if (!all_have_data) break;

		// Pop one sample from each healthy source.
		for (size_t i = 0; i < 4; ++i) {
			if (!source_healthy[i]) continue;
			IMUData sample;
			buffers[i]->pop(sample);
			kalman.ingestImuChunk(i, &sample, 1);
			drained++;
		}
	}

#if KALMAN_DEBUG_PRINT
	const uint64_t t_imu_process_end = app_timebase_now_us();
#endif

	// Liftoff must be consumed before aiding ingestion so that GPS
	// samples arriving in the same tick see in_flight_==true and
	// can anchor the NED origin immediately, matching ktp-soft ordering.
	kalman.selfLatchLiftoffIfDetected();
	uint32_t liftoff_ms = 0;
	if (kalman_take_pending_liftoff(&liftoff_ms) != 0U) {
		kalman.onLiftoff(liftoff_ms);
	}

	kalman.ingestAidingFromStore();

#if KALMAN_DEBUG_PRINT
	const uint64_t t_aiding_end = app_timebase_now_us();
#endif

	// Use wall-clock time for onTick so the catchUp horizon matches real
	// time, not the latest IMU sample timestamp (which may lag due to
	// central-diff delay or empty batches).
	const uint64_t tick_now_us = app_timebase_now_us();
	kalman.onTick(tick_now_us);

#if KALMAN_DEBUG_PRINT
	const uint64_t t_tick_end = app_timebase_now_us();
#endif

	const uint64_t kalman_loop_end_us = app_timebase_now_us();
	const uint32_t kalman_loop_elapsed_us =
		static_cast<uint32_t>(kalman_loop_end_us - kalman_loop_start_us);
	kalman.health.last_kalman_loop_us = kalman_loop_elapsed_us;
	kalman.health.max_kalman_loop_us =
		std::max(kalman.health.max_kalman_loop_us, kalman_loop_elapsed_us);
	// Pull the latest main-loop wall-clock values (published by
	// kalman_note_main_loop_iteration_us on the previous super-loop iteration)
	// into the snapshot so readers always observe a consistent record.
	kalman.health.last_main_loop_iteration_us =
		g_last_main_loop_iteration_us.load(std::memory_order_relaxed);
	kalman.health.max_main_loop_iteration_us =
		g_max_main_loop_iteration_us.load(std::memory_order_relaxed);

	KalmanHealthStore::instance().set(kalman.health);

	// -----------------------------------------------------------------
	// KALMAN_DEBUG_FORCE_FLIGHT: auto-transition to BURN for bench test.
	// After a configurable settling period the force-flight logic
	// synthetically pushes the Kalman lifecycle through the same path
	// as a normal INIT → BURN transition, so the ESKF initialises from
	// the Rail Shadow and begins tracking attitude.
	// -----------------------------------------------------------------
#if KALMAN_DEBUG_FORCE_FLIGHT
	if (!kalman.force_flight_done_) {
		// Use wall-clock milliseconds (HAL_GetTick) instead of loop-iteration
		// count.  kalman_loop() runs at the super-loop rate (~18 kHz), so a
		// tick-based delay of 10 000 would be only ~556 ms — too short for
		// the 1-second ground-reference window to complete.
		const uint32_t now_ms = HAL_GetTick();
		if (kalman.force_flight_start_ms_ == 0) {
			kalman.force_flight_start_ms_ = now_ms;
		}
		if ((now_ms - kalman.force_flight_start_ms_) >= KALMAN_DEBUG_FORCE_FLIGHT_DELAY_MS) {
			app_printf("[KAL-DBG] Force-flight: triggering liftoff via FSM  "
			       "(waited %lu ms, HAL_ms=%lu)\r\n",
			       (unsigned long)(now_ms - kalman.force_flight_start_ms_),
			       (unsigned long)HAL_GetTick());
			// Route through the FSM instead of bypassing it.
			// The FSM will call kalman_on_state_change(BURN) and
			// kalman_on_liftoff() on the next tick.
			g_uart_force_liftoff = true;
			kalman.force_flight_done_ = true;
		}
	}
#endif

	// -----------------------------------------------------------------
	// KALMAN_DEBUG_PRINT: human-readable console diagnostics.
	// -----------------------------------------------------------------
#if KALMAN_DEBUG_PRINT
	// Populate IMU driver diagnostics for debug output
	app_imu_frame_counts(
		&kalman.debug_raw_sensor_.frame_h78,
		&kalman.debug_raw_sensor_.frame_hF0,
		&kalman.debug_raw_sensor_.frame_other);
	app_imu_ts_diagnostics(
		&kalman.debug_raw_sensor_.frame_h7C,
		&kalman.debug_raw_sensor_.imu_monotonic_repairs,
		&kalman.debug_raw_sensor_.imu_last_offset_err,
		&kalman.debug_raw_sensor_.imu_offset_reject_count,
		&kalman.debug_raw_sensor_.imu_max_rejected_err,
		&kalman.debug_raw_sensor_.imu_spi_fifo_fail,
		&kalman.debug_raw_sensor_.imu_spi_not_ready,
		&kalman.debug_raw_sensor_.imu_offset_burst_count,
		&kalman.debug_raw_sensor_.imu_gate_armed);
	kalman.debug_raw_sensor_.imu_status_flags =
		app_imu_sensor_status_flags(0);
	// Use the health snapshot's yieldable_imu_drops as it's already tracked
	kalman.debug_raw_sensor_.imu_drop_count =
		kalman.health.yieldable_imu_drops;

	// Populate timing breakdown
	const uint64_t t_output_end = app_timebase_now_us();
	kalman.debug_raw_sensor_.t_imu_drain_us =
		static_cast<uint32_t>(t_imu_drain_end - t_imu_drain_start);
	kalman.debug_raw_sensor_.t_imu_process_us =
		static_cast<uint32_t>(t_imu_process_end - t_imu_drain_end);
	kalman.debug_raw_sensor_.t_aiding_us =
		static_cast<uint32_t>(t_aiding_end - t_imu_process_end);
	kalman.debug_raw_sensor_.t_tick_us =
		static_cast<uint32_t>(t_tick_end - t_aiding_end);
	kalman.debug_raw_sensor_.t_output_us =
		static_cast<uint32_t>(t_output_end - t_tick_end);

	kalman_debug::printDebugLine(
		kalman.estimator,
		kalman.health,
		kalman.last_state,
		kalman_loop_elapsed_us,
		static_cast<uint32_t>(drained),
		static_cast<uint32_t>(align_discarded),
		kalman.debug_raw_sensor_);
#endif

	return drained;
}

uint8_t kalman_request_reset(void) {
	if (!kalmanResetAllowedIn(kalman_current_state())) {
		return 0u;
	}
	g_reset_requested.store(true);
	return 1u;
}

void kalman_note_main_loop_iteration_us(uint32_t iteration_us) {
	g_last_main_loop_iteration_us.store(iteration_us, std::memory_order_relaxed);

	uint32_t observed_max =
		g_max_main_loop_iteration_us.load(std::memory_order_relaxed);
	while (iteration_us > observed_max &&
		   !g_max_main_loop_iteration_us.compare_exchange_weak(
			   observed_max,
			   iteration_us,
			   std::memory_order_relaxed,
			   std::memory_order_relaxed)) {
	}
	// The next kalman_loop iteration folds these atomics into KalmanHealthStore.
	// We deliberately avoid a read-modify-write on the store here to keep a
	// single writer path (kalman_loop) and prevent momentary inconsistencies.
}

void kalman_note_baro_trigger(uint64_t trigger_us) {
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	g_pending_baro_trigger_us = trigger_us;
	if ((primask & 0x1u) == 0u) __enable_irq();
}

void kalman_get_group_stats(uint32_t* fire_count, uint32_t* solo_flush,
                            uint32_t* stale_flush) {
	auto& rt = runtime();
	*fire_count  = rt.estimator.imu_group_fire_count_;
	*solo_flush  = rt.estimator.imu_solo_flush_count_;
	*stale_flush = rt.estimator.imu_stale_flush_count_;
}
