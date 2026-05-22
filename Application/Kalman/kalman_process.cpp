
#include "Core/Inc/main.h"
// #include "cmsis_os.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Kalman/AppLayer/apogee_hub.hpp"
#include "Application/Kalman/AppLayer/apogee_factory.hpp"
#include "Application/Kalman/AppLayer/hw_config.hpp"
#include "Application/Kalman/AppLayer/output_bridge.hpp"
#include "Application/Kalman/kalman_health.hpp"
#include "Application/Kalman/kalman_debug.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/Data/data.hpp"
#include "Application/FlightControl/threshold.h"
#include "Application/FlightControl/liftoff_detector.hpp"
#include "Application/Modules/baro_module.hpp"
#include "Application/Modules/imu_modlue.hpp"

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Application/app_timebase.h"
#include "Application/main.h"
#include "Drivers/InvIMU/InvIMU.h"
#include "Application/FlightControl/uart_cmd.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

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
#if APP_IMU_PRIMARY_ODR_HZ > 0
constexpr uint32_t kNominalImuDtUs =
	static_cast<uint32_t>(1000000ULL / APP_IMU_PRIMARY_ODR_HZ);
#else
constexpr uint32_t kNominalImuDtUs = 1000U;
#endif

std::atomic<uint32_t> g_last_main_loop_iteration_us{0u};
std::atomic<uint32_t> g_max_main_loop_iteration_us{0u};
volatile uint64_t g_pending_baro_trigger_us = 0u;

// ---------------------------------------------------------------
// Liftoff acceleration-hold evaluator.
//
// After the FSM enters IGNITION, this waits RAMP_UP_DURATION seconds
// (motor spin-up), then evaluates whether the mean vertical
// acceleration exceeds ACCEL_LIFTOFF for at least
// ACCEL_LIFTOFF_DURATION_MS milliseconds.  The result is written
// as a tri-state AccHoldStatus into the GOATStore eventStore so
// that the FSM can decide between BURN and ABORT_ON_GROUND.
// ---------------------------------------------------------------
struct LiftoffAccHold {
	bool active = false;         ///< Evaluation in progress
	uint64_t ignition_enter_us = 0;  ///< Timestamp when IGNITION was entered

	// Accumulator for the evaluation window.
	double accel_sum = 0.0;
	uint32_t accel_count = 0;
	uint64_t window_start_us = 0;  ///< Timestamp when the evaluation window began
	bool in_window = false;        ///< True once ramp-up has elapsed

	void reset() {
		active = false;
		ignition_enter_us = 0;
		accel_sum = 0.0;
		accel_count = 0;
		window_start_us = 0;
		in_window = false;
	}

	void onIgnitionEnter(uint64_t now_us) {
		reset();
		active = true;
		ignition_enter_us = now_us;
	}

	/// Call every Kalman tick while in IGNITION.
	/// @param vertical_accel_mps2  Kalman-filtered vertical acceleration
	///                             (positive = upward / away from ground).
	/// @param now_us               Current wall-clock time in microseconds.
	/// @return ACC_HOLD_NOT_ELAPSED while still evaluating,
	///         ACC_HOLD_DID_HOLD / ACC_HOLD_DID_NOT_HOLD when done.
	flight_computer::AccHoldStatus evaluate(
		double vertical_accel_mps2, uint64_t now_us)
	{
		using flight_computer::AccHoldStatus;
		using flight_computer::ACC_HOLD_NOT_ELAPSED;
		using flight_computer::ACC_HOLD_DID_HOLD;
		using flight_computer::ACC_HOLD_DID_NOT_HOLD;

		if (!active) {
			return ACC_HOLD_NOT_ELAPSED;
		}

		constexpr uint64_t ramp_up_us =
			static_cast<uint64_t>(RAMP_UP_DURATION * 1000000.0f);
		constexpr uint64_t window_us =
			static_cast<uint64_t>(ACCEL_LIFTOFF_DURATION_MS * 1000.0f);

		// Phase 1: wait for motor ramp-up to complete.
		if (!in_window) {
			if ((now_us - ignition_enter_us) < ramp_up_us) {
				return ACC_HOLD_NOT_ELAPSED;
			}
			in_window = true;
			window_start_us = now_us;
			accel_sum = 0.0;
			accel_count = 0;
		}

		// Phase 2: accumulate vertical acceleration samples.
		accel_sum += vertical_accel_mps2;
		accel_count++;

		// Phase 3: has the evaluation window elapsed?
		if ((now_us - window_start_us) < window_us) {
			return ACC_HOLD_NOT_ELAPSED;
		}

		// Window complete — compute mean and decide.
		active = false;
		if (accel_count == 0) {
			return ACC_HOLD_DID_NOT_HOLD;
		}

		const double mean_accel = accel_sum / static_cast<double>(accel_count);
		if (mean_accel > static_cast<double>(ACCEL_LIFTOFF)) {
			return ACC_HOLD_DID_HOLD;
		}
		return ACC_HOLD_DID_NOT_HOLD;
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
		printf("[KAL-CFG] ESKF_FORCE_FLOAT32=%d  sizeof(eskf_scalar)=%lu  "
		       "COV_DECIM=%d  IMU_ODR=%d  BARO_ODR=%d\r\n",
		       ESKF_FORCE_FLOAT32,
		       static_cast<unsigned long>(sizeof(eskf_scalar)),
		       ESKF_COVARIANCE_DECIMATION,
		       ESKF_IMU_PRIMARY_ODR_HZ,
		       ESKF_BARO_ODR_HZ);
		printf("[KAL-CFG] activeBaroSources=%lu  "
		       "FORCE_FLIGHT=%d  PRINT_DECIM=%d  "
		       "FORCE_FLIGHT_DELAY_MS=%u\r\n",
		       static_cast<unsigned long>(kActiveBaroSources),
		       KALMAN_DEBUG_FORCE_FLIGHT,
		       KALMAN_DEBUG_PRINT_DECIMATION,
		       (unsigned)KALMAN_DEBUG_FORCE_FLIGHT_DELAY_MS);
#ifdef COMPILE_OPT_LEVEL
		printf("[KAL-CFG] Optimization: -O%d", COMPILE_OPT_LEVEL);
#elif defined(__OPTIMIZE__)
		printf("[KAL-CFG] Optimization: ON (unknown level)");
#else
		printf("[KAL-CFG] Optimization: OFF (-O0)");
#endif
		printf("  SYSCLK=%luMHz\r\n", SystemCoreClock / 1000000UL);
		app_timebase_print_init_diag();
		printf("[KAL-CFG] timebase_now: HAL=%lu  app=%lu  cycles_per_us=%lu\r\n",
		       static_cast<unsigned long>(HAL_GetTick()),
		       static_cast<unsigned long>(app_timebase_now_ms()),
		       SystemCoreClock / 1000000UL);
#endif
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
			estimator.reset();
			estimator.configureReplaySensorCounts(4u, kActiveBaroSources);
			active_imu_sources = 4u;
			apogee_hub.reset();
			last_imu_ts_us[0] = 0;
			last_imu_ts_us[1] = 0;
			last_imu_ts_us[2] = 0;
			last_imu_ts_us[3] = 0;
			has_prev_imu_ts[0] = false;
			has_prev_imu_ts[1] = false;
			has_prev_imu_ts[2] = false;
			has_prev_imu_ts[3] = false;
			liftoff_ms = 0;
			apogee_detected = false;
			last_body_accel_mps2 = {};
			acc_hold.reset();
			liftoff_detector.reset();
			KalmanHealthStore::instance().reset();
		}

		// Start liftoff acceleration-hold evaluation on IGNITION entry.
		if (state == flight_computer::State::IGNITION) {
			acc_hold.onIgnitionEnter(app_timebase_now_us());
		}
	}

	void onLiftoff(uint32_t liftoff_ms) {
		initIfNeeded();
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
				printf("[INGEST-IMU] src=%u count=%u ts[0]=%u:%u",
					(unsigned)source_index, (unsigned)count,
					(unsigned)(samples[0].timestamp_us >> 32),
					(unsigned)samples[0].timestamp_us);
				if (count >= 2) {
					printf("  ts[1]=%u:%u  dt=%u",
						(unsigned)(samples[1].timestamp_us >> 32),
						(unsigned)samples[1].timestamp_us,
						(unsigned)(uint32_t)(samples[1].timestamp_us - samples[0].timestamp_us));
				}
				printf("  ts[last]=%u:%u\r\n",
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
					printf("[BATCH-DT] #%u  t0=%u:%u  dt_us=%u  count=%u  "
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
			if (!liftoff_detector.isDetected()) {
				const bool gate = estimator.railShadow().isGateOpen();
				const bool was_armed = liftoff_detector.isArmed();
				for (size_t i = 0; i < slice_count; ++i) {
					liftoff_detector.update(
						slice[i].accel_x,
						slice[i].accel_y,
						slice[i].accel_z,
						gate);
				}
				if (!was_armed && liftoff_detector.isArmed()) {
					printf("[LIFTOFF] Detector ARMED (pad stable for ~2s)\r\n");
				}
				if (liftoff_detector.isDetected()) {
					printf("[LIFTOFF] IMU liftoff DETECTED!\r\n");
				}
			}

			last_body_accel_mps2.x = slice[slice_count - 1u].accel_x;
			last_body_accel_mps2.y = slice[slice_count - 1u].accel_y;
			last_body_accel_mps2.z = slice[slice_count - 1u].accel_z;
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

		// --- Liftoff acceleration-hold evaluation (DONE) ---
		// While in IGNITION, feed the evaluator with the latest body-X
		// acceleration (thrust axis ≈ vertical on the pad).  When the
		// evaluation window completes, write the result to eventStore.
		flight_computer::AccHoldStatus acc_hold_result =
			flight_computer::ACC_HOLD_NOT_ELAPSED;
		bool acc_hold_terminal = false;
		if (acc_hold.active) {
			// body_accel_mps2.x is the thrust-axis (body-X) accel from the
			// latest IMU sample — positive along thrust direction.
			const double vertical_accel = last_body_accel_mps2.x;
			acc_hold_result = acc_hold.evaluate(vertical_accel, now_us);
			acc_hold_terminal =
				(acc_hold_result != flight_computer::ACC_HOLD_NOT_ELAPSED);
		}

		const bool set_catastrophic_failure = eskf_diverged;
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
				printf("[APOGEE] Detected at t=%lu ms (liftoff+%lu ms)\r\n",
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
			// Forward ALL fixes to the estimator (including invalid ones) so
			// the estimator's own stale/frozen detection and fix-drop
			// diagnostics can observe the full GNSS health picture.
			// The estimator has its own usability gate:
			//   (sample.valid && sample.fix_type >= 2).
			estimator.processGpsSample(sample);
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
	for (;;) {
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
			printf("[KAL-DBG] Force-flight: triggering liftoff via FSM  "
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
