
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Kalman/AppLayer/apogee_hub.hpp"
#include "Application/Kalman/AppLayer/apogee_factory.hpp"
#include "Application/Kalman/AppLayer/hw_config.hpp"
#include "Application/Kalman/AppLayer/output_bridge.hpp"
#include "Application/Kalman/kalman_health.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/Data/data.hpp"
#include "Application/Modules/imu_modlue.hpp"

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Application/app_timebase.h"
#include "Application/main.h"
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

extern osMutexId_t imuData1MutexHandle;
extern osMutexId_t imuData2MutexHandle;
extern osMutexId_t imuData3MutexHandle;
extern osThreadId_t kalmanTaskHandle;

extern osMutexId_t gpsDataMutexHandle;
extern osMutexId_t eventStoreMutexHandle;
extern osMutexId_t navigationDataMutexHandle;


extern RingBuffer<IMUData, 100> imuData1;
extern RingBuffer<IMUData, 100> imuData2;
extern RingBuffer<IMUData, 100> imuData3;

extern RingBuffer<GpsBasicFixData, 100> gpsData;


namespace {

constexpr uint32_t kKalmanThreadWakeFlag = 0x0001U;
constexpr size_t kMaxImuSamplesPerSourcePerRun = 32u;
constexpr size_t kMaxGpsSamplesPerRun = 8u;
#if APP_IMU_PRIMARY_ODR_HZ > 0
constexpr uint32_t kNominalImuDtUs =
	static_cast<uint32_t>(1000000ULL / APP_IMU_PRIMARY_ODR_HZ);
#else
constexpr uint32_t kNominalImuDtUs = 1000U;
#endif

struct KalmanRuntime {
	app::EskfEstimator estimator;
	app::ApogeeHub apogee_hub;
	bool apogee_ready = false;
	bool apogee_detected = false;
	uint32_t liftoff_ms = 0;
	bool initialized = false;
	uint64_t last_imu_ts_us[3] = {0, 0, 0};
	bool has_prev_imu_ts[3] = {false, false, false};
	flight_computer::State last_state = flight_computer::State::INIT;
	flight_computer::Vector3 last_body_accel_mps2{};
	KalmanHealthSnapshot health{};
	size_t active_imu_sources = 3u;

	void initIfNeeded() {
		if (initialized) {
			return;
		}

		estimator.reset();
		estimator.configureReplaySensorCounts(3, 0);
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
			estimator.configureReplaySensorCounts(3u, 0u);
			active_imu_sources = 3u;
			apogee_hub.reset();
			last_imu_ts_us[0] = 0;
			last_imu_ts_us[1] = 0;
			last_imu_ts_us[2] = 0;
			has_prev_imu_ts[0] = false;
			has_prev_imu_ts[1] = false;
			has_prev_imu_ts[2] = false;
			liftoff_ms = 0;
			apogee_detected = false;
			last_body_accel_mps2 = {};
			KalmanHealthStore::instance().reset();
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
		estimator.configureReplaySensorCounts(imu_sources, 0u);
		active_imu_sources = imu_sources;
	}

	static app::ImuSample convertImuSample(const IMUData &sample) {
		constexpr float kAccelScale = (INV_IMU_MAX_ACCEL_G * 9.80665f) / 32768.0f;
		constexpr float kGyroScale =
			(INV_IMU_MAX_GYRO_DPS * 3.14159265359f / 180.0f) / 32768.0f;
		constexpr float kTempScale = 1.0f / 2.07f;
		constexpr float kTempOffsetK = 25.0f + 273.15f;

		app::ImuSample out{};
		out.ax = sample.accel_x / kAccelScale;
		out.ay = sample.accel_y / kAccelScale;
		out.az = sample.accel_z / kAccelScale;
		out.gx = sample.gyro_x / kGyroScale;
		out.gy = sample.gyro_y / kGyroScale;
		out.gz = sample.gyro_z / kGyroScale;

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

	void ingestImuChunk(size_t source_index, const IMUData *samples, size_t count) {
		initIfNeeded();
		if (samples == nullptr || count == 0u) {
			return;
		}

		if (count > kMaxImuSamplesPerSourcePerRun) {
			count = kMaxImuSamplesPerSourcePerRun;
		}

		const bool has_prev_ts = has_prev_imu_ts[source_index];
		const uint64_t prev_ts = last_imu_ts_us[source_index];
		const uint64_t first_ts = samples[0].timestamp_us;
		const uint64_t last_ts = samples[count - 1u].timestamp_us;
		last_imu_ts_us[source_index] = last_ts;
		has_prev_imu_ts[source_index] = true;

		app::ImuSample converted[kMaxImuSamplesPerSourcePerRun] = {};
		for (size_t i = 0; i < count; ++i) {
			converted[i] = convertImuSample(samples[i]);
		}

		app::ImuBatch batch{};
		batch.data = converted;
		batch.count = count;
		batch.t0_us = first_ts;
		if (count >= 2u && samples[1].timestamp_us > first_ts) {
			batch.dt_us =
				static_cast<uint32_t>(samples[1].timestamp_us - first_ts);
		} else {
			batch.dt_us =
				(has_prev_ts && first_ts > prev_ts)
					? static_cast<uint32_t>(first_ts - prev_ts)
					: kNominalImuDtUs;
		}
		batch.source = static_cast<uint8_t>(source_index);
		batch.slot = 0xFF;

		estimator.processImuBatch(batch);

		last_body_accel_mps2.x = samples[count - 1u].accel_x;
		last_body_accel_mps2.y = samples[count - 1u].accel_y;
		last_body_accel_mps2.z = samples[count - 1u].accel_z;
		health.imu_samples_consumed += static_cast<uint32_t>(count);
	}

	void onTick(uint64_t now_us) {
		initIfNeeded();
		estimator.onTick(now_us);

		const app::EstimatorOutput output = estimator.output();
		const bool eskf_diverged = estimator.isEskfDiverged();
		const bool is_coast_phase = estimator.isCoastPhase();
		auto &goat = flight_computer::GOATStore::get_instance();
		flight_computer::bmp3_data latest_baro{};

		if (navigationDataMutexHandle != nullptr) {
			osMutexAcquire(navigationDataMutexHandle, osWaitForever);
		}

		latest_baro = goat.navigationDataStore.get_baro();

		const auto nav = app::mapEstimatorToNavigation(
			output,
			latest_baro,
			last_body_accel_mps2);
		goat.navigationDataStore.set(nav);

		if (navigationDataMutexHandle != nullptr) {
			osMutexRelease(navigationDataMutexHandle);
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
			}
		}

		if (set_catastrophic_failure || set_apogee_detected) {
			if (eventStoreMutexHandle != nullptr) {
				osMutexAcquire(eventStoreMutexHandle, osWaitForever);
			}

			auto event = goat.eventStore.get();
			if (set_catastrophic_failure) {
				event.catastrophic_failure = true;
			}
			if (set_apogee_detected) {
				event.apogee_detected = true;
			}
			goat.eventStore.set(event);

			if (eventStoreMutexHandle != nullptr) {
				osMutexRelease(eventStoreMutexHandle);
			}
		}

		health.diverged = eskf_diverged;
		health.altitude_valid = output.altitude_valid;
		health.velocity_valid = output.velocity_valid;
		health.wake_imu = kalman_wake_count_imu();
		health.wake_lifecycle = kalman_wake_count_lifecycle();
		health.wake_timer = kalman_wake_count_timer();
		health.wake_backlog = kalman_wake_count_backlog();
		KalmanHealthStore::instance().set(health);
	}

	void ingestAidingFromStore() {
		GpsBasicFixData staged_fixes[kMaxGpsSamplesPerRun] = {};
		size_t fix_count = 0;
		bool gps_backlog_pending = false;

		if (gpsDataMutexHandle != nullptr) {
			osMutexAcquire(gpsDataMutexHandle, osWaitForever);
		}

		while (fix_count < kMaxGpsSamplesPerRun &&
			   gpsData.pop(staged_fixes[fix_count])) {
			++fix_count;
		}
		gps_backlog_pending = !gpsData.empty();

		if (gpsDataMutexHandle != nullptr) {
			osMutexRelease(gpsDataMutexHandle);
		}

		const uint64_t fallback_timestamp_us =
			app_timebase_now_us();
		for (size_t i = 0; i < fix_count; ++i) {
			const app::sensors::gnss::GnssSample sample =
				convertGpsFixSample(staged_fixes[i], fallback_timestamp_us);
			if (!sample.valid) {
				continue;
			}
			estimator.processGpsSample(sample);
			health.gps_updates += 1;
		}

		if (gps_backlog_pending && kalmanTaskHandle != nullptr) {
			osThreadFlagsSet(kalmanTaskHandle, kKalmanThreadWakeFlag);
			kalman_note_wake_backlog();
		}

		// TODO(kalman): Wire barometer into estimator once BMP samples are exposed
		// through a dedicated timestamped queue/ring-buffer.
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

	const uint32_t current_state = kalman_current_state();
	kalman.onStateChange(current_state);

	RingBuffer<IMUData, 100> *buffers[] = {&imuData1, &imuData2, &imuData3};
	osMutexId_t locks[] = {imuData1MutexHandle, imuData2MutexHandle,
						   imuData3MutexHandle};

	IMUData staged_samples[3][kMaxImuSamplesPerSourcePerRun] = {};
	size_t staged_count[3] = {0, 0, 0};
	size_t staged_index[3] = {0, 0, 0};
	bool source_healthy[3] = {false, false, false};
	size_t healthy_source_count = 0;
	int drained = 0;
	uint64_t latest_imu_ts = 0;
	bool imu_backlog_pending = false;

	for (size_t i = 0; i < 3; ++i) {
		if (locks[i] == nullptr) {
			continue;
		}

		source_healthy[i] =
			(app_imu_sensor_healthy(static_cast<uint8_t>(i)) != 0U);
		if (source_healthy[i]) {
			++healthy_source_count;
		}

		osMutexAcquire(locks[i], osWaitForever);
		size_t source_samples = 0;
		while (source_samples < kMaxImuSamplesPerSourcePerRun &&
			   buffers[i]->pop(staged_samples[i][source_samples])) {
			latest_imu_ts = std::max(latest_imu_ts,
							  staged_samples[i][source_samples].timestamp_us);
			++source_samples;
		}
		if (!buffers[i]->empty()) {
			imu_backlog_pending = true;
		}
		osMutexRelease(locks[i]);

		staged_count[i] = source_samples;
		drained += static_cast<int>(source_samples);
	}

	kalman.setActiveImuSources(healthy_source_count);

	for (;;) {
		size_t best_source = 3;
		uint64_t best_ts = UINT64_MAX;

		for (size_t i = 0; i < 3; ++i) {
			if (!source_healthy[i]) {
				continue;
			}
			if (staged_index[i] >= staged_count[i]) {
				continue;
			}

			const uint64_t ts = staged_samples[i][staged_index[i]].timestamp_us;
			if (ts < best_ts) {
				best_ts = ts;
				best_source = i;
			}
		}

		if (best_source >= 3) {
			break;
		}

		uint64_t next_other_ts = UINT64_MAX;
		for (size_t i = 0; i < 3; ++i) {
			if (i == best_source || !source_healthy[i]) {
				continue;
			}
			if (staged_index[i] >= staged_count[i]) {
				continue;
			}
			next_other_ts =
				std::min(next_other_ts, staged_samples[i][staged_index[i]].timestamp_us);
		}

		const size_t start_idx = staged_index[best_source];
		size_t chunk_count = 0;
		while ((start_idx + chunk_count) < staged_count[best_source]) {
			const uint64_t ts =
				staged_samples[best_source][start_idx + chunk_count].timestamp_us;
			if (chunk_count > 0u && next_other_ts != UINT64_MAX && ts > next_other_ts) {
				break;
			}
			++chunk_count;
		}

		if (chunk_count == 0u) {
			chunk_count = 1u;
		}

		kalman.ingestImuChunk(
			best_source,
			&staged_samples[best_source][start_idx],
			chunk_count);
		staged_index[best_source] += chunk_count;
	}

	if (latest_imu_ts == 0) {
		latest_imu_ts = app_timebase_now_us();
	}

	kalman.ingestAidingFromStore();

	uint32_t liftoff_ms = 0;
	if (kalman_take_pending_liftoff(&liftoff_ms) != 0U) {
		kalman.onLiftoff(liftoff_ms);
	}

	kalman.onTick(latest_imu_ts);

	if (imu_backlog_pending && kalmanTaskHandle != nullptr) {
		osThreadFlagsSet(kalmanTaskHandle, kKalmanThreadWakeFlag);
		kalman_note_wake_backlog();
	}

	return drained;
}
