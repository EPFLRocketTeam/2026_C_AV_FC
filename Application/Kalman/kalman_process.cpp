
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

	void ingestImu(size_t source_index, const IMUData &sample) {
		initIfNeeded();

		const bool has_prev_ts = has_prev_imu_ts[source_index];
		const uint64_t prev_ts = last_imu_ts_us[source_index];
		last_imu_ts_us[source_index] = sample.timestamp_us;
		has_prev_imu_ts[source_index] = true;

		app::ImuSample converted = convertImuSample(sample);
		app::ImuBatch batch{};
		batch.data = &converted;
		batch.count = 1;
		batch.t0_us = sample.timestamp_us;
		batch.dt_us =
			(has_prev_ts && sample.timestamp_us > prev_ts)
				? static_cast<uint32_t>(sample.timestamp_us - prev_ts)
				: kNominalImuDtUs;
		batch.source = static_cast<uint8_t>(source_index);
		batch.slot = 0xFF;

		estimator.processImuBatch(batch);

		last_body_accel_mps2.x = sample.accel_x;
		last_body_accel_mps2.y = sample.accel_y;
		last_body_accel_mps2.z = sample.accel_z;
		health.imu_samples_consumed += 1;
	}

	void onTick(uint64_t now_us) {
		initIfNeeded();
		estimator.onTick(now_us);

		const app::EstimatorOutput output = estimator.output();
		const bool eskf_diverged = estimator.isEskfDiverged();
		auto &goat = flight_computer::GOATStore::get_instance();

		const auto nav = app::mapEstimatorToNavigation(
			output,
			goat.navigationDataStore.get_baro(),
			last_body_accel_mps2);
		goat.navigationDataStore.set(nav);

		auto event = goat.eventStore.get();
		if (eskf_diverged) {
			event.catastrophic_failure = true;
		}

		if (!apogee_detected && liftoff_ms > 0) {
			const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000ULL);
			const app::ApogeeInput input = app::buildApogeeInput(
				output,
				eskf_diverged,
				last_state,
				liftoff_ms,
				now_ms);

			const app::ApogeeDecision decision = apogee_hub.update(now_ms, input);
			if (decision.triggered) {
				apogee_detected = true;
				event.apogee_detected = true;
			}
		}

		goat.eventStore.set(event);

		health.diverged = eskf_diverged;
		health.altitude_valid = output.altitude_valid;
		health.velocity_valid = output.velocity_valid;
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
			static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
		for (size_t i = 0; i < fix_count; ++i) {
			const app::sensors::gnss::GnssSample sample =
				convertGpsFixSample(staged_fixes[i], fallback_timestamp_us);
			estimator.processGpsSample(sample);
			health.gps_updates += 1;
		}

		if (gps_backlog_pending && kalmanTaskHandle != nullptr) {
			osThreadFlagsSet(kalmanTaskHandle, kKalmanThreadWakeFlag);
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

	uint32_t liftoff_ms = 0;
	if (kalman_take_pending_liftoff(&liftoff_ms) != 0U) {
		kalman.onLiftoff(liftoff_ms);
	}

	RingBuffer<IMUData, 100> *buffers[] = {&imuData1, &imuData2, &imuData3};
	osMutexId_t locks[] = {imuData1MutexHandle, imuData2MutexHandle,
						   imuData3MutexHandle};

	IMUData staged_samples[3][kMaxImuSamplesPerSourcePerRun] = {};
	size_t staged_count[3] = {0, 0, 0};
	size_t staged_index[3] = {0, 0, 0};
	bool source_healthy[3] = {false, false, false};
	int drained = 0;
	uint64_t latest_imu_ts = 0;
	bool imu_backlog_pending = false;

	for (size_t i = 0; i < 3; ++i) {
		if (locks[i] == nullptr) {
			continue;
		}

		source_healthy[i] =
			(app_imu_sensor_healthy(static_cast<uint8_t>(i)) != 0U);

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

		kalman.ingestImu(best_source,
					 staged_samples[best_source][staged_index[best_source]]);
		++staged_index[best_source];
	}

	if (latest_imu_ts == 0) {
		latest_imu_ts = static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
	}

	kalman.ingestAidingFromStore();

	kalman.onTick(latest_imu_ts);

	if (imu_backlog_pending && kalmanTaskHandle != nullptr) {
		osThreadFlagsSet(kalmanTaskHandle, kKalmanThreadWakeFlag);
	}

	return drained;
}
