
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Kalman/AppLayer/apogee_hub.hpp"
#include "Application/Kalman/AppLayer/apogee_factory.hpp"
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

extern osMutexId_t gpsDataMutexHandle;


extern RingBuffer<IMUData, 100> imuData1;
extern RingBuffer<IMUData, 100> imuData2;
extern RingBuffer<IMUData, 100> imuData3;

extern RingBuffer<GpsBasicFixData, 100> gpsData;


namespace {

struct KalmanRuntime {
	app::EskfEstimator estimator;
	app::ApogeeHub apogee_hub;
	bool apogee_ready = false;
	bool apogee_detected = false;
	uint32_t liftoff_ms = 0;
	bool initialized = false;
	uint64_t last_imu_ts_us[3] = {0, 0, 0};
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

	void ingestImu(size_t source_index, const IMUData &sample) {
		initIfNeeded();

		const uint64_t prev_ts = last_imu_ts_us[source_index];
		last_imu_ts_us[source_index] = sample.timestamp_us;

		app::ImuSample converted = convertImuSample(sample);
		app::ImuBatch batch{};
		batch.data = &converted;
		batch.count = 1;
		batch.t0_us = sample.timestamp_us;
		batch.dt_us =
			(prev_ts > 0 && sample.timestamp_us > prev_ts)
				? static_cast<uint32_t>(sample.timestamp_us - prev_ts)
				: 1000U;
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
				static_cast<float>(last_body_accel_mps2.x),
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

		// TODO(kalman): Wire barometer into estimator once BMP samples are exposed
		// through a dedicated timestamped queue/ring-buffer.
		// TODO(kalman): Wire GNSS fixes into estimator once GPS module publishes
		// validated fix packets to the Kalman task boundary.
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

	IMUData sample{};
	int drained = 0;
	uint64_t latest_imu_ts = 0;

	for (size_t i = 0; i < 3; ++i) {
		if (locks[i] == nullptr) {
			continue;
		}

		const bool source_healthy =
			(app_imu_sensor_healthy(static_cast<uint8_t>(i)) != 0U);

		osMutexAcquire(locks[i], osWaitForever);
		if (!source_healthy) {
			while (buffers[i]->pop(sample)) {
				latest_imu_ts = std::max(latest_imu_ts, sample.timestamp_us);
				++drained;
			}
			osMutexRelease(locks[i]);
			continue;
		}

		while (buffers[i]->pop(sample)) {
			kalman.ingestImu(i, sample);
			latest_imu_ts = std::max(latest_imu_ts, sample.timestamp_us);
			++drained;
		}
		osMutexRelease(locks[i]);
	}

	if (latest_imu_ts == 0) {
		latest_imu_ts = static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
	}

	kalman.ingestAidingFromStore();

	kalman.onTick(latest_imu_ts);

	return drained;
}
