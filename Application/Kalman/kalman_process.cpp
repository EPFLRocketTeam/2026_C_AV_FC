
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Kalman/AppLayer/eskf_estimator.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/Modules/imu_modlue.hpp"

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"

#include <algorithm>
#include <cmath>

extern osMutexId_t imuData1MutexHandle;
extern osMutexId_t imuData2MutexHandle;
extern osMutexId_t imuData3MutexHandle;

extern RingBuffer<IMUData, 100> imuData1;
extern RingBuffer<IMUData, 100> imuData2;
extern RingBuffer<IMUData, 100> imuData3;

namespace {

struct KalmanRuntime {
	app::EskfEstimator estimator;
	bool initialized = false;
	uint64_t last_imu_ts_us[3] = {0, 0, 0};
	flight_computer::State last_state = flight_computer::State::INIT;

	void initIfNeeded() {
		if (initialized) {
			return;
		}

		estimator.reset();
		estimator.configureReplaySensorCounts(3, 0);
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
		}
	}

	void onLiftoff(uint32_t liftoff_ms) {
		initIfNeeded();
		estimator.onLiftoff(liftoff_ms);
	}

	static app::ImuSample convertImuSample(const IMUData &sample) {
		constexpr float kAccelScale = (16.0f * 9.80665f) / 32768.0f;
		constexpr float kGyroScale =
			(2000.0f * 3.14159265359f / 180.0f) / 32768.0f;
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
	}

	void onTick(uint64_t now_us) {
		initIfNeeded();
		estimator.onTick(now_us);
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

		osMutexAcquire(locks[i], osWaitForever);
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
	kalman.onTick(latest_imu_ts);

	return drained;
}
