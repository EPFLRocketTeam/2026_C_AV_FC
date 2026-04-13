
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Modules/imu_modlue.hpp"

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"

extern osMutexId_t imuData1MutexHandle;
extern osMutexId_t imuData2MutexHandle;
extern osMutexId_t imuData3MutexHandle;

extern RingBuffer<IMUData, 100> imuData1;
extern RingBuffer<IMUData, 100> imuData2;
extern RingBuffer<IMUData, 100> imuData3;

int kalman_loop() {
	uint32_t liftoff_ms = 0;
	if (kalman_take_pending_liftoff(&liftoff_ms) != 0U) {
		// TODO: feed liftoff timestamp to estimator lifecycle when P2/P3
		// estimator object is introduced in this task.
		(void)liftoff_ms;
	}

	const uint32_t current_state = kalman_current_state();
	(void)current_state;

	RingBuffer<IMUData, 100> *buffers[] = {&imuData1, &imuData2, &imuData3};
	osMutexId_t locks[] = {imuData1MutexHandle, imuData2MutexHandle,
						   imuData3MutexHandle};

	IMUData sample{};
	int drained = 0;

	for (size_t i = 0; i < 3; ++i) {
		if (locks[i] == nullptr) {
			continue;
		}

		osMutexAcquire(locks[i], osWaitForever);
		while (buffers[i]->pop(sample)) {
			++drained;
		}
		osMutexRelease(locks[i]);
	}

	return drained;
}
