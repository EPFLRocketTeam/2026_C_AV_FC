
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Application/Modules/imu_modlue.hpp"

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"

extern RingBuffer<IMUData, 100> imuData1;
extern RingBuffer<IMUData, 100> imuData2;
extern RingBuffer<IMUData, 100> imuData3;

int kalman_loop() {
	return 0;
}
