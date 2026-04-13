
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Application/Modules/imu_modlue.hpp"

extern "C" {
#include <Application/Kalman/kalman_process.h>
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"

int kalman_loop() {
	return 0;
}
