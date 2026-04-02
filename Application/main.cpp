// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Modules/imu_modlue.hpp"

extern "C" {
#include "Application/main.h"
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"

extern SPI_HandleTypeDef hspi1;
extern osThreadId_t kalmanTaskHandle;

static RingBuffer<IMUData, 100> imuData1;
static RingBuffer<IMUData, 100> imuData2;
static RingBuffer<IMUData, 100> imuData3;

void mainLoop() {
    Config imu_cfg{};
    imu_cfg.hspi = &hspi1;
    imu_cfg.cs_port = BMI4_NSS_GPIO_Port;
    imu_cfg.cs_pin = BMI4_NSS_Pin;
    imu_cfg.use_dwt_timestamps = false;

    InvIMU_STM32 invImu1(imu_cfg);
    InvIMU_STM32 invImu2(imu_cfg);
    InvIMU_STM32 invImu3(imu_cfg);

    InvIMU_Interface* invArr[] = {&invImu1, &invImu2, &invImu3};
    RingBuffer<IMUData, 100>* ringArr[] = {&imuData1, &imuData2, &imuData3};

    ImuModule imuModule(invArr, ringArr);

    if (!imuModule.init()) {
    	return;
    }
    uint32_t startTick = HAL_GetTick();
    uint32_t currTick;
    while (1) {
    	currTick = startTick - HAL_GetTick();
    	imuModule.update(currTick);

    	if (imuData1.size() == 100) {
    		osThreadFlagsSet(kalmanTaskHandle, 0x0001U);
    	}
    }
}
