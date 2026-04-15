// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Modules/imu_modlue.hpp"
#include "Modules/gps_module.hpp"

extern "C" {
#include "Application/main.h"
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"
#include  "Drivers/UBX_GPS/ubx_gps_interface.h"

extern SPI_HandleTypeDef hspi1;
extern osThreadId_t kalmanTaskHandle;
extern UART_HandleTypeDef huart7;


RingBuffer<IMUData, 100> imuData1;
RingBuffer<IMUData, 100> imuData2;
RingBuffer<IMUData, 100> imuData3;

RingBuffer<GpsBasicFixData, 100> gpsData;


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

    UbxGpsInterface gps(&huart7, 1000);
    UbxGpsInterface* gpsArr[] = {&gps};
    RingBuffer<GpsBasicFixData, 100>* gpsRing[] = {&gpsData};

    GpsModule gpsModule(gpsArr, gpsRing);
    if (!gpsModule.init()) {
		return;
    }


    uint32_t startTick = HAL_GetTick();
    uint32_t currTick;
    while (1) {
    	currTick = startTick - HAL_GetTick();
    	imuModule.update(currTick);
    	gpsModule.update(currTick);
    	if (imuData1.size() == 100) {
    		osThreadFlagsSet(kalmanTaskHandle, 0x0001U);
    	}
    }
}
