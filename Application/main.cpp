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
    Config imu_cfg1{};
    imu_cfg1.hspi = &hspi1;
    imu_cfg1.cs_port = BMI4_NSS_GPIO_Port;
    imu_cfg1.cs_pin = BMI4_NSS_Pin;
    imu_cfg1.use_dwt_timestamps = false;

    Config imu_cfg2 = imu_cfg1;
    imu_cfg2.cs_port = BMI3_NSS_GPIO_Port;
    imu_cfg2.cs_pin = BMI3_NSS_Pin;

    Config imu_cfg3 = imu_cfg1;
    imu_cfg3.cs_port = BMI2_NSS_GPIO_Port;
    imu_cfg3.cs_pin = BMI2_NSS_Pin;

    InvIMU_STM32 invImu1(imu_cfg1);
    InvIMU_STM32 invImu2(imu_cfg2);
    InvIMU_STM32 invImu3(imu_cfg3);

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
	currTick = HAL_GetTick() - startTick;
    	imuModule.update(currTick);
    	gpsModule.update(currTick);
        if (imuModule.takeProducedCount() > 0) {
            osThreadFlagsSet(kalmanTaskHandle, 0x0001U);
        }
    }
}
