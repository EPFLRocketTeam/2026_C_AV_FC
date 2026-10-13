#pragma once
#include "Application/Data/ring_buffer.hpp"
#include "Drivers/InvIMU/InvIMU.h"
using namespace Drivers::InvIMU;
#ifndef APP_IMU_RING_CAPACITY
#define APP_IMU_RING_CAPACITY 128u
#endif
using AppImuRingBuffer = RingBuffer<IMUData, APP_IMU_RING_CAPACITY>;
extern AppImuRingBuffer imuData1;
extern AppImuRingBuffer imuData2;
extern AppImuRingBuffer imuData3;
extern AppImuRingBuffer imuData4;
