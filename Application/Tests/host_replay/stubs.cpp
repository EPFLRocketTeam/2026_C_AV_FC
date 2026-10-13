// Host stubs for running the firmware Kalman runtime (kalman_process.cpp)
// against recorded data. Provides a simulated microsecond clock, the
// application ring buffers, IMU health, flight params and the HAL tick.
#include <cstdint>
#include <cstdio>

#include "Application/Data/ring_buffer.hpp"
#include "Drivers/InvIMU/InvIMU.h"
#include "Drivers/BMP390/BMP390.h"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "Application/Config/config.hpp"

uint64_t g_sim_us = 0;
bool g_printf_enabled = true;

using Drivers::InvIMU::IMUData;
RingBuffer<IMUData, 128u> imuData1, imuData2, imuData3, imuData4;
RingBuffer<Drivers::BMP390::BaroData, 100> baroData1, baroData2, baroData3, baroData4;
RingBuffer<GpsBasicFixData, 100> gpsData;

bool g_liftoff_detection_allowed = false;
extern "C" { volatile bool g_uart_force_liftoff = false; }

uint32_t SystemCoreClock = 480000000u;
uint32_t HAL_GetTick(void) { return static_cast<uint32_t>(g_sim_us / 1000u); }

extern "C" {
void app_timebase_init(void) {}
uint64_t app_timebase_now_us(void) { return g_sim_us; }
uint32_t app_timebase_now_ms(void) { return static_cast<uint32_t>(g_sim_us / 1000u); }
void app_timebase_print_init_diag(void) {}
bool app_printf_is_enabled() { return g_printf_enabled; }
bool g_imu_healthy_sim[4] = {true, true, true, true};
uint8_t app_imu_sensor_healthy(uint8_t i) { return (i < 4 && g_imu_healthy_sim[i]) ? 1u : 0u; }
uint32_t app_imu_sensor_status_flags(uint8_t) { return 0u; }
}

namespace config {
static FlightParams g_params{};
const FlightParams &get() { return g_params; }
}
