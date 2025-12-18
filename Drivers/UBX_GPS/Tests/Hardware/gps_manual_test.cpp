/*
 * gps_manual_test.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: maxime
 */

#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "main.h"
#include <cstdio>





#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&serial_huart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

int main() {
  HAL_Init();
  SystemClock_Config();
  MX_USART2_UART_Init(); // GPS UART
  MX_USART3_UART_Init(); // Debug UART
  MX_GPIO_Init();

  printf("\r\n--- UBX GPS Interface Test ---\r\n");

  UbxGpsInterface gps(&huart2, 1000); // 1Hz update rate

  printf("Initializing GPS...\r\n");
  GpsStatus status = gps.init();

  if (status != GpsStatus::OK) {
    printf("GPS init failed! Status = %d\r\n", static_cast<int>(status));
    while (1)
      ;
  }

  printf("GPS initialized successfully.\r\n");

  // Main loop: read data every second
  GpsBasicFixData fixData;

  while (1) {
    status = gps.getPvt(&fixData, 2000); // timeout 2s

    if (status == GpsStatus::OK) {
      double lat_deg = fixData.lat * UBX_SCALE_LAT_LON;
      double lon_deg = fixData.lon * UBX_SCALE_LAT_LON;

      printf("Fix OK | Type: %d | Sats: %2d | Lat: | Lon: | hAcc: %lu cm\r\n",
             static_cast<int>(fixData.fixType),
             fixData.numSV,

             (int)fixData.hAcc);
    } else if (status == GpsStatus::ERROR_TIMEOUT) {
      printf("No GPS fix received (timeout)\r\n");
    } else {
      printf("GPS read error: %d\r\n", static_cast<int>(status));
    }

    HAL_Delay(1000);
  }
}


