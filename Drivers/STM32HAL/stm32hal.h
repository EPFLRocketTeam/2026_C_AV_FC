
#ifndef STM32_HAL_MOCK_H
#define STM32_HAL_MOCK_H

#ifndef UNIT_TEST_ENV
#include "stm32h7xx_hal.h"
#else
/* Place your mocked types here */

/*
 * For example, define the UART_HandleTypeDef
 *  to be just an integer so that it is can be
 *  used in the interfaces of the drivers.
 */
// using UART_HandleTypeDef = int;

#include <stdint.h>
#include <vector>
#include <queue>
#include <thread>
#include <chrono>
#include <cstring>

// Mock HAL status definitions

// Simple mock UART handle
typedef struct {
    uint32_t dummy; // Just a placeholder for the mock
} UART_HandleTypeDef;

// Global UART handle used in tests
extern UART_HandleTypeDef huart2;

typedef enum
{
  HAL_OK,
  HAL_ERROR,
  HAL_BUSY,
  HAL_TIMEOUT,
} HAL_StatusTypeDef;
// Mock HAL functions
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

// Mock control functions for testing
namespace MockHAL {
    // Initialize the mock HAL
    void Init();

    // Reset all mock state
    void Reset();

    // Add a fake GPS message to the receive queue
    void AddFakeUbxMessage(const std::vector<uint8_t>& message);

    // Add a complete NAV-PVT message with default test data
    void AddFakePvtMessage();

    // Check if all transmitted data has been consumed
    bool IsTransmitQueueEmpty();

    // Get the transmitted data for verification
    std::vector<uint8_t> GetTransmittedData();

    // Advance the mock time by the specified milliseconds
    void AdvanceTime(uint32_t ms);

    // Set the mock time to a specific value
    void SetTime(uint32_t ms);
}

#endif

#endif
