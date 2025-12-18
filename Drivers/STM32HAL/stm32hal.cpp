/*
 * stm32hal.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: maxime
 */




#include "stm32hal.h"
#ifdef UNIT_TEST_ENV
#include <iostream>
#include <mutex>

// Ghlobal UART handle used in tests
UART_HandleTypeDef huart2;

// Mock HAL implementation
namespace MockHAL {
    // Time tracking
    static uint32_t mock_time = 0;

    // Transmit and receive queues
    static std::queue<uint8_t> rx_queue;
    static std::vector<uint8_t> tx_buffer;

    // Mutex for thread safety
    static std::mutex mock_mutex;

    void Init() {
        Reset();
    }

    void Reset() {
        std::lock_guard<std::mutex> lock(mock_mutex);
        mock_time = 0;
        std::queue<uint8_t> empty;
        rx_queue.swap(empty);
        tx_buffer.clear();
    }

    void AddFakeUbxMessage(const std::vector<uint8_t>& message) {
        std::lock_guard<std::mutex> lock(mock_mutex);
        for (auto byte : message) {
            rx_queue.push(byte);
        }
    }

    void AddFakePvtMessage() {
        // Create a valid NAV-PVT message with test data
        std::vector<uint8_t> message;

        // UBX header
        message.push_back(0xB5);  // Sync char 1
        message.push_back(0x62);  // Sync char 2
        message.push_back(0x01);  // Class: NAV
        message.push_back(0x07);  // ID: PVT
        message.push_back(0x5C);  // Length LSB (92 bytes)
        message.push_back(0x00);  // Length MSB

        // Payload (92 bytes) - using test data
        uint8_t payload[92] = {
            // iTOW (4 bytes)
            0x10, 0x27, 0x05, 0x00,
            // Year (2 bytes)
            0xE6, 0x07,
            // Month (1 byte)
            0x0C,
            // Day (1 byte)
            0x0B,
            // Hour (1 byte)
            0x0A,
            // Minute (1 byte)
            0x29,
            // Second (1 byte)
            0x15,
            // Valid (1 byte) - all flags set
            0x0F,
            // tAcc (4 bytes)
            0x20, 0x00, 0x00, 0x00,
            // nano (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // fixType (1 byte) - 3D fix
            0x03,
            // Flags (1 byte) - GNSS fix OK
            0x01,
            // Flags2 (1 byte)
            0x00,
            // numSV (1 byte)
            0x08,
            // lon (4 bytes) - 8.542278 degrees = 85422780 * 1e-7
			// 0x05, 0x09, 0x72, 0x2C,
            0x2C, 0x72, 0x09, 0x05,
            // lat (4 bytes) - 47.376888 degrees = 473768880 * 1e-7
            0x60, 0xE6, 0x3A, 0x02,
            // height (4 bytes)
            0x80, 0xF2, 0x04, 0x00,
            // hMSL (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // hAcc (4 bytes)
            0x50, 0x00, 0x00, 0x00,
            // vAcc (4 bytes)
            0x60, 0x00, 0x00, 0x00,
            // velN (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // velE (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // velD (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // gSpeed (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // headMot (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // sAcc (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // headAcc (4 bytes)
            0x00, 0x00, 0x00, 0x00,
            // pDOP (2 bytes)
            0x80, 0x00,
            // Remaining padding to reach 92 bytes
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

        // Calculate checksum
        uint8_t ck_a = 0, ck_b = 0;
        for (int i = 2; i < 2 + 4 + 92; i++) {
            ck_a += (i < 2 + 4) ? message[i] : payload[i-6];
            ck_b += ck_a;
        }

        // Add payload to message
        for (int i = 0; i < 92; i++) {
            message.push_back(payload[i]);
        }

        // Add checksum
        message.push_back(ck_a);
        message.push_back(ck_b);

        // Add to queue
        AddFakeUbxMessage(message);
    }

    bool IsTransmitQueueEmpty() {
        std::lock_guard<std::mutex> lock(mock_mutex);
        return tx_buffer.empty();
    }

    std::vector<uint8_t> GetTransmittedData() {
        std::lock_guard<std::mutex> lock(mock_mutex);
        return tx_buffer;
    }

    void AdvanceTime(uint32_t ms) {
        std::lock_guard<std::mutex> lock(mock_mutex);
        mock_time += ms;
    }

    void SetTime(uint32_t ms) {
        std::lock_guard<std::mutex> lock(mock_mutex);
        mock_time = ms;
    }
}

// Mock HAL implementations
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;  // Unused in mock
    (void)Timeout; // Not used in mock

    std::lock_guard<std::mutex> lock(MockHAL::mock_mutex);

    // Copy data to transmit buffer for verification
    for (uint16_t i = 0; i < Size; i++) {
        MockHAL::tx_buffer.push_back(pData[i]);
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;  // Unused in mock

    std::lock_guard<std::mutex> lock(MockHAL::mock_mutex);

    // Check if we have enough data in the queue
    if (MockHAL::rx_queue.size() < Size) {
        // Simulate timeout by advancing time if needed
        MockHAL::mock_time += Timeout;
        return HAL_TIMEOUT;
    }

    // Copy data from queue to buffer
    for (uint16_t i = 0; i < Size; i++) {
        pData[i] = MockHAL::rx_queue.front();
        MockHAL::rx_queue.pop();
    }

    return HAL_OK;
}

uint32_t HAL_GetTick(void) {
    std::lock_guard<std::mutex> lock(MockHAL::mock_mutex);
    return MockHAL::mock_time;
}

void HAL_Delay(uint32_t Delay) {
    std::lock_guard<std::mutex> lock(MockHAL::mock_mutex);
    MockHAL::mock_time += Delay;
}

#endif
