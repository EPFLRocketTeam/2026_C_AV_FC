#ifndef MOCK_UART_DEVICE_H
#define MOCK_UART_DEVICE_H

#include "Drivers/STM32HAL/Simulations/stm32sim_uart.hpp"
#include <vector>
#include <queue>
#include <cstdint>

namespace SIMULATOR_NAMESPACE::interfaces {

class MockUARTDevice {
public:
    /**
     * Construct a mock device and register it with the simulator.
     * @param huart The UART handle to bind to.
     */
    MockUARTDevice(UART_HandleTypeDef *huart);

    /**
     * Unregister the device automatically when it goes out of scope.
     */
    ~MockUARTDevice();

    // Prevent copying (because of the 'this' pointer passed to C callbacks)
    MockUARTDevice(const MockUARTDevice&) = delete;
    MockUARTDevice& operator=(const MockUARTDevice&) = delete;

    /* ---------------------------------------------------------
     *  Test Helper API
     * --------------------------------------------------------- */

    /** 
     * Connect this device's TX output to another device's RX input.
     * Creates a virtual wire.
     */
    void connectTo(MockUARTDevice *peer);

    /**
     * Feed data into this device's RX buffer (simulating incoming data).
     */
    void feedRx(const std::vector<uint8_t> &data);

    /**
     * Read data that was transmitted by this device.
     */
    std::vector<uint8_t> captureTx();

    /**
     * Get a reference to the raw TX buffer (for inspection without clearing).
     */
    const std::vector<uint8_t>& getTxBuffer() const;

    /**
     * Check if RX buffer is empty.
     */
    bool rxEmpty() const;

    /**
     * Get the underlying UART handle.
     */
    UART_HandleTypeDef* getHandle() const;

private:
    UART_HandleTypeDef *huart_;
    uart::SIM_UARTDevice device_ = -1;
    
    // State
    std::queue<uint8_t> rxQueue_;
    std::vector<uint8_t> txBuffer_;
    
    // Cross-wiring
    MockUARTDevice *peer_ = nullptr;

    /* ---------------------------------------------------------
     *  Callbacks (C-linkage style static functions)
     * --------------------------------------------------------- */

    static void sOnTransmit(void *data, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
    static HAL_StatusTypeDef sOnReceive(void *data, uint8_t *pData, uint16_t Size, uint32_t Timeout);
};

} // namespace SIMULATOR_NAMESPACE::interfaces

#endif // MOCK_UART_DEVICE_H
