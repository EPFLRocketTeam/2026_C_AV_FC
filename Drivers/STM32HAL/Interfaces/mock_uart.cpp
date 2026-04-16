#include "mock_uart.hpp"

namespace SIMULATOR_NAMESPACE::interfaces {

MockUARTDevice::MockUARTDevice(UART_HandleTypeDef *huart) : huart_(huart) {
    uart::SIM_UARTDeviceParameters params = {};
    params.deviceData = this;
    params.onTransmit = &MockUARTDevice::sOnTransmit;
    params.onReceive = &MockUARTDevice::sOnReceive;
    
    device_ = uart::SIM_RegisterUARTDevice(huart_, params);
}

MockUARTDevice::~MockUARTDevice() {
    if (device_ >= 0) {
        uart::SIM_UnregisterUARTDevice(device_);
    }
}

void MockUARTDevice::connectTo(MockUARTDevice *peer) {
    peer_ = peer;
}

void MockUARTDevice::feedRx(const std::vector<uint8_t> &data) {
    for (auto b : data) {
        rxQueue_.push(b);
    }
}

std::vector<uint8_t> MockUARTDevice::captureTx() {
    std::vector<uint8_t> data = txBuffer_;
    txBuffer_.clear();
    return data;
}

const std::vector<uint8_t>& MockUARTDevice::getTxBuffer() const {
    return txBuffer_;
}

bool MockUARTDevice::rxEmpty() const {
    return rxQueue_.empty();
}

UART_HandleTypeDef* MockUARTDevice::getHandle() const {
    return huart_;
}

/* ---------------------------------------------------------
 *  Static Callback Implementations
 * --------------------------------------------------------- */

void MockUARTDevice::sOnTransmit(void *data, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    auto *self = static_cast<MockUARTDevice*>(data);
    
    // 1. Log locally
    for (uint16_t i = 0; i < Size; i++) {
        self->txBuffer_.push_back(pData[i]);
    }

    // 2. If wired, push to peer's RX immediately
    if (self->peer_) {
        for (uint16_t i = 0; i < Size; i++) {
            self->peer_->rxQueue_.push(pData[i]);
        }
    }
    
    (void)Timeout; // Unused in simple mock
}

HAL_StatusTypeDef MockUARTDevice::sOnReceive(void *data, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    auto *self = static_cast<MockUARTDevice*>(data);

    if (self->rxQueue_.size() < Size) {
        return HAL_TIMEOUT;
    }

    for (uint16_t i = 0; i < Size; i++) {
        pData[i] = self->rxQueue_.front();
        self->rxQueue_.pop();
    }

    return HAL_OK;
}

} // namespace SIMULATOR_NAMESPACE::interfaces
