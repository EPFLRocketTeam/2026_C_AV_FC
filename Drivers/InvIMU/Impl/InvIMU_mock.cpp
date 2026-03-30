#include "InvIMU_mock.h"

namespace Drivers {
namespace InvIMU {

InvIMU_Mock::InvIMU_Mock()
    : _accel(AccelRange::_32G),
      _gyro(GyroRange::_4000DPS),
      _odr(ODR::_100Hz),
      _fifo_configured(false),
      _fsync_enabled(false),
      _fifo_watermark(32),
      _fifo_max_size(512),
      _interrupt_pending(false),
      _dma_in_progress(false),
      _spi_error(false),
      _fifo_overflow(false),
      _next_timestamp(0)
{}

bool InvIMU_Mock::init() { return true; }
bool InvIMU_Mock::ping() { return true; }

void InvIMU_Mock::configure(AccelRange ar, GyroRange gr, ODR odr) {
    _accel = ar;
    _gyro  = gr;
    _odr   = odr;
}

void InvIMU_Mock::configureFifo() {
    _fifo_configured = true;
}

void InvIMU_Mock::enableFsync() {
    _fsync_enabled = true;
}

void InvIMU_Mock::addMockSample(IMUData sample) {
    if (_fifo.size() >= _fifo_max_size) {
        _fifo_overflow = true;
        return;
    }

    if (sample.timestamp_us == 0) {
        sample.timestamp_us = _next_timestamp;
        _next_timestamp += 156; // ~6.4 kHz
    }

    _fifo.push(sample);
}

void InvIMU_Mock::addMockSamples(const std::vector<IMUData>& samples) {
    for (auto s : samples) {
        addMockSample(s);
    }
}

void InvIMU_Mock::onInterrupt() {
    if (_fifo.size() >= _fifo_watermark) {
        _interrupt_pending = true;
    }
}

void InvIMU_Mock::tick() {
    if (_interrupt_pending && !_dma_in_progress) {
        _dma_in_progress = true;
        _interrupt_pending = false;
    }
}

void InvIMU_Mock::onDmaComplete() {
    if (_spi_error) {
        _dma_in_progress = false;
        return;
    }
    _dma_in_progress = false;
}

bool InvIMU_Mock::getFrame(IMUData& out_data) {
    if (_dma_in_progress) return false;
    if (_fifo.empty()) return false;
    if (_spi_error) return false;

    out_data = _fifo.front();
    _fifo.pop();
    return true;
}

void InvIMU_Mock::setFifoWatermark(size_t watermark) {
    _fifo_watermark = watermark;
}

void InvIMU_Mock::setMaxFifoSize(size_t size) {
    _fifo_max_size = size;
}

void InvIMU_Mock::injectSpiError() {
    _spi_error = true;
}

void InvIMU_Mock::clearErrors() {
    _spi_error = false;
    _fifo_overflow = false;
}

void InvIMU_Mock::setBaseTimestamp(uint64_t ts) {
    _next_timestamp = ts;
}

bool InvIMU_Mock::hasFifoOverflow() const {
    return _fifo_overflow;
}

} // namespace InvIMU
} // namespace Drivers
