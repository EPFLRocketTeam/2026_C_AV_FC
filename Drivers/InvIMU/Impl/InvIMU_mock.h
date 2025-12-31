#pragma once

#include "../InvIMU.h"
#include <queue>
#include <vector>
#include <cstdint>

namespace Drivers {
namespace InvIMU {

class InvIMU_Mock : public InvIMU_Interface {
public:
    InvIMU_Mock();

    // ===== Interface =====
    bool init() override;
    bool ping() override;

    void configure(AccelRange ar, GyroRange gr, ODR odr) override;
    void configureFifo() override;
    void enableFsync() override;

    bool getFrame(IMUData& out_data) override;

    void onInterrupt() override;
    void tick() override;
    void onDmaComplete() override;

    // ===== Test helpers =====
    void addMockSample(IMUData sample);
    void addMockSamples(const std::vector<IMUData>& samples);

    void setFifoWatermark(size_t watermark);
    void setMaxFifoSize(size_t size);

    void injectSpiError();
    void clearErrors();

    void setBaseTimestamp(uint64_t ts);

    bool hasFifoOverflow() const;

    // ===== Getters for tests =====
    AccelRange getAccelRange() const { return _accel; }
    GyroRange  getGyroRange()  const { return _gyro; }
    ODR        getOdr()        const { return _odr; }
    bool       isFifoConfigured() const { return _fifo_configured; }
    bool       isFsyncEnabled()   const { return _fsync_enabled; }

private:
    // Config state
    AccelRange _accel;
    GyroRange  _gyro;
    ODR        _odr;

    bool _fifo_configured;
    bool _fsync_enabled;

    // FIFO / DMA simulation
    std::queue<IMUData> _fifo;
    size_t _fifo_watermark;
    size_t _fifo_max_size;

    bool _interrupt_pending;
    bool _dma_in_progress;

    // Error simulation
    bool _spi_error;
    bool _fifo_overflow;

    // Timestamp generation
    uint64_t _next_timestamp;
};

} // namespace InvIMU
} // namespace Drivers
