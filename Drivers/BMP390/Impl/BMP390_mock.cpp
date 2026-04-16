#include "BMP390_mock.h"

namespace Drivers {
namespace BMP390 {

BMP390_Mock::BMP390_Mock()
    : init_returns(true),
      ping_returns(true),
      getFrame_returns(true),
      not_ready_cycles(0),
      inject_data{},
      inject_status(BMP390_STATUS_OK),
      call_init(0),
      call_ping(0),
      call_configure(0),
      call_trigger(0),
      call_getFrame(0),
      last_osr_p(OsrPressure::x1),
      last_osr_t(OsrTemp::x1),
      last_filter(IIRFilter::OFF),
      pending_(false),
      not_ready_remaining_(0)
{}

bool BMP390_Mock::init() {
    ++call_init;
    return init_returns;
}

bool BMP390_Mock::ping() {
    ++call_ping;
    return ping_returns;
}

void BMP390_Mock::configure(OsrPressure osr_p, OsrTemp osr_t, IIRFilter filter) {
    ++call_configure;
    last_osr_p  = osr_p;
    last_osr_t  = osr_t;
    last_filter = filter;
}

void BMP390_Mock::triggerMeasurement() {
    ++call_trigger;
    pending_             = true;
    not_ready_remaining_ = not_ready_cycles;
}

bool BMP390_Mock::getFrame(BaroData& out) {
    ++call_getFrame;

    if (!pending_) return false;

    if (not_ready_remaining_ > 0) {
        --not_ready_remaining_;
        return false;
    }

    if (!getFrame_returns) {
        pending_ = false;
        inject_status |= BMP390_STATUS_SPI_ERROR;
        return false;
    }

    out      = inject_data;
    pending_ = false;
    return true;
}

uint32_t BMP390_Mock::getStatus() const {
    return inject_status;
}

void BMP390_Mock::resetCounters() {
    call_init = call_ping = call_configure = 0;
    call_trigger = call_getFrame = 0;
    pending_             = false;
    not_ready_remaining_ = 0;
}

} // namespace BMP390
} // namespace Drivers
