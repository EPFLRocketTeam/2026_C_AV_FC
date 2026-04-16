#pragma once

#include "../BMP390.h"

namespace Drivers {
namespace BMP390 {

class BMP390_Mock : public BMP390_Interface {
public:
    BMP390_Mock();

    // ===== Interface =====
    bool     init()                                              override;
    bool     ping()                                              override;
    void     configure(OsrPressure osr_p, OsrTemp osr_t,
                       IIRFilter filter)                         override;
    void     triggerMeasurement()                                override;
    bool     getFrame(BaroData& out)                             override;
    uint32_t getStatus() const                                   override;

    // ===== Injectable behaviour =====
    bool     init_returns     = true;
    bool     ping_returns     = true;
    bool     getFrame_returns = true;
    int      not_ready_cycles = 0;   // getFrame() returns false this many times before yielding data
    BaroData inject_data      = {};
    uint32_t inject_status    = BMP390_STATUS_OK;

    // ===== Call counters =====
    int call_init      = 0;
    int call_ping      = 0;
    int call_configure = 0;
    int call_trigger   = 0;
    int call_getFrame  = 0;

    // ===== Last captured configure() arguments =====
    OsrPressure last_osr_p  = OsrPressure::x1;
    OsrTemp     last_osr_t  = OsrTemp::x1;
    IIRFilter   last_filter = IIRFilter::OFF;

    // ===== Utilities =====
    void resetCounters();
    bool isPending() const { return pending_; }

private:
    bool pending_             = false;
    int  not_ready_remaining_ = 0;
};

} // namespace BMP390
} // namespace Drivers
