
#pragma once

#include "Drivers/TMP1075/TMP1075.hpp"
#include "Application/app_printf.h"
#include "Application/app_timebase.h"
#include "Application/Data/data.hpp"

extern I2C_HandleTypeDef hi2c4;              // whichever I2C bus TMP1075 is on

static constexpr uint8_t TMP1075_ADDR7      = 0x4E;  // found via I2C bus scan (schematic doc note was off by one)
static constexpr bool    TMP1075_HAS_DIE_ID = true;  // TMP1075DSG has the die-ID register

using namespace Drivers::TMP1075;

struct FCTemperatureModule {
private:
    static constexpr TMP1075_Driver::Config createConfig () {
        TMP1075_Driver::Config cfg;
        
        cfg.hi2c          = &hi2c4;
        cfg.address7      = TMP1075_ADDR7;
        cfg.checkDeviceId = TMP1075_HAS_DIE_ID;
        cfg.opMode        = OpMode::OneShot;
        cfg.commandRateHz = 10;

        return cfg;
    }

    TMP1075_Driver driver = TMP1075_Driver(createConfig());

    bool did_init_     = false;
    bool is_preparing_ = false;
public:
    void init () {
        if (!driver.init()) {
            app_printf("[TMP1075] Init failure...\n");
            return ;
        }
        if (!driver.ping()) {
            app_printf("[TMP1075] Ping failure...\n");
            return ;
        }

        driver.configure(
            ConversionRate::ms27_5,
            FaultCount::f1,
            AlertPolarity::ActiveLow,
            AlertMode::Comparator
        );

        if ((driver.getStatus() & TMP1075_STATUS_CONFIG_ERROR) != 0) {
            app_printf("[TMP1075] Configuration failure...\n");
            return ;
        }

        did_init_ = true;
    }
    void tick () {
        if (!did_init_) {
            RUN_EVERY(10'000) app_printf("[TMP1075] Failure to init, did not attemp read.\n");
            return ;
        } 

        RUN_EVERY(500) {
            if (is_preparing_) {
                is_preparing_ = false;

                TempData dump;

                if (!driver.getFrame(dump)) {
                    RUN_EVERY(10'000) app_printf("[TMP1075] Get frame has failed.\n");
                    return ;
                }

                flight_computer::GOATStore::get_instance().setFcTemperature(dump.temperature_c);
                RUN_EVERY(1'000) app_printf("[TMP1075] fc_temperature = %f\n", dump.temperature_c);
            } else {
                if (!driver.triggerConversion()) {
                    RUN_EVERY(10'000) app_printf("[TMP1075] Trigger has failed.\n");
                    return ;
                }
                is_preparing_ = true;
            }
        }
    }
};
