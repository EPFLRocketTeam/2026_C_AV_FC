#pragma once

#include "Application/Data/data.hpp"
#include "Application/app_printf.h"
#include "Drivers/INA228/INA228.h"
#include <cstddef>
#include <cstdint>

// Battery telemetry doesn't need control-loop rate; each rail is 2 blocking
// I2C register reads, so polling every super-loop tick would waste time.
// Gate it in the module instead of at the call site.
#ifndef BATTERY_MODULE_PERIOD_MS
#define BATTERY_MODULE_PERIOD_MS 100u
#endif

#ifndef BATTERY_MODULE_ADDR_LPB1
#define BATTERY_MODULE_ADDR_LPB1       0x40
#endif
#ifndef BATTERY_MODULE_ADDR_LPB2
#define BATTERY_MODULE_ADDR_LPB2       0x48
#endif
#ifndef BATTERY_MODULE_ADDR_VOUT_5V_1
#define BATTERY_MODULE_ADDR_VOUT_5V_1  0x41
#endif
#ifndef BATTERY_MODULE_ADDR_VOUT_5V_2
#define BATTERY_MODULE_ADDR_VOUT_5V_2  0x49
#endif
#ifndef BATTERY_MODULE_ADDR_HPB_MAIN
#define BATTERY_MODULE_ADDR_HPB_MAIN   0x44
#endif
#ifndef BATTERY_MODULE_ADDR_HPB_BACKUP
#define BATTERY_MODULE_ADDR_HPB_BACKUP 0x46
#endif
#ifndef BATTERY_MODULE_ADDR_VOUT_24V
#define BATTERY_MODULE_ADDR_VOUT_24V   0x45
#endif

#ifndef BATTERY_MODULE_SHUNT_OHMS
#define BATTERY_MODULE_SHUNT_OHMS 0.010f
#endif
#ifndef BATTERY_MODULE_MAX_CURRENT_A
#define BATTERY_MODULE_MAX_CURRENT_A 20.0f
#endif

class BatteryModule {
public:
    static constexpr size_t kNumRails = 7;

    enum RailIndex : size_t {
        kLpb1 = 0,
        kLpb2,
        kVout5v1,
        kVout5v2,
        kHpbMain,
        kHpbBackup,
        kVout24v,
    };

    explicit BatteryModule(I2C_HandleTypeDef *hi2c)
      : rails_{
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_LPB1,       hi2c),
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_LPB2,       hi2c),
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_VOUT_5V_1,  hi2c),
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_VOUT_5V_2,  hi2c),
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_HPB_MAIN,   hi2c),
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_HPB_BACKUP, hi2c),
            Drivers::INA228::INA228_Driver(BATTERY_MODULE_ADDR_VOUT_24V,   hi2c),
        } {}

    // Returns true only if every rail's INA228 responded and was calibrated.
    // A rail that fails init() is left disabled -- update() skips it and its
    // telemetry field stays at its last (or zero) value instead of stalling
    // the others.
    bool init() {
        bool all_ok = true;
        for (size_t i = 0; i < kNumRails; ++i) {
            rail_ok_[i] = rails_[i].begin() &&
                (rails_[i].setMaxCurrentShunt(BATTERY_MODULE_MAX_CURRENT_A,
                                               BATTERY_MODULE_SHUNT_OHMS) == 0);
            if (rail_ok_[i]) {
                rails_[i].setAverage(Drivers::INA228::INA228_4_SAMPLES);
                rails_[i].setMode(Drivers::INA228::INA228_MODE_CONT_BUS_SHUNT);
            }
            all_ok &= rail_ok_[i];
        }

        const char* ok = "ok"; const char* no = "no";
        app_printf("\n\nBattery Status\n");
        app_printf("LPB1 LPB2 5V1 5V2 HPB-M HPB-B 24V\n");
        app_printf("%s   %s   %s  %s  %s    %s    %s\n\n",
            rail_ok_[kLpb1] ? ok : no,
            rail_ok_[kLpb2] ? ok : no,
            rail_ok_[kVout5v1] ? ok : no,
            rail_ok_[kVout5v2] ? ok : no,
            rail_ok_[kHpbMain] ? ok : no,
            rail_ok_[kHpbBackup] ? ok : no,
            rail_ok_[kVout24v] ? ok : no
        );

        return all_ok;
    }

    bool railHealthy(size_t rail_index) const {
        return rail_index < kNumRails && rail_ok_[rail_index];
    }

    void update(uint32_t tick_ms) {
        if (tick_ms < next_update_ms_) return;
        next_update_ms_ = tick_ms + BATTERY_MODULE_PERIOD_MS;

        flight_computer::BatteriesStore &store =
            flight_computer::GOATStore::get_instance().batteriesStore;

        if (rail_ok_[kLpb1]) {
            store.set_lpb1_voltage(rails_[kLpb1].getBusVoltage());
            store.set_lpb1_current(rails_[kLpb1].getCurrent());
        }
        if (rail_ok_[kLpb2]) {
            store.set_lpb2_voltage(rails_[kLpb2].getBusVoltage());
            store.set_lpb2_current(rails_[kLpb2].getCurrent());
        }

        if (rail_ok_[kVout5v1]) {
            store.set_vout1_5v_voltage(rails_[kVout5v1].getBusVoltage());
            store.set_vout1_5v_current(rails_[kVout5v1].getCurrent());
        }
        if (rail_ok_[kVout5v2]) {
            store.set_vout2_5v_voltage(rails_[kVout5v2].getBusVoltage());
            store.set_vout2_5v_current(rails_[kVout5v2].getCurrent());
        }

        if (rail_ok_[kHpbMain]) {
            store.set_hpb_main_voltage(rails_[kHpbMain].getBusVoltage());
            store.set_hpb_main_current(rails_[kHpbMain].getCurrent());
        }
        if (rail_ok_[kHpbBackup]) {
            store.set_hpb_backup_voltage(rails_[kHpbBackup].getBusVoltage());
            store.set_hpb_backup_current(rails_[kHpbBackup].getCurrent());
        }
        if (rail_ok_[kVout24v]) {
            store.set_vout_24v_voltage(rails_[kVout24v].getBusVoltage());
            store.set_vout_24v_current(rails_[kVout24v].getCurrent());
        }
    }

private:
    Drivers::INA228::INA228_Driver rails_[kNumRails];
    bool rail_ok_[kNumRails] = {false, false, false, false, false, false, false};
    uint32_t next_update_ms_ = 0;
};
