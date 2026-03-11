
#include <gtest/gtest.h>

#include "Application/Data/data.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

using namespace flight_computer;

TEST(ApplicationData, TestAvState) {
    EXPECT_EQ( DataManager::read().state, INIT );

    DataManager::setState( ABORT_ON_GROUND );
    
    EXPECT_EQ( DataManager::read().state, ABORT_ON_GROUND );
}

TEST(StateStoreTest, DefaultStateIsInit)
{
    StateStore store;
    EXPECT_EQ(store.get(), State::INIT);
}

TEST(StateStoreTest, SetAndGetState)
{
    StateStore store;

    store.set(State::ABORT_ON_GROUND);
    EXPECT_EQ(store.get(), State::ABORT_ON_GROUND);

    store.set(State::PRESSURIZATION);
    EXPECT_EQ(store.get(), State::PRESSURIZATION);
}

//
// ✅ GpsStore Tests
//

TEST(GpsStoreTest, DefaultConstructedIsZeroInitialized)
{
    GpsStore store;
    const auto& data = store.get();

    EXPECT_EQ(data.lat, 0);
    EXPECT_EQ(data.lon, 0);
    EXPECT_EQ(data.hAcc, 0);
    EXPECT_EQ(data.numSV, 0);
}

TEST(GpsStoreTest, SetAndGetGpsData)
{
    GpsStore store;

    GpsBasicFixData fix{};
    fix.lat = 123456;
    fix.lon = 654321;
    fix.hAcc = 42;
    fix.numSV = 10;

    store.set(fix);

    const auto& result = store.get();

    EXPECT_EQ(result.lat, 123456);
    EXPECT_EQ(result.lon, 654321);
    EXPECT_EQ(result.hAcc, 42);
    EXPECT_EQ(result.numSV, 10);
}

TEST(GpsStoreTest, HelperSettersAndGettersWork)
{
    GpsStore store;

    store.setLon(123456789);
    store.setLat(987654321);
    store.setHAcc(42);
    store.setNumSV(10);

    GpsFixType fixType = GpsFixType{};  // default if enum or struct
    store.setFixType(fixType);

    GpsValidityFlags validity{};
    GpsStatusFlags flags{};

    store.setValidity(validity);
    store.setStatusFlags(flags);

    EXPECT_EQ(store.getLon(), 123456789);
    EXPECT_EQ(store.getLat(), 987654321);
    EXPECT_EQ(store.getHAcc(), 42u);
    EXPECT_EQ(store.getNumSV(), 10u);
}
//
// ✅ Full Struct Set + Helper Get Consistency
//
TEST(GpsStoreTest, SetterAndGetterVerification)
{
    GpsStore store;

    // Test int32_t fields
    store.setLon(123456789);
    store.setLat(-987654321);
    EXPECT_EQ(store.getLon(), 123456789);
    EXPECT_EQ(store.getLat(), -987654321);

    // Test uint32_t field
    store.setHAcc(4294967295u);
    EXPECT_EQ(store.getHAcc(), 4294967295u);

    // Test uint8_t field
    store.setNumSV(255);
    EXPECT_EQ(store.getNumSV(), 255);

    // Test GpsValidityFlags
    GpsValidityFlags vFlags = {true, false, true, false};
    store.setValidity(vFlags);
    GpsValidityFlags vResult = store.getValidity();
    EXPECT_TRUE(vResult.validDate);
    EXPECT_FALSE(vResult.validTime);
    EXPECT_TRUE(vResult.fullyResolved);
    EXPECT_FALSE(vResult.validMag);

    GpsStatusFlags sFlags = {
        true,           // gnssFixOK
        false,          // diffSoln
        GpsPowerSaveMode::NOT_ACTIVE, 
        true,           // headVehValid
        GpsCarrierPhaseStatus::NONE 
    };
    store.setStatusFlags(sFlags);
    GpsStatusFlags sResult = store.getStatusFlags();
    EXPECT_TRUE(sResult.gnssFixOK);
    EXPECT_FALSE(sResult.diffSoln);
    EXPECT_TRUE(sResult.headVehValid);
    EXPECT_EQ(sResult.psmState, GpsPowerSaveMode::NOT_ACTIVE);
    EXPECT_EQ(sResult.carrSoln, GpsCarrierPhaseStatus::NONE);
}

TEST(GpsStoreTest, BoundaryValueTesting)
{
    GpsStore store;

    // 1. Test Int32 boundaries
    int32_t max_int = std::numeric_limits<int32_t>::max();
    int32_t min_int = std::numeric_limits<int32_t>::min();

    store.setLon(max_int);
    EXPECT_EQ(store.getLon(), max_int);
    
    store.setLat(min_int);
    EXPECT_EQ(store.getLat(), min_int);

    // 2. Test Uint32 boundaries
    uint32_t max_uint32 = std::numeric_limits<uint32_t>::max();
    store.setHAcc(max_uint32);
    EXPECT_EQ(store.getHAcc(), max_uint32);

    // 3. Test Uint8 boundaries
    uint8_t max_uint8 = std::numeric_limits<uint8_t>::max();
    store.setNumSV(max_uint8);
    EXPECT_EQ(store.getNumSV(), max_uint8);
}

//
// ✅ GOATStore Tests
//

TEST(GOATStoreTest, LiveDataDumpIntegrity)
{
    GOATStore goat;

    // 1. Setup initial state
    goat.stateStore.set(State::ARMED);
    
    GpsBasicFixData initialFix{};
    initialFix.lat = 500;
    initialFix.lon = 600;
    goat.gpsStore.set(initialFix);

    goat.sensStatusStore.set_adxl_status(0x42);
    goat.propSensorsStore.set_chamber_pressure(100.0);
    goat.valvesStore.set_valve_prb_main_lox(true);
    goat.navigationDataStore.set_altitude(500.0);
    goat.eventStore.set_ignited(true);
    goat.batteriesStore.set_lpb_voltage(12.5f);
    goat.camsRecordingStore.set_cam_sep(true);
    goat.uplinkCmdStore.set_id(10);

    // 2. Retrieve "Live" dump
    const DataDump& dump = goat.get();

    // Verify initial values via the dump
    EXPECT_EQ(dump.av_state, State::ARMED);
    EXPECT_EQ(dump.gps_state.lat, 500);
    EXPECT_EQ(dump.gps_state.lon, 600);
    EXPECT_EQ(dump.sensStatus.adxl_status, 0x42);
    EXPECT_DOUBLE_EQ(dump.propSensors.chamber_pressure, 100.0);
    EXPECT_TRUE(dump.valves.valve_prb_main_lox);
    EXPECT_DOUBLE_EQ(dump.navigationData.altitude, 500.0);
    EXPECT_TRUE(dump.event.ignited);
    EXPECT_FLOAT_EQ(dump.batteries.lpb_voltage, 12.5f);
    EXPECT_TRUE(dump.camsRecording.cam_sep);
    EXPECT_EQ(dump.uplinkCmd.id, 10);

    // 3. Mutate the stores directly and verify the dump does NOT see the change
    // This confirms get() returns a COPY, not a live reference
    goat.stateStore.set(State::IGNITION);
    goat.gpsStore.get_ref()->lat = 999;
    goat.sensStatusStore.set_adxl_status(0xFF);
    goat.propSensorsStore.set_chamber_pressure(999.0);
    goat.valvesStore.set_valve_prb_main_lox(false);
    goat.navigationDataStore.set_altitude(9999.0);
    goat.eventStore.set_ignited(false);
    goat.batteriesStore.set_lpb_voltage(99.9f);
    goat.camsRecordingStore.set_cam_sep(false);
    goat.uplinkCmdStore.set_id(99);

    EXPECT_EQ(dump.av_state, State::ARMED);
    EXPECT_EQ(dump.gps_state.lat, 500);
    EXPECT_EQ(dump.sensStatus.adxl_status, 0x42);
    EXPECT_DOUBLE_EQ(dump.propSensors.chamber_pressure, 100.0);
    EXPECT_TRUE(dump.valves.valve_prb_main_lox);
    EXPECT_DOUBLE_EQ(dump.navigationData.altitude, 500.0);
    EXPECT_TRUE(dump.event.ignited);
    EXPECT_FLOAT_EQ(dump.batteries.lpb_voltage, 12.5f);
    EXPECT_TRUE(dump.camsRecording.cam_sep);
    EXPECT_EQ(dump.uplinkCmd.id, 10);

    // 4. Verify that get_ref() DOES return a live reference to data_
    // Modifying data_ via get_ref() should be visible directly in the reference
    DataDump* liveDump = goat.get_ref();
    liveDump->av_state = State::BURN;
    EXPECT_EQ(liveDump->av_state, State::BURN);
}

TEST(GOATStoreTest, DataDumpContainsAllStores)
{
    GOATStore goat;
    const DataDump& dump = goat.get();

    EXPECT_EQ(dump.av_state, State::INIT);
    EXPECT_EQ(dump.gps_state.lat, 0);
    EXPECT_EQ(dump.sensStatus.adxl_status, 0);
    EXPECT_EQ(dump.propSensors.N2_pressure, 0.0);
    EXPECT_EQ(dump.valves.valve_dpr_pressure_lox, false);
    EXPECT_EQ(dump.navigationData.altitude, 0.0);
    EXPECT_EQ(dump.event.ignited, false);
    EXPECT_EQ(dump.batteries.lpb_voltage, 0.0f);
    EXPECT_EQ(dump.camsRecording.cam_sep, false);
    EXPECT_EQ(dump.uplinkCmd.id, 0);
}

TEST(GOATStoreTest, DataDumpCanBeMutated)
{
    GOATStore goat;

    DataDump* dump = goat.get_ref();
    dump->sensStatus.adxl_status = 0x42;
    dump->propSensors.chamber_pressure = 500.0;
    dump->valves.valve_prb_main_lox = true;
    dump->navigationData.altitude = 1000.0;
    dump->event.ignited = true;
    dump->batteries.lpb_voltage = 12.5f;
    dump->camsRecording.cam_sep = true;
    dump->uplinkCmd.id = 5;

    const DataDump& result = *goat.get_ref();
    EXPECT_EQ(result.sensStatus.adxl_status, 0x42);
    EXPECT_DOUBLE_EQ(result.propSensors.chamber_pressure, 500.0);
    EXPECT_TRUE(result.valves.valve_prb_main_lox);
    EXPECT_DOUBLE_EQ(result.navigationData.altitude, 1000.0);
    EXPECT_TRUE(result.event.ignited);
    EXPECT_FLOAT_EQ(result.batteries.lpb_voltage, 12.5f);
    EXPECT_TRUE(result.camsRecording.cam_sep);
    EXPECT_EQ(result.uplinkCmd.id, 5);
}

TEST(NavSensorsStoreTest, DefaultConstructedIsZeroInitialized)
{
    NavSensorsStore store;
    const auto& data = store.get();

    // Verify a sample from each sub-struct
    EXPECT_FLOAT_EQ(data.adxl.x, 0.0f);
    EXPECT_FLOAT_EQ(data.bmi_gyro.z, 0.0f);
    EXPECT_DOUBLE_EQ(data.bmp.pressure, 0.0);
}

TEST(NavSensorsStoreTest, SetAndGetFullStruct)
{
    NavSensorsStore store;
    
    NavSensors data;
    data.adxl = {1.0f, 2.0f, 3.0f};
    data.bmi_accel = {4.0f, 5.0f, 6.0f};
    data.bmp = {25.5, 1013.25};

    store.set(data);
    const auto& result = store.get();

    EXPECT_FLOAT_EQ(result.adxl.x, 1.0f);
    EXPECT_FLOAT_EQ(result.bmi_accel.y, 5.0f);
    EXPECT_DOUBLE_EQ(result.bmp.pressure, 1013.25);
}

TEST(NavSensorsStoreTest, HelperSettersAndGettersWork)
{
    NavSensorsStore store;

    // 1. ADXL sensors
    adxl375_data adxl = {1.1f, 1.2f, 1.3f};
    adxl375_data adxl_aux = {2.1f, 2.2f, 2.3f};
    store.set_adxl(adxl);
    store.set_adxl_aux(adxl_aux);
    
    EXPECT_FLOAT_EQ(store.get_adxl().x, 1.1f);
    EXPECT_FLOAT_EQ(store.get_adxl_aux().y, 2.2f);

    // 2. BMI sensors (Accel and Gyro)
    bmi08_sensor_data_f b_accel = {10.0f, 11.0f, 12.0f};
    bmi08_sensor_data_f b_gyro = {0.1f, 0.2f, 0.3f};
    bmi08_sensor_data_f b_aux_accel = {20.0f, 21.0f, 22.0f};
    bmi08_sensor_data_f b_aux_gyro = {0.4f, 0.5f, 0.6f};

    store.set_bmi_accel(b_accel);
    store.set_bmi_gyro(b_gyro);
    store.set_bmi_aux_accel(b_aux_accel);
    store.set_bmi_aux_gyro(b_aux_gyro);

    EXPECT_FLOAT_EQ(store.get_bmi_accel().x, 10.0f);
    EXPECT_FLOAT_EQ(store.get_bmi_gyro().z, 0.3f);
    EXPECT_FLOAT_EQ(store.get_bmi_aux_accel().y, 21.0f);
    EXPECT_FLOAT_EQ(store.get_bmi_aux_gyro().x, 0.4f);

    // 3. BMP sensors (Pressure and Temperature)
    bmp3_data bmp = {25.0, 1013.25};
    bmp3_data bmp_aux = {26.0, 990.0};
    
    store.set_bmp(bmp);
    store.set_bmp_aux(bmp_aux);

    EXPECT_DOUBLE_EQ(store.get_bmp().temperature, 25.0);
    EXPECT_DOUBLE_EQ(store.get_bmp().pressure, 1013.25);
    EXPECT_DOUBLE_EQ(store.get_bmp_aux().temperature, 26.0);
    EXPECT_DOUBLE_EQ(store.get_bmp_aux().pressure, 990.0);
}

TEST(NavSensorsStoreTest, PrecisionAndBoundaryTesting)
{
    NavSensorsStore store;

    // Test extreme values
    float extreme_f = 1e6f;
    double extreme_d = 1e12;

    adxl375_data adxl = {extreme_f, -extreme_f, 0.0f};
    bmp3_data bmp = {extreme_d, extreme_d};

    store.set_adxl(adxl);
    store.set_bmp(bmp);

    EXPECT_FLOAT_EQ(store.get_adxl().x, extreme_f);
    EXPECT_DOUBLE_EQ(store.get_bmp().pressure, extreme_d);
}

TEST(SensStatusStoreTest, DefaultConstructedIsZeroInitialized)
{
    SensStatusStore store;
    const auto& data = store.get();

    EXPECT_EQ(data.adxl_status, 0);
    EXPECT_EQ(data.adxl_aux_status, 0);
    EXPECT_EQ(data.bmi_accel_status, 0);
    EXPECT_EQ(data.bmi_gyro_status, 0);
    EXPECT_EQ(data.bmp_status.pwr_on_rst, 0);
    EXPECT_EQ(data.bmp_status.intr.drdy, 0);
}

TEST(SensStatusStoreTest, SetAndGetFullStruct)
{
    SensStatusStore store;

    SensStatus data;
    data.adxl_status = 0x01;
    data.adxl_aux_status = 0x02;
    data.bmi_accel_status = 0x03;
    data.bmi_aux_accel_status = 0x04;
    data.bmi_gyro_status = 0x05;
    data.bmi_aux_gyro_status = 0x06;
    data.bmp_status.pwr_on_rst = 0x07;
    data.bmp_aux_status.pwr_on_rst = 0x08;

    store.set(data);
    const auto& result = store.get();

    EXPECT_EQ(result.adxl_status, 0x01);
    EXPECT_EQ(result.adxl_aux_status, 0x02);
    EXPECT_EQ(result.bmi_accel_status, 0x03);
    EXPECT_EQ(result.bmi_aux_accel_status, 0x04);
    EXPECT_EQ(result.bmi_gyro_status, 0x05);
    EXPECT_EQ(result.bmi_aux_gyro_status, 0x06);
    EXPECT_EQ(result.bmp_status.pwr_on_rst, 0x07);
    EXPECT_EQ(result.bmp_aux_status.pwr_on_rst, 0x08);
}

TEST(SensStatusStoreTest, HelperSettersAndGettersWork)
{
    SensStatusStore store;

    store.set_adxl_status(0x11);
    store.set_adxl_aux_status(0x22);
    store.set_bmi_accel_status(0x33);
    store.set_bmi_aux_accel_status(0x44);
    store.set_bmi_gyro_status(0x55);
    store.set_bmi_aux_gyro_status(0x66);

    EXPECT_EQ(store.get_adxl_status(), 0x11);
    EXPECT_EQ(store.get_adxl_aux_status(), 0x22);
    EXPECT_EQ(store.get_bmi_accel_status(), 0x33);
    EXPECT_EQ(store.get_bmi_aux_accel_status(), 0x44);
    EXPECT_EQ(store.get_bmi_gyro_status(), 0x55);
    EXPECT_EQ(store.get_bmi_aux_gyro_status(), 0x66);
}

TEST(SensStatusStoreTest, BmpStatusGettersAndSetters)
{
    SensStatusStore store;

    bmp3_status bmp;
    bmp.intr.fifo_wm = 1;
    bmp.intr.fifo_full = 0;
    bmp.intr.drdy = 1;
    bmp.sensor.cmd_rdy = 1;
    bmp.sensor.drdy_press = 0;
    bmp.sensor.drdy_temp = 1;
    bmp.err.fatal = 0;
    bmp.err.cmd = 0;
    bmp.err.conf = 1;
    bmp.pwr_on_rst = 0xAA;

    store.set_bmp_status(bmp);

    bmp3_status result = store.get_bmp_status();
    EXPECT_EQ(result.intr.fifo_wm, 1);
    EXPECT_EQ(result.intr.fifo_full, 0);
    EXPECT_EQ(result.intr.drdy, 1);
    EXPECT_EQ(result.sensor.cmd_rdy, 1);
    EXPECT_EQ(result.sensor.drdy_press, 0);
    EXPECT_EQ(result.sensor.drdy_temp, 1);
    EXPECT_EQ(result.err.fatal, 0);
    EXPECT_EQ(result.err.cmd, 0);
    EXPECT_EQ(result.err.conf, 1);
    EXPECT_EQ(result.pwr_on_rst, 0xAA);
}

TEST(SensStatusStoreTest, BmpAuxStatusGettersAndSetters)
{
    SensStatusStore store;

    bmp3_status bmp_aux;
    bmp_aux.intr.drdy = 1;
    bmp_aux.pwr_on_rst = 0xBB;

    store.set_bmp_aux_status(bmp_aux);

    bmp3_status result = store.get_bmp_aux_status();
    EXPECT_EQ(result.intr.drdy, 1);
    EXPECT_EQ(result.pwr_on_rst, 0xBB);
}

TEST(PropSensorsStoreTest, DefaultConstructedIsZeroInitialized)
{
    PropSensorsStore store;
    const auto& data = store.get();

    EXPECT_DOUBLE_EQ(data.N2_pressure, 0.0);
    EXPECT_DOUBLE_EQ(data.fuel_pressure, 0.0);
    EXPECT_DOUBLE_EQ(data.LOX_pressure, 0.0);
    EXPECT_DOUBLE_EQ(data.chamber_pressure, 0.0);
    EXPECT_DOUBLE_EQ(data.fuel_level, 0.0);
    EXPECT_DOUBLE_EQ(data.LOX_level, 0.0);
    EXPECT_EQ(data.PR_state, 0u);
}

TEST(PropSensorsStoreTest, SetAndGetFullStruct)
{
    PropSensorsStore store;

    PropSensors data;
    data.N2_pressure = 3000.0;
    data.fuel_pressure = 250.5;
    data.LOX_pressure = 280.75;
    data.igniter_pressure = 100.0;
    data.LOX_inj_pressure = 150.25;
    data.fuel_inj_pressure = 140.0;
    data.chamber_pressure = 500.0;
    data.fuel_level = 75.5;
    data.LOX_level = 80.0;
    data.N2_temperature = 25.0;
    data.fuel_temperature = 20.0;
    data.LOX_temperature = -180.0;
    data.igniter_temperature = 1500.0;
    data.fuel_inj_temperature = 30.0;
    data.fuel_inj_cooling_temperature = 25.0;
    data.LOX_inj_temperature = -175.0;
    data.chamber_temperature = 2500.0;
    data.PR_state = 42;

    store.set(data);
    const auto& result = store.get();

    EXPECT_DOUBLE_EQ(result.N2_pressure, 3000.0);
    EXPECT_DOUBLE_EQ(result.fuel_pressure, 250.5);
    EXPECT_DOUBLE_EQ(result.LOX_pressure, 280.75);
    EXPECT_DOUBLE_EQ(result.chamber_pressure, 500.0);
    EXPECT_DOUBLE_EQ(result.fuel_level, 75.5);
    EXPECT_DOUBLE_EQ(result.LOX_level, 80.0);
    EXPECT_DOUBLE_EQ(result.LOX_temperature, -180.0);
    EXPECT_DOUBLE_EQ(result.chamber_temperature, 2500.0);
    EXPECT_EQ(result.PR_state, 42u);
}

TEST(PropSensorsStoreTest, HelperSettersAndGettersWork)
{
    PropSensorsStore store;

    store.set_N2_pressure(3100.0);
    store.set_fuel_pressure(255.5);
    store.set_LOX_pressure(285.75);
    store.set_igniter_pressure(105.0);
    store.set_LOX_inj_pressure(155.25);
    store.set_fuel_inj_pressure(145.0);
    store.set_chamber_pressure(510.0);
    store.set_fuel_level(76.5);
    store.set_LOX_level(81.0);
    store.set_N2_temperature(26.0);
    store.set_fuel_temperature(21.0);
    store.set_LOX_temperature(-179.0);
    store.set_igniter_temperature(1510.0);
    store.set_fuel_inj_temperature(31.0);
    store.set_fuel_inj_cooling_temperature(26.0);
    store.set_LOX_inj_temperature(-174.0);
    store.set_chamber_temperature(2510.0);
    store.set_PR_state(123);

    EXPECT_DOUBLE_EQ(store.get_N2_pressure(), 3100.0);
    EXPECT_DOUBLE_EQ(store.get_fuel_pressure(), 255.5);
    EXPECT_DOUBLE_EQ(store.get_LOX_pressure(), 285.75);
    EXPECT_DOUBLE_EQ(store.get_igniter_pressure(), 105.0);
    EXPECT_DOUBLE_EQ(store.get_LOX_inj_pressure(), 155.25);
    EXPECT_DOUBLE_EQ(store.get_fuel_inj_pressure(), 145.0);
    EXPECT_DOUBLE_EQ(store.get_chamber_pressure(), 510.0);
    EXPECT_DOUBLE_EQ(store.get_fuel_level(), 76.5);
    EXPECT_DOUBLE_EQ(store.get_LOX_level(), 81.0);
    EXPECT_DOUBLE_EQ(store.get_N2_temperature(), 26.0);
    EXPECT_DOUBLE_EQ(store.get_fuel_temperature(), 21.0);
    EXPECT_DOUBLE_EQ(store.get_LOX_temperature(), -179.0);
    EXPECT_DOUBLE_EQ(store.get_igniter_temperature(), 1510.0);
    EXPECT_DOUBLE_EQ(store.get_fuel_inj_temperature(), 31.0);
    EXPECT_DOUBLE_EQ(store.get_fuel_inj_cooling_temperature(), 26.0);
    EXPECT_DOUBLE_EQ(store.get_LOX_inj_temperature(), -174.0);
    EXPECT_DOUBLE_EQ(store.get_chamber_temperature(), 2510.0);
    EXPECT_EQ(store.get_PR_state(), 123u);
}

TEST(PropSensorsStoreTest, BoundaryValueTesting)
{
    PropSensorsStore store;

    double max_double = std::numeric_limits<double>::max();
    double min_double = -std::numeric_limits<double>::max();
    uint32_t max_uint32 = std::numeric_limits<uint32_t>::max();

    store.set_N2_pressure(max_double);
    store.set_fuel_pressure(min_double);
    store.set_PR_state(max_uint32);

    EXPECT_DOUBLE_EQ(store.get_N2_pressure(), max_double);
    EXPECT_DOUBLE_EQ(store.get_fuel_pressure(), min_double);
    EXPECT_EQ(store.get_PR_state(), max_uint32);
}

TEST(ValvesStoreTest, DefaultConstructedIsFalse)
{
    ValvesStore store;
    const auto& data = store.get();

    EXPECT_FALSE(data.valve_dpr_pressure_lox);
    EXPECT_FALSE(data.valve_dpr_pressure_fuel);
    EXPECT_FALSE(data.valve_dpr_vent_copv);
    EXPECT_FALSE(data.valve_dpr_vent_lox);
    EXPECT_FALSE(data.valve_dpr_vent_fuel);
    EXPECT_FALSE(data.valve_prb_main_lox);
    EXPECT_FALSE(data.valve_prb_main_fuel);
}

TEST(ValvesStoreTest, SetAndGetFullStruct)
{
    ValvesStore store;

    Valves data;
    data.valve_dpr_pressure_lox = true;
    data.valve_dpr_pressure_fuel = true;
    data.valve_dpr_vent_copv = false;
    data.valve_dpr_vent_lox = true;
    data.valve_dpr_vent_fuel = false;
    data.valve_prb_main_lox = true;
    data.valve_prb_main_fuel = true;

    store.set(data);
    const auto& result = store.get();

    EXPECT_TRUE(result.valve_dpr_pressure_lox);
    EXPECT_TRUE(result.valve_dpr_pressure_fuel);
    EXPECT_FALSE(result.valve_dpr_vent_copv);
    EXPECT_TRUE(result.valve_dpr_vent_lox);
    EXPECT_FALSE(result.valve_dpr_vent_fuel);
    EXPECT_TRUE(result.valve_prb_main_lox);
    EXPECT_TRUE(result.valve_prb_main_fuel);
}

TEST(ValvesStoreTest, HelperSettersAndGettersWork)
{
    ValvesStore store;

    store.set_valve_dpr_pressure_lox(true);
    store.set_valve_dpr_pressure_fuel(true);
    store.set_valve_dpr_vent_copv(false);
    store.set_valve_dpr_vent_lox(true);
    store.set_valve_dpr_vent_fuel(false);
    store.set_valve_prb_main_lox(true);
    store.set_valve_prb_main_fuel(true);

    EXPECT_TRUE(store.get_valve_dpr_pressure_lox());
    EXPECT_TRUE(store.get_valve_dpr_pressure_fuel());
    EXPECT_FALSE(store.get_valve_dpr_vent_copv());
    EXPECT_TRUE(store.get_valve_dpr_vent_lox());
    EXPECT_FALSE(store.get_valve_dpr_vent_fuel());
    EXPECT_TRUE(store.get_valve_prb_main_lox());
    EXPECT_TRUE(store.get_valve_prb_main_fuel());
}

TEST(NavigationDataStoreTest, DefaultConstructedIsZeroInitialized)
{
    NavigationDataStore store;
    const auto& data = store.get();

    EXPECT_DOUBLE_EQ(data.position_kalman.x, 0.0);
    EXPECT_DOUBLE_EQ(data.position_kalman.y, 0.0);
    EXPECT_DOUBLE_EQ(data.position_kalman.z, 0.0);
    EXPECT_DOUBLE_EQ(data.speed.x, 0.0);
    EXPECT_DOUBLE_EQ(data.accel.x, 0.0);
    EXPECT_DOUBLE_EQ(data.attitude.x, 0.0);
    EXPECT_DOUBLE_EQ(data.course, 0.0);
    EXPECT_DOUBLE_EQ(data.altitude, 0.0);
    EXPECT_DOUBLE_EQ(data.baro.temperature, 0.0);
    EXPECT_DOUBLE_EQ(data.baro.pressure, 0.0);
}

TEST(NavigationDataStoreTest, SetAndGetFullStruct)
{
    NavigationDataStore store;

    NavigationData data;
    data.position_kalman = {1.0, 2.0, 3.0};
    data.speed = {10.0, 20.0, 30.0};
    data.accel = {1.5, 2.5, 3.5};
    data.attitude = {0.1, 0.2, 0.3};
    data.course = 45.0;
    data.altitude = 1000.0;
    data.baro = {25.0, 1013.25};

    store.set(data);
    const auto& result = store.get();

    EXPECT_DOUBLE_EQ(result.position_kalman.x, 1.0);
    EXPECT_DOUBLE_EQ(result.position_kalman.y, 2.0);
    EXPECT_DOUBLE_EQ(result.position_kalman.z, 3.0);
    EXPECT_DOUBLE_EQ(result.speed.x, 10.0);
    EXPECT_DOUBLE_EQ(result.speed.y, 20.0);
    EXPECT_DOUBLE_EQ(result.speed.z, 30.0);
    EXPECT_DOUBLE_EQ(result.accel.x, 1.5);
    EXPECT_DOUBLE_EQ(result.attitude.z, 0.3);
    EXPECT_DOUBLE_EQ(result.course, 45.0);
    EXPECT_DOUBLE_EQ(result.altitude, 1000.0);
    EXPECT_DOUBLE_EQ(result.baro.temperature, 25.0);
    EXPECT_DOUBLE_EQ(result.baro.pressure, 1013.25);
}

TEST(NavigationDataStoreTest, HelperSettersAndGettersWork)
{
    NavigationDataStore store;

    Vector3 pos = {100.0, 200.0, 300.0};
    Vector3 vel = {10.0, 20.0, 30.0};
    Vector3 acc = {1.0, 2.0, 3.0};
    Vector3 att = {0.5, 0.6, 0.7};

    store.set_position_kalman(pos);
    store.set_speed(vel);
    store.set_accel(acc);
    store.set_attitude(att);
    store.set_course(90.0);
    store.set_altitude(5000.0);
    store.set_baro({20.0, 950.0});

    EXPECT_DOUBLE_EQ(store.get_position_kalman().x, 100.0);
    EXPECT_DOUBLE_EQ(store.get_speed().y, 20.0);
    EXPECT_DOUBLE_EQ(store.get_accel().z, 3.0);
    EXPECT_DOUBLE_EQ(store.get_attitude().x, 0.5);
    EXPECT_DOUBLE_EQ(store.get_course(), 90.0);
    EXPECT_DOUBLE_EQ(store.get_altitude(), 5000.0);
    EXPECT_DOUBLE_EQ(store.get_baro().temperature, 20.0);
    EXPECT_DOUBLE_EQ(store.get_baro().pressure, 950.0);
}

TEST(NavigationDataStoreTest, Vector3Norm)
{
    Vector3 v = {3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(v.norm(), 5.0);
}

TEST(EventStoreTest, DefaultConstructedIsFalse)
{
    EventStore store;
    const auto& data = store.get();

    EXPECT_FALSE(data.command_updated);
    EXPECT_FALSE(data.calibrated);
    EXPECT_FALSE(data.dpr_eth_ready);
    EXPECT_FALSE(data.dpr_eth_pressure_ok);
    EXPECT_FALSE(data.dpr_lox_ready);
    EXPECT_FALSE(data.dpr_lox_pressure_ok);
    EXPECT_FALSE(data.prb_ready);
    EXPECT_FALSE(data.trb_ready);
    EXPECT_FALSE(data.ignited);
    EXPECT_FALSE(data.seperated);
    EXPECT_FALSE(data.chute_unreefed);
    EXPECT_FALSE(data.ignition_failed);
    EXPECT_FALSE(data.catastrophic_failure);
}

TEST(EventStoreTest, SetAndGetFullStruct)
{
    EventStore store;

    Event data;
    data.command_updated = true;
    data.calibrated = true;
    data.dpr_eth_ready = true;
    data.dpr_eth_pressure_ok = true;
    data.dpr_lox_ready = true;
    data.dpr_lox_pressure_ok = true;
    data.prb_ready = true;
    data.trb_ready = true;
    data.ignited = true;
    data.seperated = true;
    data.chute_unreefed = true;
    data.ignition_failed = false;
    data.catastrophic_failure = false;

    store.set(data);
    const auto& result = store.get();

    EXPECT_TRUE(result.command_updated);
    EXPECT_TRUE(result.calibrated);
    EXPECT_TRUE(result.dpr_eth_ready);
    EXPECT_TRUE(result.dpr_eth_pressure_ok);
    EXPECT_TRUE(result.dpr_lox_ready);
    EXPECT_TRUE(result.dpr_lox_pressure_ok);
    EXPECT_TRUE(result.prb_ready);
    EXPECT_TRUE(result.trb_ready);
    EXPECT_TRUE(result.ignited);
    EXPECT_TRUE(result.seperated);
    EXPECT_TRUE(result.chute_unreefed);
    EXPECT_FALSE(result.ignition_failed);
    EXPECT_FALSE(result.catastrophic_failure);
}

TEST(EventStoreTest, HelperSettersAndGettersWork)
{
    EventStore store;

    store.set_command_updated(true);
    store.set_calibrated(true);
    store.set_dpr_eth_ready(true);
    store.set_dpr_eth_pressure_ok(true);
    store.set_dpr_lox_ready(true);
    store.set_dpr_lox_pressure_ok(true);
    store.set_prb_ready(true);
    store.set_trb_ready(true);
    store.set_ignited(true);
    store.set_seperated(true);
    store.set_chute_unreefed(true);
    store.set_ignition_failed(false);
    store.set_catastrophic_failure(false);

    EXPECT_TRUE(store.get_command_updated());
    EXPECT_TRUE(store.get_calibrated());
    EXPECT_TRUE(store.get_dpr_eth_ready());
    EXPECT_TRUE(store.get_dpr_eth_pressure_ok());
    EXPECT_TRUE(store.get_dpr_lox_ready());
    EXPECT_TRUE(store.get_dpr_lox_pressure_ok());
    EXPECT_TRUE(store.get_prb_ready());
    EXPECT_TRUE(store.get_trb_ready());
    EXPECT_TRUE(store.get_ignited());
    EXPECT_TRUE(store.get_seperated());
    EXPECT_TRUE(store.get_chute_unreefed());
    EXPECT_FALSE(store.get_ignition_failed());
    EXPECT_FALSE(store.get_catastrophic_failure());
}

TEST(BatteriesStoreTest, DefaultConstructedIsZeroInitialized)
{
    BatteriesStore store;
    const auto& data = store.get();

    EXPECT_FLOAT_EQ(data.lpb_voltage, 0.0f);
    EXPECT_FLOAT_EQ(data.hpb_voltage, 0.0f);
}

TEST(BatteriesStoreTest, SetAndGetFullStruct)
{
    BatteriesStore store;

    Batteries data;
    data.lpb_voltage = 12.5f;
    data.hpb_voltage = 48.0f;

    store.set(data);
    const auto& result = store.get();

    EXPECT_FLOAT_EQ(result.lpb_voltage, 12.5f);
    EXPECT_FLOAT_EQ(result.hpb_voltage, 48.0f);
}

TEST(BatteriesStoreTest, HelperSettersAndGettersWork)
{
    BatteriesStore store;

    store.set_lpb_voltage(11.0f);
    store.set_hpb_voltage(50.0f);

    EXPECT_FLOAT_EQ(store.get_lpb_voltage(), 11.0f);
    EXPECT_FLOAT_EQ(store.get_hpb_voltage(), 50.0f);
}

TEST(CamsRecordingStoreTest, DefaultConstructedIsFalse)
{
    CamsRecordingStore store;
    const auto& data = store.get();

    EXPECT_FALSE(data.cam_sep);
    EXPECT_FALSE(data.cam_up);
    EXPECT_FALSE(data.cam_down);
}

TEST(CamsRecordingStoreTest, SetAndGetFullStruct)
{
    CamsRecordingStore store;

    CamsRecording data;
    data.cam_sep = true;
    data.cam_up = false;
    data.cam_down = true;

    store.set(data);
    const auto& result = store.get();

    EXPECT_TRUE(result.cam_sep);
    EXPECT_FALSE(result.cam_up);
    EXPECT_TRUE(result.cam_down);
}

TEST(CamsRecordingStoreTest, HelperSettersAndGettersWork)
{
    CamsRecordingStore store;

    store.set_cam_sep(true);
    store.set_cam_up(false);
    store.set_cam_down(true);

    EXPECT_TRUE(store.get_cam_sep());
    EXPECT_FALSE(store.get_cam_up());
    EXPECT_TRUE(store.get_cam_down());
}

TEST(UplinkCmdStoreTest, DefaultConstructedIsZero)
{
    UplinkCmdStore store;
    const auto& data = store.get();

    EXPECT_EQ(data.id, 0);
    EXPECT_EQ(data.value, 0);
}

TEST(UplinkCmdStoreTest, SetAndGetFullStruct)
{
    UplinkCmdStore store;

    UplinkCmd data;
    data.id = 42;
    data.value = 123;

    store.set(data);
    const auto& result = store.get();

    EXPECT_EQ(result.id, 42);
    EXPECT_EQ(result.value, 123);
}

TEST(UplinkCmdStoreTest, HelperSettersAndGettersWork)
{
    UplinkCmdStore store;

    store.set_id(10);
    store.set_value(20);

    EXPECT_EQ(store.get_id(), 10);
    EXPECT_EQ(store.get_value(), 20);
}
