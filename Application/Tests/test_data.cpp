
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

    // 2. Retrieve "Live" dump
    const DataDump& dump = goat.get();

    // Verify initial values via the dump
    EXPECT_EQ(*dump.av_state, State::ARMED);
    EXPECT_EQ(dump.gps_state->lat, 500);
    EXPECT_EQ(dump.gps_state->lon, 600);

    // 3. Mutate the stores directly and verify the dump sees the change
    // This confirms the "Live Reference" behavior
    goat.stateStore.set(State::IGNITION);
    goat.gpsStore.get_ref()->lat = 999;

    EXPECT_EQ(*dump.av_state, State::IGNITION);
    EXPECT_EQ(dump.gps_state->lat, 999);
}


