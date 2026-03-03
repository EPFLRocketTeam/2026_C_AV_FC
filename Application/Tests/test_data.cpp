
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

    // Set all fields individually
    store.setLon(123456789);
    store.setLat(987654321);
    store.setHAcc(42);
    store.setNumSV(10);

    // Use dummy values for enum/flags (adapt if needed)
    GpsFixType fixType = static_cast<GpsFixType>(1);
    GpsValidityFlags validity = static_cast<GpsValidityFlags>(2);
    GpsStatusFlags flags = static_cast<GpsStatusFlags>(3);

    store.setFixType(fixType);
    store.setValidity(validity);
    store.setStatusFlags(flags);

    // Check all getters
    EXPECT_EQ(store.getLon(), 123456789);
    EXPECT_EQ(store.getLat(), 987654321);
    EXPECT_EQ(store.getHAcc(), 42u);
    EXPECT_EQ(store.getNumSV(), 10u);
    EXPECT_EQ(store.getFixType(), fixType);
    EXPECT_EQ(store.getValidity(), validity);
    EXPECT_EQ(store.getStatusFlags(), flags);
}

//
// ✅ Full Struct Set + Helper Get Consistency
//

TEST(GpsStoreTest, FullStructSetReflectsInHelpers)
{
    GpsStore store;

    GpsBasicFixData fix{};
    fix.lon = 111;
    fix.lat = 222;
    fix.hAcc = 333;
    fix.numSV = 4;
    fix.fixType = static_cast<GpsFixType>(5);
    fix.valid = static_cast<GpsValidityFlags>(6);
    fix.flags = static_cast<GpsStatusFlags>(7);

    store.set(fix);

    EXPECT_EQ(store.getLon(), 111);
    EXPECT_EQ(store.getLat(), 222);
    EXPECT_EQ(store.getHAcc(), 333u);
    EXPECT_EQ(store.getNumSV(), 4u);
    EXPECT_EQ(store.getFixType(), fix.fixType);
    EXPECT_EQ(store.getValidity(), fix.valid);
    EXPECT_EQ(store.getStatusFlags(), fix.flags);
}

//
// ✅ GOATStore Tests
//

TEST(GOATStoreTest, CreateResetsStores)
{
    GOATStore goat;
    goat.init();

    // Modify values first
    goat.stateStore.set(State::ABORT_ON_GROUND);

    GpsBasicFixData fix{};
    fix.lat = 111;
    goat.gpsStore.set(fix);

    // Reset
    goat.init();

    EXPECT_EQ(goat.stateStore.get(), State::INIT);
    EXPECT_EQ(goat.gpsStore.get().lat, 0);
}


