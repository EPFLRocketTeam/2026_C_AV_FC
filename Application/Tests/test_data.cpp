
#include <gtest/gtest.h>

#include "Application/Data/data.hpp"

using namespace flight_computer;

TEST(ApplicationData, TestAvState) {
    EXPECT_EQ( DataManager::read().state, INIT );

    DataManager::setState( ERRORGROUND );
    
    EXPECT_EQ( DataManager::read().state, ERRORGROUND );
}

TEST(StateStoreTest, DefaultStateIsInit)
{
    StateStore store;
    EXPECT_EQ(store.get(), State::INIT);
}

TEST(StateStoreTest, SetAndGetState)
{
    StateStore store;

    store.set(State::ERRORGROUND);
    EXPECT_EQ(store.get(), State::ERRORGROUND);

    store.set(State::READY);
    EXPECT_EQ(store.get(), State::READY);
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

//
// ✅ GOATStore Tests
//

TEST(GOATStoreTest, CreateResetsStores)
{
    GOATStore goat;
    goat.init();

    // Modify values first
    goat.stateStore.set(State::ERRORGROUND);

    GpsBasicFixData fix{};
    fix.lat = 111;
    goat.gpsStore.set(fix);

    // Reset
    goat.init();

    EXPECT_EQ(goat.stateStore.get(), State::INIT);
    EXPECT_EQ(goat.gpsStore.get().lat, 0);
}


