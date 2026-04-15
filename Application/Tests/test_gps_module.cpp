/*
 * test_gps_module.cpp
 *
 * Unit tests for GpsModule (Application/Modules/gps_module.hpp).
 *
 * The module wraps a UbxGpsInterface driver and a RingBuffer, then exposes
 * init() / update() / getBuffer() as the Module interface.
 *
 * Strategy
 * --------
 * - A MockUARTDevice backs the UbxGpsInterface so no real hardware is needed.
 * - stm32sim_ticks provides HAL_GetTick() so the driver's internal timeout
 *   logic works correctly.
 * - gpsDataMutexHandle is set to nullptr; the cmsis_os host shim treats a
 *   null handle as a no-op (returns non-zero but the module does not check
 *   the return value), so mutex semantics are skipped in these unit tests.
 * - gpsData is already defined in Application/main.cpp (part of the
 *   flight_computer library) and satisfies the extern declaration in
 *   gps_module.hpp.
 */

#include <gtest/gtest.h>

#include "Application/Data/ring_buffer.hpp"
#include "Application/Data/data.hpp"
#include "Application/Modules/gps_module.hpp"
#include "Drivers/STM32HAL/Interfaces/mock_uart.hpp"
#include "Drivers/UBX_GPS/Tests/ubx_test_helpers.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_ticks.hpp"

// ---------------------------------------------------------------------------
// Globals required by gps_module.hpp
// ---------------------------------------------------------------------------

// gpsData is defined in Application/main.cpp (flight_computer library).
// gpsDataMutexHandle is not generated on host builds; define it here as null
// so that osMutexAcquire / osMutexRelease become safe no-ops.
osMutexId_t gpsDataMutexHandle = 0;  // null handle → mutex ops are no-ops in tests

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

static UART_HandleTypeDef huart_gps_mod = {20, "GPS_MOD_UART"};

class GpsModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        stm32sim_ticks_init();
        mockGps_ = new SIMULATOR_NAMESPACE::interfaces::MockUARTDevice(&huart_gps_mod);
        gps_     = new UbxGpsInterface(mockGps_->getHandle(), 1000);

        UbxGpsInterface*               driversArr[] = {gps_};
        RingBuffer<GpsBasicFixData, 100>* bufsArr[]  = {&buf_};

        module_ = new GpsModule(driversArr, bufsArr);
    }

    void TearDown() override {
        delete module_;
        delete gps_;
        delete mockGps_;
        stm32sim_ticks_deinit();
    }

    // Feed a pre-built packet so the next update() call succeeds instantly.
    void feedValidPacket() {
        mockGps_->feedRx(UBXTestHelpers::createDefaultPvtPacket());
    }

    void feedCustomPacket(const UBXTestHelpers::PvtPacketParams& p) {
        mockGps_->feedRx(UBXTestHelpers::createCustomPvtPacket(p));
    }

    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice* mockGps_ = nullptr;
    UbxGpsInterface*                                 gps_     = nullptr;
    RingBuffer<GpsBasicFixData, 100>                 buf_;
    GpsModule*                                       module_  = nullptr;
};

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

TEST_F(GpsModuleTest, InitSuccess) {
    // With a mock UART the driver sends config commands successfully.
    // After the bug-fix (== → !=) init() must return true.
    EXPECT_TRUE(module_->init());
}

// ---------------------------------------------------------------------------
// update() – buffer interaction
// ---------------------------------------------------------------------------

TEST_F(GpsModuleTest, UpdateAppendsToBuffer) {
    ASSERT_TRUE(module_->init());
    ASSERT_TRUE(buf_.empty());

    feedValidPacket();
    module_->update(0);

    EXPECT_EQ(buf_.size(), 1u);
}

TEST_F(GpsModuleTest, UpdateOnTimeoutStillAppendsZeroedFix) {
    // When no data is available getPvt() times out, but update() still
    // appends the zero-initialised GpsBasicFixData to keep the buffer
    // advancing in lock-step with the scheduler tick.
    ASSERT_TRUE(module_->init());

    // No feedRx → getPvt will time out after 150 ms
    module_->update(0);

    EXPECT_EQ(buf_.size(), 1u);
    const GpsBasicFixData* entry = buf_.get(0);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->lat, 0);
    EXPECT_EQ(entry->lon, 0);
    EXPECT_EQ(entry->fixType, GpsFixType::NO_FIX);
}

TEST_F(GpsModuleTest, MultipleUpdatesGrowBuffer) {
    ASSERT_TRUE(module_->init());

    for (int i = 0; i < 3; ++i) {
        feedValidPacket();
        module_->update(static_cast<uint32_t>(i * 1000));
    }

    EXPECT_EQ(buf_.size(), 3u);
}

// ---------------------------------------------------------------------------
// update() – GOATStore interaction
// ---------------------------------------------------------------------------

TEST_F(GpsModuleTest, UpdateSetsGOATStore) {
    ASSERT_TRUE(module_->init());

    UBXTestHelpers::PvtPacketParams p;
    p.lat   = 473768880;   // 47.376888°
    p.lon   = 85052780;    // 8.505278°
    p.hAcc  = 2000;
    p.numSV = 12;

    feedCustomPacket(p);
    module_->update(0);

    const GpsBasicFixData& stored =
        flight_computer::GOATStore::get_instance().gpsStore.get();

    EXPECT_EQ(stored.lat,   473768880);
    EXPECT_EQ(stored.lon,   85052780);
    EXPECT_EQ(stored.hAcc,  2000u);
    EXPECT_EQ(stored.numSV, 12);
}

TEST_F(GpsModuleTest, UpdateSetsGOATStore_FixTypeAndValidity) {
    ASSERT_TRUE(module_->init());

    UBXTestHelpers::PvtPacketParams p;
    p.fixType     = 3;     // FIX_3D
    p.validFlags  = 0x0F;  // all flags set
    p.statusFlags = 0x01;  // gnssFixOK

    feedCustomPacket(p);
    module_->update(0);

    const GpsBasicFixData& stored =
        flight_computer::GOATStore::get_instance().gpsStore.get();

    EXPECT_EQ(stored.fixType,            GpsFixType::FIX_3D);
    EXPECT_TRUE(stored.valid.validDate);
    EXPECT_TRUE(stored.valid.validTime);
    EXPECT_TRUE(stored.valid.fullyResolved);
    EXPECT_TRUE(stored.flags.gnssFixOK);
}

// ---------------------------------------------------------------------------
// getBuffer()
// ---------------------------------------------------------------------------

TEST_F(GpsModuleTest, GetBufferReturnsTheSameBuffer) {
    ASSERT_TRUE(module_->init());

    feedValidPacket();
    module_->update(0);

    // getBuffer() must expose the same data that was written by update()
    const RingBuffer<GpsBasicFixData, 100>& exposed = module_->getBuffer(0);
    EXPECT_EQ(exposed.size(), buf_.size());

    const GpsBasicFixData* via_buf    = buf_.get(0);
    const GpsBasicFixData* via_getter = exposed.get(0);

    ASSERT_NE(via_buf,    nullptr);
    ASSERT_NE(via_getter, nullptr);
    EXPECT_EQ(via_buf->lat, via_getter->lat);
    EXPECT_EQ(via_buf->lon, via_getter->lon);
}
