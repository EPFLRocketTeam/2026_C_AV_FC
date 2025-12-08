
#include <gtest/gtest.h>

#include "Application/Data/data.hpp"

using namespace flight_computer;

TEST(ApplicationData, TestAvState) {
    EXPECT_EQ( DataManager::read().state, INIT );

    DataManager::setState( ERRORGROUND );
    
    EXPECT_EQ( DataManager::read().state, ERRORGROUND );
}
