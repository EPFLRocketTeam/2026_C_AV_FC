cmake_minimum_required(VERSION 3.28.0)
PROJECT(2026_C_AV_FC)

# Firmware ELF, only with -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake.
if(CMAKE_CROSSCOMPILING)
    include(cmake/firmware.cmake)
    return()
endif()

SET(CMAKE_C_FLAGS "-g -Wall")
SET(CMAKE_CXX_FLAGS "-g -Wall -std=c++20")

option(ENABLE_COVERAGE "Enable code coverage instrumentation (requires gcov/lcov)" OFF)
find_program(VALGRIND_EXECUTABLE NAMES valgrind)
add_compile_definitions(UNIT_TEST_ENV APP_TARGET_NATIVE=1)

if (ENABLE_COVERAGE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -O0")

    set(MEMORYCHECK_COMMAND "${VALGRIND_EXECUTABLE}")
    set(MEMORYCHECK_COMMAND_OPTIONS "--tool=memcheck --leak-check=full --track-fds=yes --error-exitcode=1")

    add_custom_target(
        memcheck

        COMMAND ctest -T memcheck
    )
    add_custom_target(
        coverage

        COMMAND lcov --directory . --zerocounters --rc branch_coverage=1
        COMMAND ctest

        COMMAND lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch --rc branch_coverage=1
        COMMAND lcov --remove coverage.info '/usr/*' '*test*' '*_deps*' --output-file coverage_filtered.info --ignore-errors unused --rc branch_coverage=1

        COMMAND genhtml coverage_filtered.info --output-directory html_report --rc branch_coverage=1
    )
endif()

#############################################################
###################### FLIGHT COMPUTER ######################
#############################################################

# Host-buildable sources only: no main.cpp, no kalman_process.cpp, no STM32 drivers.
file(GLOB FC_STORE_SOURCES Application/Data/Stores/*.cpp)
file(GLOB_RECURSE FC_KALMAN_SOURCES Application/Kalman/AppLayer/*.cpp Application/Kalman/kalman/*.cpp)
SET(FC_SOURCE_FILES
    Application/Data/data.cpp
    Application/Data/gps_store.cpp
    ${FC_STORE_SOURCES}
    Application/Config/config.cpp
    Application/app_printf.cpp
    Application/FlightControl/av_state.cpp
    Application/Tests/stubs/prc_can_stub.cpp
    Application/Kalman/kalman_health.cpp
    Application/Kalman/kalman_lifecycle.cpp
    ${FC_KALMAN_SOURCES}

    Drivers/UBX_GPS/Impl/ubx_gps_interface.cpp
    Drivers/STM32HAL/Interfaces/mock_uart.cpp
    Drivers/STM32HAL/Simulations/stm32sim_gpio.cpp
    Drivers/STM32HAL/Simulations/stm32sim_spi.cpp
    Drivers/STM32HAL/Simulations/stm32sim_ticks.cpp
    Drivers/STM32HAL/Simulations/stm32sim_uart.cpp
    Drivers/InvIMU/Impl/InvIMU_mock.cpp
    Drivers/BMP390/Impl/BMP390_mock.cpp
)

add_library(flight_computer STATIC ${FC_SOURCE_FILES})
target_include_directories(flight_computer PUBLIC
    .
    Application/Kalman
    Application/Tests/host_replay/inc
    Application/Config/2026_C_AV_FLIGHT_PARAMS
    Application/Config/2026_C_AV_FLIGHT_PARAMS/2026_C_AV_CONFMAN
    Drivers/PRC_CAN/2026_C_AV_FC_PRC_INTRANET/include
)
if (ENABLE_COVERAGE)
    target_link_libraries(flight_computer PRIVATE gcov)
endif()

#############################################################
######################## GOOGLE TEST ########################
#############################################################

include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(CTest)
set(CTEST_TIMEOUT 30)

include(GoogleTest)

function(create_test test_name test_file)
    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name} PRIVATE flight_computer GTest::gtest_main GTest::gmock)
    gtest_discover_tests(${test_name})
endfunction()

add_subdirectory(Application/Tests)
add_subdirectory(Drivers/UBX_GPS/Tests)
add_subdirectory(Drivers/STM32HAL/Tests)
add_subdirectory(Drivers/InvIMU/Tests)
add_subdirectory(Drivers/BMP390/Tests)
