cmake_minimum_required(VERSION 3.28.0)
PROJECT(2026_C_AV_FC__SIL)
SET(CMAKE_C_FLAGS "-g -Wall -Werror")
SET(CMAKE_CXX_FLAGS "-g -Wall -Werror -std=c++17")

option(ENABLE_COVERAGE "Enable code coverage instrumentation (requires gcov/lcov)" OFF)
find_program(VALGRIND_EXECUTABLE NAMES valgrind)
add_compile_definitions(UNIT_TEST_ENV)

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
        COMMAND lcov --remove coverage.info '/usr/*' '*test*' --output-file coverage_filtered.info --rc branch_coverage=1

        COMMAND genhtml coverage_filtered.info --output-directory html_report --rc branch_coverage=1
    )
endif()


#############################################################
#################### FREERTOS POSIX PORT ####################
#############################################################

include(FetchContent)

set(FREERTOS_PORT "GCC_POSIX" CACHE STRING "" FORCE)
set(FREERTOS_HEAP "3" CACHE STRING "" FORCE)

# Must be defined before FetchContent_MakeAvailable(freertos_kernel)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32HAL/Simulations/FreeRTOSConfig_host"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

FetchContent_Declare(
    freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.1.0
)
FetchContent_MakeAvailable(freertos_kernel)

#############################################################
###################### FLIGHT COMPUTER ######################
#############################################################

# Add your source files here
# e.g. Drivers/X/Impl/X_mock.cpp
SET(FC_SOURCE_FILES
    Application/Data/data.cpp
    Application/main.cpp
    Application/Data/Stores/NavSensorsStore.cpp
    Application/Data/Stores/SensStatusStore.cpp
    Application/Data/Stores/PropSensorsStore.cpp
    Application/Data/Stores/ValvesStore.cpp
    Application/Data/Stores/NavigationDataStore.cpp
    Application/Data/Stores/EventStore.cpp
    Application/Data/Stores/FlightEventTimersStore.cpp
    Application/Data/Stores/VehiculeOverviewStore.cpp
    Application/FlightControl/av_state.cpp
    Application/Data/Stores/BatteriesStore.cpp
    Application/Data/Stores/CamsRecordingStore.cpp
    Application/Data/Stores/UplinkCmdStore.cpp
    Application/Data/gps_store.cpp
    

    Drivers/UBX_GPS/Impl/ubx_gps_interface.cpp
    # Drivers/STM32HAL/stm32hal.cpp
    Drivers/STM32HAL/Interfaces/mock_uart.cpp
    Drivers/STM32HAL/Simulations/stm32sim_gpio.cpp
    Drivers/STM32HAL/Simulations/stm32sim_spi.cpp

    Drivers/InvIMU/Impl/InvIMU_mock.cpp
    Drivers/STM32HAL/Simulations/stm32sim_ticks.cpp
    Drivers/STM32HAL/Simulations/stm32sim_uart.cpp
)

add_library(flight_computer STATIC ${FC_SOURCE_FILES})
target_link_libraries(flight_computer PRIVATE
    freertos_kernel
    freertos_kernel_port
    pthread
    gcov
)
target_include_directories(flight_computer PUBLIC
    # FreeRTOSConfig_host must come first so its cmsis_os.h shadows the STM32 one
    Drivers/STM32HAL/Simulations/FreeRTOSConfig_host
    .
)

# Add includes here
# For example target_include_directories(flight_computer PUBLIC Drivers/X)


#############################################################
######################## GOOGLE TEST ########################
#############################################################

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

