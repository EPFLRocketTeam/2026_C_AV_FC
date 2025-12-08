cmake_minimum_required(VERSION 3.28.0)
PROJECT(2026_C_AV_FC__SIL)
SET(CMAKE_C_FLAGS "-g -Wall -Werror")
SET(CMAKE_CXX_FLAGS "-g -Wall -Werror -std=c++17")

option(ENABLE_COVERAGE "Enable code coverage instrumentation (requires gcov/lcov)" OFF)
find_program(VALGRIND_EXECUTABLE NAMES valgrind)

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
###################### FLIGHT COMPUTER ######################
#############################################################

SET(FC_SOURCE_FILES
    Application/Data/data.cpp
    Application/main.cpp)
add_library(flight_computer STATIC ${FC_SOURCE_FILES})
target_link_libraries(flight_computer PRIVATE gcov)
target_include_directories(flight_computer PUBLIC .)

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
