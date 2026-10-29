# FC firmware ELF, mirroring the Debug configuration of .cproject.
# Included from the root CMakeLists.txt only when cross compiling with
# cmake/arm-none-eabi.cmake; the header of that file lists what is read.
#
# Source folders, their exclusions, include paths and defines are read from
# .cproject at configure time, so a .cproject edit reaches this build too.
# Only what .cproject does not store is written here: MCU flags (toolchain
# file), CubeIDE default flags, optimisation, runtime and the linker script.

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(FATAL_ERROR "Only the .cproject Debug configuration is modelled; pass -DCMAKE_BUILD_TYPE=Debug")
endif()

enable_language(ASM)

# The .cproject sets optimisation and debug level itself; drop CMake's defaults.
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_ASM_FLAGS_DEBUG "")

set(FW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
include(${CMAKE_CURRENT_LIST_DIR}/cproject.cmake)

set(FW_NAME 2026_C_AV_FC)
add_executable(${FW_NAME} ${FW_SOURCES})
set_target_properties(${FW_NAME} PROPERTIES
    SUFFIX ".elf"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON)

target_include_directories(${FW_NAME} PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:${FW_C_INCLUDES}>"
    "$<$<COMPILE_LANGUAGE:CXX>:${FW_CXX_INCLUDES}>"
    "$<$<COMPILE_LANGUAGE:ASM>:${FW_ASM_INCLUDES}>")

target_compile_definitions(${FW_NAME} PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:${FW_C_DEFINES}>"
    "$<$<COMPILE_LANGUAGE:CXX>:${FW_CXX_DEFINES}>"
    "$<$<COMPILE_LANGUAGE:ASM>:${FW_ASM_DEFINES}>")

target_compile_options(${FW_NAME} PRIVATE
    -g3
    "$<$<COMPILE_LANGUAGE:C,CXX>:-O2;-ffunction-sections;-fdata-sections;-Wall;-fstack-usage>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-use-cxa-atexit>")

# standard_c_nano_cpp: standard newlib C with the nano C++ libraries.
target_link_options(${FW_NAME} PRIVATE
    -T${FW_ROOT}/STM32H743ZITX_FLASH.ld
    --specs=nosys.specs
    -static
    -nostdlib++
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FW_NAME}.map
    -Wl,--gc-sections
    "SHELL:-u _printf_float"
    "SHELL:-u _printf_i64")
target_link_libraries(${FW_NAME} PRIVATE
    -Wl,--start-group c m stdc++_nano supc++_nano -Wl,--end-group)

add_custom_command(TARGET ${FW_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FW_NAME}> ${FW_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FW_NAME}> ${FW_NAME}.bin
    COMMAND ${ARM_SIZE} $<TARGET_FILE:${FW_NAME}>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    VERBATIM)
