# arm-none-eabi toolchain for the FC firmware (STM32H743ZITx, Cortex-M7).
#
# Read from .cproject at configure time by cmake/cproject.cmake (the values below are
# what it reads on main): source folders and exclusions, include paths, defines,
# optimisation level, C and C++ language standards, compiler and linker other flags,
# linker script. Any other Debug option, or a value outside its allow list, stops
# configure with the option id and value.
#
# Written by hand, because .cproject does not store them or the upstream toolchain
# lacks ST's specs files:
#   MCU flags in this file (cproject.cmake checks fpu fpv5-d16, float abi hard and an
#     STM32H743 part)
#   -g3 in firmware.cmake (cproject.cmake checks the debug level is g3)
#   the CubeIDE default flags listed at the end of this block
#   the runtime and specs choice in firmware.cmake (FW_RUNTIME_CPP must equal the
#     .cproject runtimelibrary_cpp value; system calls must be minimal, i.e. nosys.specs)
#
# What was read from .cproject, Debug configuration (line numbers on main a671cca):
#   MCU            STM32H743ZITx, fpu fpv5-d16, float abi hard            (:20,23,24)
#   Source folders ThirdParty, Drivers, Application, Core, Middlewares,
#                  USB_DEVICE                                             (:144-149)
#   Exclusions     Drivers: CMSIS/Device, ERT_RF_Protocol_Interface/tests,
#                  Capsule/src/basicExample.cpp, STM32HAL/Interfaces,
#                  Plume/2026_C_AV_PLUME/src/plume/cli,
#                  Plume/2026_C_AV_PLUME/src/plume/drivers/unix.c,
#                  Plume/2026_C_AV_PLUME/tests, STM32HAL/Simulations,
#                  STM32HAL/Tests, UBX_GPS/Tests/test_gps.cpp,
#                  BMP390/Tests/Mock, InvIMU/Impl/InvIMU_mock.cpp,
#                  InvIMU/Tests/Mock                                       (:145)
#                  Application: Tests                                      (:146)
#   C defines      DEBUG KALMAN_DEBUG_FORCE_FLIGHT=0 KALMAN_DEBUG_PRINT=0
#                  COMPILE_OPT_LEVEL=2 USE_PWR_LDO_SUPPLY USE_HAL_DRIVER
#                  STM32H743xx                                             (:47-53)
#   C++ defines    same seven                                             (:83-89)
#   ASM defines    DEBUG; ASM includes: project root                     (:34-40)
#   C includes     Core/Inc, Application/Config/2026_C_AV_FLIGHT_PARAMS(+/2026_C_AV_CONFMAN),
#                  Drivers/SX127X, Drivers/Plume/2026_C_AV_PLUME/include,
#                  Drivers/STM32HAL, Drivers/Plume, root, HAL Inc(+Legacy),
#                  CMSIS Device Include, CMSIS Include, USB_DEVICE/App,
#                  USB_DEVICE/Target, USB Core/CDC/DFU Inc, Drivers/InvIMU,
#                  Application, ThirdParty/ArduinoEigen-0.3.2              (:55-76)
#   C++ includes   every C path, plus Drivers/Capsule/src,
#                  Drivers/PRC_CAN/2026_C_AV_FC_PRC_INTRANET/include,
#                  Drivers/LOG_AGGREGATOR/2026_C_AV_LOG_AGGREGATOR/include,
#                  Drivers/CLI/2026_C_AV_CLI, Drivers/BMP390               (:91-116)
#   Optimisation   -O2 -g3 for C and C++, -g3 for ASM                      (:33,44-45,80-81)
#   C standard     not set (compiler default)
#   C++ standard   gnu++20                                                 (:118)
#   Runtime        standard_c_nano_cpp: standard newlib C, nano libstdc++  (:29)
#   System calls   not set: CubeIDE default, minimal (--specs=nosys.specs)
#   Startup        Core/Startup/startup_stm32h743zitx.s (inside Core)
#   Linker script  STM32H743ZITX_FLASH.ld                                  (:123)
#   Link flags     -u _printf_float -u _printf_i64                         (:125-126)
#   Convert binary on                                                      (:28)
#
# CubeIDE adds these at build time without recording them in .cproject; they are
# its defaults and are applied in cmake/firmware.cmake:
#   -ffunction-sections -fdata-sections -Wall -fstack-usage
#   C++: -fno-exceptions -fno-rtti -fno-use-cxa-atexit
#   ASM: -x assembler-with-cpp
#   link: -static -Wl,--gc-sections -Wl,-Map=<elf>.map, libc libm libstdc++ libsupc++ in a group

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(ARM_SIZE arm-none-eabi-size)

# A bare metal compiler cannot link a test program without a linker script.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_MCU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_C_FLAGS_INIT "${ARM_MCU_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${ARM_MCU_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${ARM_MCU_FLAGS} -x assembler-with-cpp")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
