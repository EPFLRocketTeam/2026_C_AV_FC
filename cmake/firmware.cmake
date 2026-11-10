# FC firmware ELF, mirroring the Debug configuration of .cproject.
# Included from the root CMakeLists.txt only when cross compiling with
# cmake/arm-none-eabi.cmake; the header of that file lists what is read.
#
# Source folders, exclusions, include paths, defines, optimisation, language
# standards, compiler and linker other flags and the linker script are read
# from .cproject at configure time by cmake/cproject.cmake, which also stops
# on any .cproject option it does not model. Written by hand here: the -g3
# debug level (checked against .cproject), the CubeIDE default flags, and the
# runtime and specs choice (FW_RUNTIME_CPP is checked against .cproject).

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(FATAL_ERROR "Only the .cproject Debug configuration is modelled; pass -DCMAKE_BUILD_TYPE=Debug")
endif()

enable_language(ASM)

# The .cproject sets optimisation and debug level itself; drop CMake's defaults.
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_ASM_FLAGS_DEBUG "")

set(FW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(FW_RUNTIME_CPP "standard_c_nano_cpp")
include(${CMAKE_CURRENT_LIST_DIR}/cproject.cmake)

set(FW_NAME 2026_C_AV_FC)
add_executable(${FW_NAME} ${FW_SOURCES})
# Link with the gcc driver so it adds no C++ library of its own; the group
# below names the nano ones (standard_c_nano_cpp) on any GCC version.
set_target_properties(${FW_NAME} PROPERTIES SUFFIX ".elf" LINKER_LANGUAGE C)
foreach(_lang C CXX)
    if(NOT FW_${_lang}_STANDARD STREQUAL "")
        set_target_properties(${FW_NAME} PROPERTIES
            ${_lang}_STANDARD ${FW_${_lang}_STANDARD}
            ${_lang}_STANDARD_REQUIRED ON
            ${_lang}_EXTENSIONS ${FW_${_lang}_EXTENSIONS})
    endif()
endforeach()

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
    "$<$<COMPILE_LANGUAGE:C>:${FW_C_OPTIONS}>"
    "$<$<COMPILE_LANGUAGE:CXX>:${FW_CXX_OPTIONS}>"
    "$<$<COMPILE_LANGUAGE:ASM>:${FW_ASM_OPTIONS}>"
    "$<$<COMPILE_LANGUAGE:C,CXX>:-ffunction-sections;-fdata-sections;-Wall;-fstack-usage>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-use-cxa-atexit>")

# standard_c_nano_cpp: standard newlib C with the nano C++ libraries.
target_link_options(${FW_NAME} PRIVATE
    -T${FW_LINKER_SCRIPT}
    --specs=nosys.specs
    -static
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FW_NAME}.map
    -Wl,--gc-sections
    ${FW_LINK_OPTIONS})
target_link_libraries(${FW_NAME} PRIVATE
    -Wl,--start-group c m stdc++_nano supc++_nano -Wl,--end-group)

add_custom_command(TARGET ${FW_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FW_NAME}> ${FW_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FW_NAME}> ${FW_NAME}.bin
    COMMAND ${ARM_SIZE} $<TARGET_FILE:${FW_NAME}>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    VERBATIM)
