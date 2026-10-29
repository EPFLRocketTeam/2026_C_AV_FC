# Reads the Debug configuration of the STM32CubeIDE .cproject at FW_ROOT and sets:
#   FW_SOURCES                         every source file the .cproject compiles
#   FW_{ASM,C,CXX}_INCLUDES            include paths, resolved against FW_ROOT
#   FW_{ASM,C,CXX}_DEFINES             preprocessor defines
#   FW_{ASM,C,CXX}_OPTIONS             optimisation level and compiler other flags
#   FW_{C,CXX}_STANDARD, _EXTENSIONS   language standard, empty when not set
#   FW_LINKER_SCRIPT                   linker script, resolved against FW_ROOT
#   FW_LINK_OPTIONS                    linker other flags
# The caller sets FW_RUNTIME_CPP to the runtime library value its link line
# models ("" for the CubeIDE default); configure fails when .cproject differs.
#
# What is read, inside the first cconfiguration whose id contains ".debug."
# (configure stops on a <fileInfo> or a <folderInfo> with a resourcePath there):
#   every <option id="..." value="..."> of the tool chain and its tools, with
#     <listOptionValue value="..."/> lines up to </option> for list options
#   <sourceEntries>
#     <entry excluding="a|b|c" kind="sourcePath" name="Folder"/>
# An exclusion names a path relative to its folder and everything under it.
#
# Every option must be on the allow list below with a value this build
# models; any other option or value stops configure with its id and value.

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${FW_ROOT}/.cproject")

# One line per list element. &quot; wraps workspace paths; it is the only
# entity in the file and its ';' would split a line, so it goes first.
file(READ "${FW_ROOT}/.cproject" _cp)
string(REPLACE "&quot;" "" _cp "${_cp}")
string(REPLACE ";" " " _cp "${_cp}")
string(REPLACE "\n" ";" _cp "${_cp}")

set(_cp_debug FALSE)
set(_cp_list "")
set(_cp_keys "")
set(_cp_folders "")

foreach(_line IN LISTS _cp)
    # Only the first Debug configuration; a copy made from it also has ".debug.".
    if(_line MATCHES "<cconfiguration id=\"[^\"]*\\.debug\\.")
        set(_cp_debug TRUE)
    elseif(_cp_debug AND _line MATCHES "</cconfiguration>")
        break()
    endif()
    if(NOT _cp_debug)
        continue()
    endif()

    # Per file and per folder settings are not modelled; refuse them.
    if(_line MATCHES "<fileInfo[ >]")
        message(FATAL_ERROR ".cproject Debug holds a per file override, which this build does not model: ${_line}")
    endif()
    if(_line MATCHES "<folderInfo [^>]*resourcePath=\"([^\"]*)\"")
        if(NOT "${CMAKE_MATCH_1}" STREQUAL "")
            message(FATAL_ERROR ".cproject Debug holds a folder override for ${CMAKE_MATCH_1}, which this build does not model")
        endif()
    endif()

    if(_line MATCHES "<option [^>]*id=\"([^\"]*)\"")
        # Key: the id without the ST prefix and the numeric suffix,
        # e.g. tool.cpp.compiler.option.optimization.level.
        set(_id "${CMAKE_MATCH_1}")
        string(REGEX REPLACE "\\.[0-9]+$" "" _key "${_id}")
        string(REGEX REPLACE "^com\\.st\\.stm32cube\\.ide\\.mcu\\.(gnu\\.managedbuild\\.)?" "" _key "${_key}")
        set(_value "")
        if(_line MATCHES " value=\"([^\"]*)\"")
            # Enumerated values carry the option id; keep the part after ".value.".
            string(REGEX REPLACE "^.*\\.value\\." "" _value "${CMAKE_MATCH_1}")
        endif()
        list(APPEND _cp_keys "${_key}")
        set(_cp_id_${_key} "${_id}")
        set(_cp_opt_${_key} "${_value}")
        set(_cp_list_${_key} "")
        if(NOT _line MATCHES "/>")
            set(_cp_list "${_key}")
        endif()
    elseif(_line MATCHES "</option>")
        set(_cp_list "")
    elseif(NOT _cp_list STREQUAL "" AND _line MATCHES "<listOptionValue [^>]*value=\"([^\"]*)\"")
        list(APPEND _cp_list_${_cp_list} "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "<entry [^>]*kind=\"sourcePath\"[^>]*name=\"([^\"]*)\"")
        set(_folder "${CMAKE_MATCH_1}")
        list(APPEND _cp_folders "${_folder}")
        set(_cp_excl_${_folder} "")
        if(_line MATCHES "excluding=\"([^\"]*)\"")
            string(REPLACE "|" ";" _cp_excl_${_folder} "${CMAKE_MATCH_1}")
        endif()
    endif()
endforeach()

if(_cp_folders STREQUAL "")
    message(FATAL_ERROR "No Debug sourceEntries found in ${FW_ROOT}/.cproject")
endif()

# A .cproject path is relative to the Debug build folder (../X) or a workspace
# path of the project (${workspace_loc:/${ProjName}/X}); both name FW_ROOT/X.
function(_cp_resolve out path)
    if(path MATCHES "^\\$\\{workspace_loc:/\\$\\{ProjName\\}(/.*)?\\}$")
        set(path "${FW_ROOT}${CMAKE_MATCH_1}")
    elseif(path MATCHES "^\\.\\.(/.*)?$")
        set(path "${FW_ROOT}${CMAKE_MATCH_1}")
    else()
        message(FATAL_ERROR ".cproject path not understood: ${path}")
    endif()
    string(REGEX REPLACE "/+$" "" path "${path}")
    set(${out} "${path}" PARENT_SCOPE)
endfunction()

set(_cp_lang_assembler ASM)
set(_cp_lang_c.compiler C)
set(_cp_lang_cpp.compiler CXX)
foreach(_lang ASM C CXX)
    set(FW_${_lang}_INCLUDES "")
    set(FW_${_lang}_DEFINES "")
    set(FW_${_lang}_OPTIONS "")
endforeach()
# An absent or empty optimisation option means the compiler default, -O0.
set(FW_C_OPTIONS -O0)
set(FW_CXX_OPTIONS -O0)
set(FW_C_STANDARD "")
set(FW_C_EXTENSIONS "")
set(FW_CXX_STANDARD "")
set(FW_CXX_EXTENSIONS "")
set(FW_LINKER_SCRIPT "")
set(FW_LINK_OPTIONS "")

# Allow list: each Debug option this build models and the values it accepts.
foreach(_key IN LISTS _cp_keys)
    set(_v "${_cp_opt_${_key}}")
    set(_ok FALSE)
    if(_key MATCHES "^(option\\.(target_cpuid|target_coreid|target_board|defaults|convertbinary)|debug\\.option\\.cpuclock)$")
        # IDE metadata: core ids, board, debug clock, wizard defaults, the .bin step (always on here).
        set(_ok TRUE)
    elseif(_key STREQUAL "option.target_mcu")
        # The toolchain file's MCU flags fit every STM32H743.
        if(_v MATCHES "^STM32H743")
            set(_ok TRUE)
        endif()
    elseif(_key STREQUAL "option.fpu")
        if(_v STREQUAL "fpv5-d16")
            set(_ok TRUE)
        endif()
    elseif(_key STREQUAL "option.floatabi")
        if(_v STREQUAL "hard")
            set(_ok TRUE)
        endif()
    elseif(_key STREQUAL "option.runtimelibrary_cpp")
        # Compared with FW_RUNTIME_CPP below.
        set(_ok TRUE)
    elseif(_key MATCHES "^tool\\.(assembler|c\\.compiler|cpp\\.compiler)\\.option\\.debuglevel$")
        # -g3 is written by hand in firmware.cmake.
        if(_v STREQUAL "g3")
            set(_ok TRUE)
        endif()
    elseif(_key MATCHES "^tool\\.(c\\.compiler|cpp\\.compiler)\\.option\\.optimization\\.level$")
        set(_lang "${_cp_lang_${CMAKE_MATCH_1}}")
        if(_v STREQUAL "")
            set(_ok TRUE)
        elseif(_v MATCHES "^o(0|1|2|3|s|g|z|fast)$")
            set(FW_${_lang}_OPTIONS "-O${CMAKE_MATCH_1}")
            set(_ok TRUE)
        endif()
    elseif(_key STREQUAL "tool.cpp.compiler.option.languagestandard")
        if(_v MATCHES "^gnupp([0-9]+)$")
            set(FW_CXX_STANDARD "${CMAKE_MATCH_1}")
            set(FW_CXX_EXTENSIONS ON)
            set(_ok TRUE)
        endif()
    elseif(_key STREQUAL "tool.c.compiler.option.languagestandard")
        if(_v MATCHES "^gnu([0-9]+)$")
            set(FW_C_STANDARD "${CMAKE_MATCH_1}")
            set(FW_C_EXTENSIONS ON)
            set(_ok TRUE)
        endif()
    elseif(_key MATCHES "^tool\\.(assembler|c\\.compiler|cpp\\.compiler)\\.option\\.includepaths$")
        set(_lang "${_cp_lang_${CMAKE_MATCH_1}}")
        foreach(_path IN LISTS _cp_list_${_key})
            _cp_resolve(_path "${_path}")
            list(APPEND FW_${_lang}_INCLUDES "${_path}")
        endforeach()
        set(_ok TRUE)
    elseif(_key MATCHES "^tool\\.(assembler|c\\.compiler|cpp\\.compiler)\\.option\\.definedsymbols$")
        set(_lang "${_cp_lang_${CMAKE_MATCH_1}}")
        list(APPEND FW_${_lang}_DEFINES ${_cp_list_${_key}})
        set(_ok TRUE)
    elseif(_key MATCHES "^tool\\.(assembler|c\\.compiler|cpp\\.compiler)\\.option\\.otherflags$")
        set(_lang "${_cp_lang_${CMAKE_MATCH_1}}")
        foreach(_flag IN LISTS _cp_list_${_key})
            list(APPEND FW_${_lang}_OPTIONS "SHELL:${_flag}")
        endforeach()
        set(_ok TRUE)
    elseif(_key STREQUAL "tool.cpp.linker.option.otherflags")
        # A C++ project links with the G++ linker tool, so only its flags apply.
        foreach(_flag IN LISTS _cp_list_${_key})
            list(APPEND FW_LINK_OPTIONS "SHELL:${_flag}")
        endforeach()
        set(_ok TRUE)
    elseif(_key MATCHES "^tool\\.(c|cpp)\\.linker\\.option\\.script$")
        _cp_resolve(_script "${_v}")
        if(FW_LINKER_SCRIPT STREQUAL "" OR FW_LINKER_SCRIPT STREQUAL _script)
            set(FW_LINKER_SCRIPT "${_script}")
            set(_ok TRUE)
        endif()
    elseif(_key STREQUAL "tool.cpp.linker.option.systemcalls")
        # Minimal system calls, the CubeIDE default, is --specs=nosys.specs in firmware.cmake.
        if(_v STREQUAL "" OR _v STREQUAL "minimalimplementation")
            set(_ok TRUE)
        endif()
    endif()
    if(NOT _ok)
        message(FATAL_ERROR ".cproject Debug option ${_cp_id_${_key}} = \"${_v}\" ${_cp_list_${_key}} is not modelled by cmake/cproject.cmake")
    endif()
endforeach()

if(NOT "${_cp_opt_option.runtimelibrary_cpp}" STREQUAL "${FW_RUNTIME_CPP}")
    message(FATAL_ERROR ".cproject Debug runtime library is \"${_cp_opt_option.runtimelibrary_cpp}\" but firmware.cmake links for \"${FW_RUNTIME_CPP}\"")
endif()
if(FW_LINKER_SCRIPT STREQUAL "")
    message(FATAL_ERROR ".cproject Debug names no linker script")
endif()
foreach(_lang ASM C CXX)
    list(REMOVE_DUPLICATES FW_${_lang}_INCLUDES)
endforeach()

set(FW_SOURCES "")
foreach(_folder IN LISTS _cp_folders)
    file(GLOB_RECURSE _found RELATIVE "${FW_ROOT}" CONFIGURE_DEPENDS
        "${FW_ROOT}/${_folder}/*.c" "${FW_ROOT}/${_folder}/*.cpp"
        "${FW_ROOT}/${_folder}/*.cc" "${FW_ROOT}/${_folder}/*.cxx"
        "${FW_ROOT}/${_folder}/*.s" "${FW_ROOT}/${_folder}/*.S")
    foreach(_pattern IN LISTS _cp_excl_${_folder})
        string(REPLACE "." "\\." _pattern "${_pattern}")
        string(REPLACE "+" "\\+" _pattern "${_pattern}")
        string(REPLACE "*" "[^/]*" _pattern "${_pattern}")
        list(FILTER _found EXCLUDE REGEX "^${_folder}/${_pattern}(/|$)")
    endforeach()
    list(TRANSFORM _found PREPEND "${FW_ROOT}/")
    list(APPEND FW_SOURCES ${_found})
endforeach()

foreach(_folder IN LISTS _cp_folders)
    message(STATUS "cproject folder ${_folder} excluding [${_cp_excl_${_folder}}]")
endforeach()
foreach(_lang ASM C CXX)
    foreach(_kind INCLUDES DEFINES OPTIONS)
        foreach(_v IN LISTS FW_${_lang}_${_kind})
            message(STATUS "cproject ${_lang}_${_kind} ${_v}")
        endforeach()
    endforeach()
endforeach()
message(STATUS "cproject C_STANDARD ${FW_C_STANDARD} extensions ${FW_C_EXTENSIONS}")
message(STATUS "cproject CXX_STANDARD ${FW_CXX_STANDARD} extensions ${FW_CXX_EXTENSIONS}")
message(STATUS "cproject LINKER_SCRIPT ${FW_LINKER_SCRIPT}")
foreach(_v IN LISTS FW_LINK_OPTIONS)
    message(STATUS "cproject LINK_OPTIONS ${_v}")
endforeach()
