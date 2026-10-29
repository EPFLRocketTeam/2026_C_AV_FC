# Reads the Debug configuration of the STM32CubeIDE .cproject at FW_ROOT and sets:
#   FW_SOURCES                         every source file the .cproject compiles
#   FW_{ASM,C,CXX}_INCLUDES            include paths, resolved against FW_ROOT
#   FW_{ASM,C,CXX}_DEFINES             preprocessor defines
#
# What is read, inside the first cconfiguration whose id contains ".debug."
# (configure stops on a <fileInfo> or a <folderInfo> with a resourcePath there):
#   <tool id="...tool.assembler|c.compiler|cpp.compiler...">
#     <option id="...option.includepaths..."> and <option id="...option.definedsymbols...">
#       <listOptionValue value="..."/> one per line, up to </option>
#   <sourceEntries>
#     <entry excluding="a|b|c" kind="sourcePath" name="Folder"/>
# An exclusion names a path relative to its folder and everything under it.

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${FW_ROOT}/.cproject")

# One line per list element. &quot; wraps workspace paths; it is the only
# entity in the file and its ';' would split a line, so it goes first.
file(READ "${FW_ROOT}/.cproject" _cp)
string(REPLACE "&quot;" "" _cp "${_cp}")
string(REPLACE ";" " " _cp "${_cp}")
string(REPLACE "\n" ";" _cp "${_cp}")

set(_cp_debug FALSE)
set(_cp_lang "")
set(_cp_kind "")
set(_cp_folders "")
foreach(_lang ASM C CXX)
    set(_cp_${_lang}_INCLUDES "")
    set(_cp_${_lang}_DEFINES "")
endforeach()

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

    if(_line MATCHES "<tool id=\"[^\"]*\\.tool\\.(assembler|c\\.compiler|cpp\\.compiler)\\.")
        set(_cp_lang_assembler ASM)
        set(_cp_lang_c.compiler C)
        set(_cp_lang_cpp.compiler CXX)
        set(_cp_lang "${_cp_lang_${CMAKE_MATCH_1}}")
    elseif(_line MATCHES "<tool ")
        set(_cp_lang "")
    elseif(NOT _cp_lang STREQUAL "" AND _line MATCHES "<option [^>]*\\.option\\.(includepaths|definedsymbols)\\.[^>]*[^/]>")
        if(CMAKE_MATCH_1 STREQUAL "includepaths")
            set(_cp_kind INCLUDES)
        else()
            set(_cp_kind DEFINES)
        endif()
    elseif(_line MATCHES "</option>")
        set(_cp_kind "")
    elseif(NOT _cp_kind STREQUAL "" AND _line MATCHES "<listOptionValue [^>]*value=\"([^\"]*)\"")
        list(APPEND _cp_${_cp_lang}_${_cp_kind} "${CMAKE_MATCH_1}")
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

# Include paths are relative to the Debug build folder (../X) or workspace
# paths of the project (${workspace_loc:/${ProjName}/X}); both name FW_ROOT/X.
foreach(_lang ASM C CXX)
    set(FW_${_lang}_INCLUDES "")
    foreach(_path IN LISTS _cp_${_lang}_INCLUDES)
        if(_path MATCHES "^\\$\\{workspace_loc:/\\$\\{ProjName\\}(/.*)?\\}$")
            set(_path "${FW_ROOT}${CMAKE_MATCH_1}")
        elseif(_path MATCHES "^\\.\\.(/.*)?$")
            set(_path "${FW_ROOT}${CMAKE_MATCH_1}")
        else()
            message(FATAL_ERROR ".cproject include path not understood: ${_path}")
        endif()
        string(REGEX REPLACE "/+$" "" _path "${_path}")
        list(APPEND FW_${_lang}_INCLUDES "${_path}")
    endforeach()
    list(REMOVE_DUPLICATES FW_${_lang}_INCLUDES)
    set(FW_${_lang}_DEFINES ${_cp_${_lang}_DEFINES})
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
    foreach(_v IN LISTS FW_${_lang}_INCLUDES)
        message(STATUS "cproject ${_lang}_INCLUDES ${_v}")
    endforeach()
    foreach(_v IN LISTS FW_${_lang}_DEFINES)
        message(STATUS "cproject ${_lang}_DEFINES ${_v}")
    endforeach()
endforeach()
