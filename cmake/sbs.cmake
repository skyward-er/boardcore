# Copyright (c) 2021 Skyward Experimental Rocketry
# Author: Damiano Amatruda
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

enable_language(C CXX ASM)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The codebase intentionally uses volatile for ISR/thread-shared variables;
# suppress the C++20/23 deprecation diagnostics about it (-Wvolatile, enabled
# by -Wall). The generator expression keeps the flag C++-only.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>)
endif()

# Load in SBS_BASE the project path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH SBS_BASE)
# Load in BOARDCORE_PATH the boardcore path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH BOARDCORE_PATH)

# Add the version information header to the global include path, so that all 
# targets defined after this point will have access to it (Boardcore, Kernel)
include_directories(${BOARDCORE_PATH}/version)

# Include the Boardcore libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
include(boardcore)

# Declarative registry of executable targets.
#
#   sbs_declare_target(<name> <board> <sources...> [DEFS <defs...>] [HOST])
#
# In a cross (Miosix) configure the target is instantiated only when <board>
# matches the board selected with MIOSIX_BOARD (Miosix 3 builds exactly one
# board per CMake configure). In a host configure only targets marked HOST
# are instantiated, so the host build tree stays free of kernel code.
function(sbs_declare_target TARGET OPT_BOARD)
    cmake_parse_arguments(DECL "HOST" "" "DEFS" ${ARGN})

    set(_instantiate FALSE)
    if(CMAKE_CROSSCOMPILING)
        if(OPT_BOARD STREQUAL MIOSIX_BOARD)
            set(_instantiate TRUE)
        endif()
    else()
        if(DECL_HOST)
            set(_instantiate TRUE)
        endif()
    endif()

    if(NOT _instantiate)
        return()
    endif()

    add_executable(${TARGET} ${DECL_UNPARSED_ARGUMENTS})

    if(DECL_DEFS)
        target_compile_definitions(${TARGET} PRIVATE ${DECL_DEFS})
    endif()

    sbs_target(${TARGET} ${OPT_BOARD})
endfunction()

# Function to link the Boardcore library to the target
function(sbs_target TARGET OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()

    target_include_directories(${TARGET} PRIVATE src/shared)

    # Define the version information generation command
    add_custom_target(${TARGET}-version-info
        BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET}/version.cpp
        COMMAND "${CMAKE_COMMAND}"
        "-D" "TARGET_NAME=${TARGET}"
        "-D" "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-D" "BOARDCORE_PATH=${BOARDCORE_PATH}"
        "-D" "OUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET}"
        "-P" "${BOARDCORE_PATH}/cmake/version.cmake"
        COMMENT "Generating version information file for ${TARGET}"
        VERBATIM
    )
    # Build the generated version information file as part of the target
    target_sources(${TARGET} PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET}/version.cpp
    )

    if(CMAKE_CROSSCOMPILING)
        if(NOT OPT_BOARD STREQUAL MIOSIX_BOARD)
            message(FATAL_ERROR
                "Target ${TARGET} is defined for board ${OPT_BOARD} but the "
                "current configure is for board ${MIOSIX_BOARD}. "
                "Miosix 3 builds one board per CMake configure."
            )
        endif()

        # Link the embedded Boardcore library
        target_link_libraries(${TARGET} PRIVATE Skyward::Boardcore::${OPT_BOARD})

        # Linker script, link options, map file and the .bin/.hex artifacts
        # come from Miosix's own miosix_link_target()
        miosix_link_target(${TARGET})
    else()
        target_link_libraries(${TARGET} PRIVATE Skyward::Boardcore::host)
    endif()
endfunction()

function(sbs_catch_test TARGET)
    if(NOT CMAKE_CROSSCOMPILING)
        catch_discover_tests(${TARGET})
    endif()
endfunction()
