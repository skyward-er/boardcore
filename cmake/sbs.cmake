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

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load in SBS_BASE the project path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH SBS_BASE)

# Include the Boardcore libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
include(boardcore)

# Function to link the Boardcore library to the target
function(sbs_target TARGET OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()

    target_include_directories(${TARGET} PRIVATE src/shared)

    add_custom_target(${TARGET}-version-info
        COMMAND "${CMAKE_COMMAND}"
        "-D" "TARGET_NAME=${TARGET}"
        "-D" "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-D" "BOARDCORE_PATH=${BOARDCORE_PATH}"
        "-D" "BIN_DIR=${CMAKE_CURRENT_BINARY_DIR}/include/${TARGET}"
        "-P" "${BOARDCORE_PATH}/cmake/version.cmake"
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/include/${TARGET}/version.h"
        COMMENT "Generating version information file for ${TARGET}"
        VERBATIM
    )
    add_dependencies(${TARGET} ${TARGET}-version-info)

    if(CMAKE_CROSSCOMPILING)
        # Link the embedded Boardcore library
        target_link_libraries(${TARGET} PRIVATE Skyward::Boardcore::${OPT_BOARD})
        
        # Add the generated include directory to the target
        # This also works for injecting include directories into consumed libs
        # E.g. adding version information to the kernel library
        target_include_directories(${TARGET} PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/${TARGET}>
        )
        
        # Linker script and linking options are inherited from the kernel library

        # Add a post build command to create the hex file to flash on the board
        add_custom_command(
            TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${TARGET}.hex
            COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${TARGET}.bin
            BYPRODUCTS ${TARGET}.hex ${TARGET}.bin
            VERBATIM
        )
    else()
        target_link_libraries(${TARGET} PRIVATE Skyward::Boardcore::host)
    endif()
endfunction()

function(sbs_catch_test TARGET)
    if(NOT CMAKE_CROSSCOMPILING)
        catch_discover_tests(${TARGET})
    endif()
endfunction()
