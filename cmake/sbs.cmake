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

# Load in SBS_BASE the project path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH SBS_BASE)

# Include the Boardcore libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
include(boardcore)

# Command to print all the available boards used by the sbs script
string(REPLACE ";" "\\n" BOARDS_STR "${MIOSIX_BOARDS};${BOARDCORE_BOARDS}")
add_custom_target(
    help-boards
    COMMAND printf ${BOARDS_STR}
    COMMENT "All boards available:"
    VERBATIM
)

# Function to link the Boardcore library to the target
function(sbs_target TARGET OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()

    # The only include directory of Boardcore is shared!
    target_include_directories(${TARGET} PRIVATE src/shared)

    if(CMAKE_CROSSCOMPILING)
        # Link the embedded Boardcore library
        target_link_libraries(${TARGET} PRIVATE Skyward::Boardcore::${OPT_BOARD})

        # Linker script and linking options are eredited from the kernel library

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
