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

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(SBS_BASE ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)

if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL SBS_BASE)
    add_subdirectory(${SBS_BASE} EXCLUDE_FROM_ALL)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
    include(${CMAKE_SOURCE_DIR}/cmake/dependencies.cmake OPTIONAL)
    return()
endif()

include(boardcore)

string(REPLACE ";" "\\n" BOARDS_STR "${BOARDS}")
add_custom_target(
    help-boards
    COMMAND printf ${BOARDS_STR}
    COMMENT "All boards available:"
    VERBATIM
)

function(sbs_target TARGET OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()
    target_include_directories(${TARGET} PRIVATE src/shared)
    target_link_libraries(${TARGET} PRIVATE SkywardBoardcore::Boardcore-${OPT_BOARD})
    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${TARGET}.bin
        BYPRODUCTS ${TARGET}.hex ${TARGET}.bin
        VERBATIM
    )
endfunction()
