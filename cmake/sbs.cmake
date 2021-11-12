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

# Fix VSCode warning "cannot find entry symbol arch_paths_first; defaulting to 0000000000000000"
# Alternative: set(CMAKE_<LANG>_LINK_EXECUTABLE "<CMAKE_<LANG>_COMPILER> <FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
set(CMAKE_C_LINK_FLAGS "")
set(CMAKE_CXX_LINK_FLAGS "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH SBS_BASE)

file(GLOB KPATH ${SBS_BASE}/libs/miosix-kernel/miosix)
if(NOT KPATH)
    message(FATAL_ERROR "Kernel directory not found")
endif()

include(dependencies)

function(sbs_mxgui TARGET)
    get_target_property(OPT_BOARD ${TARGET} OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()

    target_link_libraries(${TARGET} mxgui::mxgui-${OPT_BOARD})
endfunction()

function(sbs_target TARGET)
    get_target_property(OPT_BOARD ${TARGET} OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()
    set(OPT_BOARD ${OPT_BOARD} PARENT_SCOPE)
endfunction()

function(sbs_link_mxgui TARGET)
    sbs_get_board(${TARGET})
    target_link_libraries(${TARGET} PRIVATE mxgui::mxgui-${OPT_BOARD})
endfunction()

function(sbs_target TARGET)
    sbs_get_board(${TARGET})
    target_include_directories(${TARGET} PRIVATE
        ${SBS_BASE}/libs
        ${SBS_BASE}/src/shared
        src/shared
    )
    target_link_libraries(${TARGET} PRIVATE miosix::miosix-${OPT_BOARD})
    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${TARGET}.bin
        BYPRODUCTS ${TARGET}.hex ${TARGET}.bin
        VERBATIM
    )
endfunction()
