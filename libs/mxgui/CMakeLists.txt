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

cmake_minimum_required(VERSION 3.16)
project(Mxgui C CXX ASM)

set(KPATH "" CACHE PATH "Path to kernel directory")
file(GLOB KPATH ${KPATH})
if(NOT KPATH)
    message(FATAL_ERROR "Kernel directory not found")
endif()

include(cmake/boards.cmake)

foreach(OPT_BOARD ${MXGUI_BOARDS})
    set(DFLAGS -MMD -MP)
    include(${KPATH}/config/options.cmake)

    set(MXGUI_LIBRARY mxgui-${OPT_BOARD})
    add_library(${MXGUI_LIBRARY} STATIC
        mxgui/display.cpp
        mxgui/font.cpp
        mxgui/misc_inst.cpp
        mxgui/tga_image.cpp
        mxgui/resourcefs.cpp
        mxgui/resource_image.cpp
        mxgui/level2/input.cpp
        mxgui/level2/application.cpp
        mxgui/level2/drawing_context_proxy.cpp
        mxgui/level2/label.cpp
        mxgui/level2/simple_plot.cpp
        mxgui/drivers/display_stm3210e-eval.cpp
        mxgui/drivers/event_stm3210e-eval.cpp
        mxgui/drivers/display_mp3v2.cpp
        mxgui/drivers/event_mp3v2.cpp
        mxgui/drivers/resfs_mp3v2.cpp
        mxgui/drivers/display_strive.cpp
        mxgui/drivers/display_st7735.cpp
        mxgui/drivers/display_oledboard2.cpp
        mxgui/drivers/event_strive.cpp
        mxgui/drivers/display_redbull_v2.cpp
        mxgui/drivers/event_redbull_v2.cpp
        mxgui/drivers/display_bitsboard.cpp
        mxgui/drivers/display_sony-newman.cpp
        mxgui/drivers/event_sony-newman.cpp
        mxgui/drivers/display_stm32f4discovery.cpp
        mxgui/drivers/event_stm32f4discovery.cpp
        mxgui/drivers/display_generic_1bpp.cpp
        mxgui/drivers/display_generic_4bpp.cpp
        mxgui/drivers/display_st7735.cpp
    )
    add_library(Mxgui::Mxgui::${OPT_BOARD} ALIAS ${MXGUI_LIBRARY})
    target_include_directories(${MXGUI_LIBRARY} PUBLIC
        ${KPATH}/config
        ${KPATH}/config/${BOARD_INC}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/mxgui
        ${KPATH}
        ${KPATH}/arch/common
        ${KPATH}/${ARCH_INC}
        ${KPATH}/${BOARD_INC}
    )
    target_compile_options(${MXGUI_LIBRARY} PUBLIC ${DFLAGS} $<$<COMPILE_LANGUAGE:C>:${CFLAGS_BASE}> $<$<COMPILE_LANGUAGE:CXX>:${CXXFLAGS_BASE}> $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>)
    target_compile_definitions(${MXGUI_LIBRARY} PRIVATE MXGUI_LIBRARY)
endforeach()
