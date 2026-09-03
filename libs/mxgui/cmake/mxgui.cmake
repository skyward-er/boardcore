# Copyright (C) 2023 by Skyward
#
# This program is free software; you can redistribute it and/or 
# it under the terms of the GNU General Public License as published 
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# As a special exception, if other files instantiate templates or use
# macros or inline functions from this file, or you compile this file
# and link it with other works to produce a work based on this file,
# this file does not by itself cause the resulting work to be covered
# by the GNU General Public License. However the source code for this
# file must still be made available in accordance with the GNU 
# Public License. This exception does not invalidate any other 
# why a work based on this file might be covered by the GNU General
# Public License.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>

# Load in MXGUI_PATH the project path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH MXGUI_PATH)
cmake_path(GET MXGUI_PATH PARENT_PATH MXGUI_PARENT_PATH)

# Include board list
include(${MXGUI_PATH}/cmake/boards.cmake)

# MxGui source files
set(MXGUI_SRC
    ${MXGUI_PATH}/display.cpp
    ${MXGUI_PATH}/font.cpp
    ${MXGUI_PATH}/misc_inst.cpp
    ${MXGUI_PATH}/tga_image.cpp
    ${MXGUI_PATH}/resourcefs.cpp
    ${MXGUI_PATH}/resource_image.cpp
    ${MXGUI_PATH}/level2/input.cpp
    ${MXGUI_PATH}/level2/application.cpp
    ${MXGUI_PATH}/level2/drawing_context_proxy.cpp
    ${MXGUI_PATH}/level2/label.cpp
    ${MXGUI_PATH}/level2/simple_plot.cpp
    ${MXGUI_PATH}/drivers/display_stm32f4discovery.cpp
    ${MXGUI_PATH}/drivers/event_stm32f4discovery.cpp
)

# Creates the MxGui::${BOARD_NAME} library
function(add_mxgui_library BOARD_OPTIONS_FILE)
    # Get board options
    include(${BOARD_OPTIONS_FILE})

    # Create a library for the board
    set(MXGUI_LIB mxgui-${BOARD_NAME})
    add_library(${MXGUI_LIB} STATIC EXCLUDE_FROM_ALL ${MXGUI_SRC})

    # Include MxGui directories
    target_include_directories(${MXGUI_LIB} PUBLIC
        ${KPATH}
        ${KPATH}/arch/common
        ${ARCH_PATH}
        ${BOARD_PATH}
        ${BOARD_CONFIG_PATH}
        ${MXGUI_PARENT_PATH}
        ${MXGUI_PATH}
    )

    # Set include path where to find config/miosix_settings.h
    if(DEFINED BOARD_MIOSIX_SETTINGS_PATH)
        target_include_directories(${MXGUI_LIB} PRIVATE ${BOARD_MIOSIX_SETTINGS_PATH})
    else()
        target_include_directories(${MXGUI_LIB} PRIVATE ${KPATH}/default)
    endif()

    # Define MXGUI_LIB (only for C and C++)
    target_compile_definitions(${MXGUI_LIB} PRIVATE $<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:MXGUI_LIBRARY>)

    # Require cpp14 target_compile_features (this will add the -std=c++14 flag)
    target_compile_features(${MXGUI_LIB} PUBLIC cxx_std_14)

    # Configure compiler flags
    target_compile_options(${MXGUI_LIB} PRIVATE
        $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:C>:${DFLAGS} ${CFLAGS_BASE}>
        $<$<COMPILE_LANGUAGE:CXX>:${DFLAGS} ${CXXFLAGS_BASE}>
    )

    # Create a nice alias for the library
    add_library(MxGui::${BOARD_NAME} ALIAS ${MXGUI_LIB})
endfunction()

# Create MxGui libraries for the supported boards
foreach(BOARD_OPTIONS_FILE ${MXGUI_BOARDS_OPTIONS_FILES})
    add_mxgui_library(${BOARD_OPTIONS_FILE})
endforeach()
