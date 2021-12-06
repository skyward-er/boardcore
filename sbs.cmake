# Copyright (c) 2021 Skyward Experimental Rocketry
# Authors: Damiano Amatruda
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

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")

set(SBS_BASE ${CMAKE_CURRENT_LIST_DIR})

list(APPEND CMAKE_MODULE_PATH ${SBS_BASE}/cmake)

set(KPATH ${SBS_BASE}/libs/miosix-kernel/miosix)
file(GLOB KPATH ${KPATH})
if(NOT KPATH)
    message(FATAL_ERROR "Kernel not found")
endif()

include(dependencies)

function(sbs_target TARGET)
    get_target_property(OPT_BOARD ${TARGET} OPT_BOARD)
    if(NOT OPT_BOARD)
        message(FATAL_ERROR "No board selected")
    endif()
    
    set(MIOSIX_LIBRARY miosix-${OPT_BOARD})

    set(DFLAGS -MMD -MP)
    include(${KPATH}/config/options.cmake)

    target_sources(${TARGET} PRIVATE ${BOOT_FILE})
    target_include_directories(${TARGET} PRIVATE
        ${SBS_BASE}
        ${SBS_BASE}/libs
        ${SBS_BASE}/src/shared
        src/shared
    )
    set_target_properties(${TARGET} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
    target_compile_options(${TARGET} PRIVATE ${DFLAGS} $<$<COMPILE_LANGUAGE:C>:${CFLAGS_BASE}> $<$<COMPILE_LANGUAGE:CXX>:${CXXFLAGS_BASE}> $<$<COMPILE_LANGUAGE:ASM>:${AFLAGS_BASE}>)
    target_link_options(${TARGET} PRIVATE ${LFLAGS_BASE})
    target_link_libraries(${TARGET} ${MIOSIX_LIBRARY} stdc++ c m gcc atomic)
    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET} ${TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${TARGET}.bin
        BYPRODUCTS ${TARGET}.hex ${TARGET}.bin
    )
endfunction()
