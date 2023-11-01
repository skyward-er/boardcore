# Copyright (c) 2023 Skyward Experimental Rocketry
# Authors: Alberto Nidasio
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

set(BOARDCORE_BOARDS
    stm32f205rc_skyward_ciuti
    stm32f429zi_skyward_death_stack_v1
    stm32f429zi_skyward_death_stack_v2
    stm32f429zi_skyward_death_stack_v3
    stm32f429zi_nokia
    stm32f429zi_parafoil
    stm32f429zi_skyward_pyxis_auxiliary
    stm32f429zi_skyward_rig
    stm32f756zg_nucleo
    stm32f767zi_automated_antennas
    stm32f767zi_compute_unit
    stm32f767zi_gemini_gs
    stm32f767zi_gemini_motor
    stm32f767zi_skyward_death_stack_v4
)

function(get_mxgui_board BOARD)
    if(${BOARD} STREQUAL stm32f429zi_nokia)
        set(MXGUI_BOARD stm32f429zi_stm32f4discovery PARENT_SCOPE)
    endif()
endfunction()