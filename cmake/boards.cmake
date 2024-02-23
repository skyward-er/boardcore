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

set(BOARDCORE_BOARDS_OPTIONS_FILES
    ${BOARDCORE_PATH}/src/bsps/stm32f205rc_ciuti/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_nokia/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_parafoil/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_death_stack_v1/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_death_stack_v2/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_death_stack_v3/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_pyxis_auxiliary/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_rig/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f429zi_con_rig/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f756zg_nucleo/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_automated_antennas/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_compute_unit/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_compute_unit_v2/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_compute_unit_v2/config/board_options_no_xram.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_gemini_gs/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_gemini_motor/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_death_stack_v4/config/board_options.cmake
    ${BOARDCORE_PATH}/src/bsps/stm32f767zi_rig_v2/config/board_options.cmake
)
