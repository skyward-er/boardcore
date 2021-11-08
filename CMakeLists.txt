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
include(cmake/sbs.cmake)

#-------------------------------#
#            Project            #
#-------------------------------#

project(SkywardBoardcore)

#-------------------------------#
#          Entrypoint           #
#-------------------------------#

add_executable(config-dsgamma
    src/entrypoints/config-dsgamma.cpp
    ${SHARED_SOURCES}
    ${GAMMA868_SOURCES}
)
target_link_libraries(config-dsgamma PRIVATE fmt::fmt-header-only)
set_property(TARGET config-dsgamma PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(config-dsgamma)

#-------------------------------#
#          Entrypoint           #
#-------------------------------#

add_executable(kernel-testsuite src/entrypoints/kernel-testsuite.cpp)
set_property(TARGET kernel-testsuite PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(kernel-testsuite)

#-------------------------------#
#          Entrypoint           #
#-------------------------------#

add_executable(mxgui-helloworld src/entrypoints/examples/mxgui-helloworld.cpp)
target_compile_definitions(mxgui-helloworld PRIVATE DEBUG)
set_property(TARGET mxgui-helloworld PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_link_mxgui(mxgui-helloworld)
sbs_target(mxgui-helloworld)

#-------------------------------#
#          Entrypoint           #
#-------------------------------#

add_executable(sdcard-benchmark
    src/entrypoints/sdcard-benchmark.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(sdcard-benchmark PRIVATE fmt::fmt-header-only)
set_property(TARGET sdcard-benchmark PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(sdcard-benchmark)

#-------------------------------#
#          Entrypoint           #
#-------------------------------#

add_executable(bmx160-calibration-entry
    src/entrypoints/bmx160-calibration-entry.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${BMX160_SOURCES}
    ${BMX160WITHCORRECTION_SOURCES}
    ${CALIBRATION_SOURCES}
)
target_link_libraries(bmx160-calibration-entry PRIVATE Eigen3::Eigen fmt::fmt-header-only)
set_property(TARGET bmx160-calibration-entry PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(bmx160-calibration-entry)

## Catch

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(tests-catch
    src/tests/catch/catch-tests-entry.cpp
    ${SHARED_SOURCES}
    ${TESTS_BOARDCORE_SOURCES}
    ${TEST_UTILS_SOURCES}
    ${SPI_SOURCES}
    ${SENSORMANAGER_SOURCES}
    ${XBEE_SOURCES}
)
target_link_libraries(tests-catch PRIVATE Eigen3::Eigen fmt::fmt-header-only)
target_compile_definitions(tests-catch PRIVATE USE_MOCK_PERIPHERALS)
set_property(TARGET tests-catch PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(tests-catch)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-kalman
#     src/tests/catch/test-kalman.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-kalman PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-kalman PRIVATE DEBUG STANDALONE_CATCH1_TEST)
# set_property(TARGET test-kalman PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-kalman)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-kalman-eigen
#     src/tests/catch/test-kalman-eigen.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-kalman-eigen PRIVATE Eigen3::Eigen fmt::fmt-header-only)
# target_compile_definitions(test-kalman-eigen PRIVATE DEBUG STANDALONE_CATCH1_TEST)
# set_property(TARGET test-kalman-eigen PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-kalman-eigen)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-taskscheduler
    src/tests/test-taskscheduler.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-taskscheduler PRIVATE fmt::fmt-header-only)
set_property(TARGET test-taskscheduler PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-taskscheduler)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-serial
    src/tests/test-serial.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-serial PRIVATE fmt::fmt-header-only)
set_property(TARGET test-serial PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-serial)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-trace-logger
    src/tests/test-trace-logger.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-trace-logger PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-trace-logger PRIVATE DEBUG)
set_property(TARGET test-trace-logger PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-trace-logger)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-hsm
    src/tests/test-hsm.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-hsm PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-hsm PRIVATE DEBUG)
set_property(TARGET test-hsm PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-hsm)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-interrupt
    src/tests/test-interrupt.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-interrupt PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-interrupt PRIVATE DEBUG)
set_property(TARGET test-interrupt PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-interrupt)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-timestamp-timer
    src/tests/test-timestamp-timer.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-timestamp-timer PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-timestamp-timer PRIVATE DEBUG)
set_property(TARGET test-timestamp-timer PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-timestamp-timer)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-logger
    src/tests/logger/test-logger.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-logger PRIVATE fmt::fmt-header-only)
set_property(TARGET test-logger PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-logger)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-kalman-benchmark src/tests/kalman/test-kalman-benchmark.cpp)
# set_property(TARGET test-kalman-benchmark PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-kalman-benchmark)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-kalman-eigen-benchmark
#     src/tests/kalman/test-kalman-eigen-benchmark.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-kalman-eigen-benchmark PRIVATE Eigen3::Eigen fmt::fmt-header-only)
# set_property(TARGET test-kalman-eigen-benchmark PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-kalman-eigen-benchmark)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-pinobserver
    src/tests/test-pinobserver.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-pinobserver PRIVATE fmt::fmt-header-only)
set_property(TARGET test-pinobserver PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-pinobserver)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-sensormanager-catch
#     src/tests/catch/test-sensormanager-catch.cpp
#     ${SHARED_SOURCES}
#     ${SENSORMANAGER_SOURCES}
# )
# target_link_libraries(test-sensormanager-catch PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-sensormanager-catch PRIVATE STANDALONE_CATCH1_TEST)
# set_property(TARGET test-sensormanager-catch PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
# sbs_target(test-sensormanager-catch)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-sensormanager
    src/tests/test-sensormanager.cpp
    ${SHARED_SOURCES}
    ${SENSORMANAGER_SOURCES}
)
target_link_libraries(test-sensormanager PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-sensormanager PRIVATE DEBUG)
set_property(TARGET test-sensormanager PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-sensormanager)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-calibration
    src/tests/calibration/test-calibration.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${CALIBRATION_SOURCES}
)
target_link_libraries(test-calibration PRIVATE Eigen3::Eigen fmt::fmt-header-only)
target_compile_definitions(test-calibration PRIVATE DEBUG)
set_property(TARGET test-calibration PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-calibration)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-calibration-benchmark
#     src/tests/calibration/test-calibration-benchmark.cpp
#     ${SHARED_SOURCES}
#     ${SPI_SOURCES}
#     ${CALIBRATION_SOURCES}
# )
# target_link_libraries(test-calibration-benchmark PRIVATE Eigen3::Eigen fmt::fmt-header-only)
# target_compile_definitions(test-calibration-benchmark PRIVATE DEBUG)
# set_property(TARGET test-calibration-benchmark PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
# sbs_target(test-calibration-benchmark)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-calibration-stats
#     src/tests/calibration/test-calibration-stats.cpp
#     ${SHARED_SOURCES}
#     ${SPI_SOURCES}
#     ${CALIBRATION_SOURCES}
# )
# target_link_libraries(test-calibration-stats PRIVATE Eigen3::Eigen fmt::fmt-header-only)
# target_compile_definitions(test-calibration-stats PRIVATE DEBUG)
# set_property(TARGET test-calibration-stats PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
# sbs_target(test-calibration-stats)

## Drivers

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-dsgamma
    src/tests/drivers/test-dsgamma.cpp
    ${SHARED_SOURCES}
    ${GAMMA868_SOURCES}
)
target_link_libraries(test-dsgamma PRIVATE fmt::fmt-header-only)
set_property(TARGET test-dsgamma PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-dsgamma)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-canbus
    src/tests/drivers/test-canbus.cpp
    ${SHARED_SOURCES}
    ${CANBUS_SOURCES}
)
target_link_libraries(test-canbus PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-canbus PRIVATE DEBUG)
set_property(TARGET test-canbus PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-canbus)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-pwm
    src/tests/drivers/test-pwm.cpp
    ${SHARED_SOURCES}
    ${PWM_SOURCES}
)
target_link_libraries(test-pwm PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-pwm PRIVATE DEBUG)
set_property(TARGET test-pwm PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-pwm)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-lis3mdl
    src/tests/drivers/test-lis3mdl.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
)
target_link_libraries(test-lis3mdl PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-lis3mdl PRIVATE DEBUG)
set_property(TARGET test-lis3mdl PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-lis3mdl)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-mavlink
    src/tests/drivers/test-mavlink.cpp
    ${SHARED_SOURCES}
    ${GAMMA868_SOURCES}
)
target_link_libraries(test-mavlink PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-mavlink PRIVATE DEBUG)
set_property(TARGET test-mavlink PROPERTY OPT_BOARD stm32f429zi_skyward_homeone)
sbs_target(test-mavlink)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(xbee-bitrate
#     src/tests/misc/xbee-bitrate.cpp
#     ${SHARED_SOURCES}
#     ${XBEE_SOURCES}
#     ${SPI_SOURCES}
# )
# target_link_libraries(xbee-bitrate PRIVATE fmt::fmt-header-only)
# target_compile_definitions(xbee-bitrate PRIVATE DEBUG SDRAM_ISSI)
# set_property(TARGET xbee-bitrate PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(xbee-bitrate)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(xbee-send-rcv
#     src/tests/misc/xbee-send-rcv.cpp
#     ${SHARED_SOURCES}
#     ${XBEE_SOURCES}
#     ${SPI_SOURCES}
# )
# target_link_libraries(xbee-send-rcv PRIVATE fmt::fmt-header-only)
# target_compile_definitions(xbee-send-rcv PRIVATE DEBUG)
# set_property(TARGET xbee-send-rcv PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(xbee-send-rcv)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(xbee-time-to-send
#     src/tests/misc/xbee-time-to-send.cpp
#     ${SHARED_SOURCES}
#     ${XBEE_SOURCES}
#     ${SPI_SOURCES}
# )
# target_link_libraries(xbee-time-to-send PRIVATE fmt::fmt-header-only)
# target_compile_definitions(xbee-time-to-send PRIVATE DEBUG)
# set_property(TARGET xbee-time-to-send PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(xbee-time-to-send)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-circularbuffer
#     src/tests/catch/test-circularbuffer.cpp
#     ${SHARED_SOURCES}
#     ${TEST_UTILS_SOURCES}
# )
# target_link_libraries(test-circularbuffer PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-circularbuffer PRIVATE STANDALONE_CATCH1_TEST)
# set_property(TARGET test-circularbuffer PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-circularbuffer)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-eventbroker
#     src/tests/catch/test-eventbroker.cpp
#     ${SHARED_SOURCES}
#     ${TEST_UTILS_SOURCES}
# )
# target_link_libraries(test-eventbroker PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-eventbroker PRIVATE STANDALONE_CATCH1_TEST)
# set_property(TARGET test-eventbroker PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-eventbroker)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-servo
    src/tests/drivers/test-servo.cpp
    ${SHARED_SOURCES}
    ${SERVO_SOURCES}
    ${PWM_SOURCES}
)
target_link_libraries(test-servo PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-servo PRIVATE DEBUG)
set_property(TARGET test-servo PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-servo)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-hbridge
    src/tests/drivers/test-hbridge.cpp
    ${SHARED_SOURCES}
    ${HBRIDGE_SOURCES}
    ${PWM_SOURCES}
)
target_link_libraries(test-hbridge PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-hbridge PRIVATE DEBUG)
set_property(TARGET test-hbridge PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-hbridge)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-ms5803
    src/tests/drivers/test-ms5803.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${SENSORMANAGER_SOURCES}
    ${MS5803_SOURCES}
)
target_link_libraries(test-ms5803 PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-ms5803 PRIVATE DEBUG)
set_property(TARGET test-ms5803 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-ms5803)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-ublox
    src/tests/drivers/test-ublox.cpp
    ${SHARED_SOURCES}
    ${UBLOXGPS_SOURCES}
)
target_link_libraries(test-ublox PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-ublox PRIVATE DEBUG)
set_property(TARGET test-ublox PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-ublox)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-l3gd20
    src/tests/drivers/test-l3gd20.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
)
target_link_libraries(test-l3gd20 PRIVATE fmt::fmt-header-only)
set_property(TARGET test-l3gd20 PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-l3gd20)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-l3gd20-fifo
    src/tests/drivers/test-l3gd20-fifo.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
)
target_link_libraries(test-l3gd20-fifo PRIVATE fmt::fmt-header-only)
set_property(TARGET test-l3gd20-fifo PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-l3gd20-fifo)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-rls
#     src/tests/test-rls.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-rls PRIVATE fmt::fmt-header-only)
# set_property(TARGET test-rls PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(test-rls)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-lis3dsh
    src/tests/drivers/test-lis3dsh.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
)
target_link_libraries(test-lis3dsh PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-lis3dsh PRIVATE DEBUG)
set_property(TARGET test-lis3dsh PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-lis3dsh)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-bmx160
    src/tests/drivers/test-bmx160.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${BMX160_SOURCES}
)
target_link_libraries(test-bmx160 PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-bmx160 PRIVATE DEBUG)
set_property(TARGET test-bmx160 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-bmx160)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-bmx160-with-correction
    src/tests/drivers/test-bmx160-with-correction.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${BMX160_SOURCES}
    ${BMX160WITHCORRECTION_SOURCES}
    ${CALIBRATION_SOURCES}
)
target_link_libraries(test-bmx160-with-correction PRIVATE Eigen3::Eigen fmt::fmt-header-only)
target_compile_definitions(test-bmx160-with-correction PRIVATE DEBUG)
set_property(TARGET test-bmx160-with-correction PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-bmx160-with-correction)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-ads1118
    src/tests/drivers/test-ads1118.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${ADS1118_SOURCES}
)
target_link_libraries(test-ads1118 PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-ads1118 PRIVATE DEBUG)
set_property(TARGET test-ads1118 PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-ads1118)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-xbee-snd
    src/tests/drivers/xbee/test-xbee-snd.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${XBEE_SOURCES}
)
target_link_libraries(test-xbee-snd PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-xbee-snd PRIVATE DEBUG)
set_property(TARGET test-xbee-snd PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-xbee-snd)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-xbee-rcv
    src/tests/drivers/xbee/test-xbee-rcv.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${XBEE_SOURCES}
)
target_link_libraries(test-xbee-rcv PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-xbee-rcv PRIVATE DEBUG)
set_property(TARGET test-xbee-rcv PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-xbee-rcv)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-xbee-bidir
    src/tests/drivers/xbee/test-xbee-bidir.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${XBEE_SOURCES}
)
target_link_libraries(test-xbee-bidir PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-xbee-bidir PRIVATE DEBUG)
set_property(TARGET test-xbee-bidir PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-xbee-bidir)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-xbee-gui
    src/tests/drivers/xbee/test-xbee-gui.cpp
    src/tests/drivers/xbee/gui/res/respect.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${XBEE_SOURCES}
)
target_link_libraries(test-xbee-gui PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-xbee-gui PRIVATE DEBUG)
set_property(TARGET test-xbee-gui PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_link_mxgui(test-xbee-gui)
sbs_target(test-xbee-gui)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-bme280
    src/tests/drivers/test-bme280.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${BME280_SOURCES}
)
target_link_libraries(test-bme280 PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-bme280 PRIVATE DEBUG)
set_property(TARGET test-bme280 PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-bme280)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-internal-adc
    src/tests/drivers/test-internal-adc.cpp
    ${SHARED_SOURCES}
    ${INTERNAL_ADC_SOURCES}
)
target_link_libraries(test-internal-adc PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-internal-adc PRIVATE DEBUG)
set_property(TARGET test-internal-adc PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-internal-adc)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-internal-adc-dma
    src/tests/drivers/test-internal-adc-dma.cpp
    ${SHARED_SOURCES}
    ${INTERNAL_ADC_SOURCES}
)
target_link_libraries(test-internal-adc-dma PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-internal-adc-dma PRIVATE DEBUG)
set_property(TARGET test-internal-adc-dma PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-internal-adc-dma)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-analog-pressure-sensors
    src/tests/drivers/analog/test-analog-pressure-sensors.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${ADS1118_SOURCES}
)
target_link_libraries(test-analog-pressure-sensors PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-analog-pressure-sensors PRIVATE DEBUG)
set_property(TARGET test-analog-pressure-sensors PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-analog-pressure-sensors)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-battery-voltage
    src/tests/drivers/analog/test-battery-voltage.cpp
    ${SHARED_SOURCES}
    ${INTERNAL_ADC_SOURCES}
)
target_link_libraries(test-battery-voltage PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-battery-voltage PRIVATE DEBUG)
set_property(TARGET test-battery-voltage PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-battery-voltage)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-current-sensor
    src/tests/drivers/analog/test-current-sensor.cpp
    ${SHARED_SOURCES}
    ${INTERNAL_ADC_SOURCES}
)
target_link_libraries(test-current-sensor PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-current-sensor PRIVATE DEBUG)
set_property(TARGET test-current-sensor PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-current-sensor)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-spi
    src/tests/drivers/test-spi.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
)
target_link_libraries(test-spi PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-spi PRIVATE DEBUG)
set_property(TARGET test-spi PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-spi)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-hwtimer-chain
    src/tests/drivers/test-hwtimer-chain.cpp
    ${SHARED_SOURCES}
)
target_link_libraries(test-hwtimer-chain PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-hwtimer-chain PRIVATE DEBUG)
set_property(TARGET test-hwtimer-chain PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-hwtimer-chain)

#-------------------------------#
#             Test              #
#-------------------------------#

add_executable(test-mpu9250
    src/tests/drivers/test-mpu9250.cpp
    ${SHARED_SOURCES}
    ${SPI_SOURCES}
    ${MPU9250_SOURCES}
)
target_link_libraries(test-mpu9250 PRIVATE fmt::fmt-header-only)
target_compile_definitions(test-mpu9250 PRIVATE DEBUG)
set_property(TARGET test-mpu9250 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-mpu9250)

## Older drivers (not up-to-date)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-piksi
#     src/tests/drivers/test-piksi.cpp
#     ${SHARED_SOURCES}
#     ${PIKSI_SOURCES}
# )
# target_link_libraries(test-piksi PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-piksi PRIVATE DEBUG)
# set_property(TARGET test-piksi PROPERTY OPT_BOARD stm32f429zi_skyward_homeone)
# sbs_target(test-piksi)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-imu-adis
#     src/tests/drivers/old_examples/test-imu-adis.cpp
#     ${SHARED_SOURCES}
#     ${SENSORMANAGER_SOURCES}
# )
# set_property(TARGET test-imu-adis PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(test-imu-adis)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-lsm
#     src/tests/drivers/old_examples/test-lsm.cpp
#     ${SHARED_SOURCES}
#     ${SENSORMANAGER_SOURCES}
# )
# target_link_libraries(test-lsm PRIVATE fmt::fmt-header-only)
# set_property(TARGET test-lsm PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-lsm)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-IMU
#     src/tests/drivers/old_examples/test-IMU.cpp
#     ${SHARED_SOURCES}
# )
# set_property(TARGET test-IMU PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
# sbs_target(test-IMU)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-mavlink-multi
#     src/tests/drivers/test-mavlink-multi.cpp
#     ${SHARED_SOURCES}
#     ${GAMMA868_SOURCES}
# )
# target_link_libraries(test-mavlink-multi PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-mavlink-multi PRIVATE DEBUG)
# set_property(TARGET test-mavlink-multi PROPERTY OPT_BOARD stm32f429zi_skyward_homeone)
# sbs_target(test-mavlink-multi)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-ad7994
#     src/tests/drivers/old_examples/test-ad7994.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-ad7994 PRIVATE fmt::fmt-header-only)
# set_property(TARGET test-ad7994 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(test-ad7994)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-tempSensor
#     src/tests/drivers/old_examples/test-tempSensor.cpp
#     ${SHARED_SOURCES}
# )
# target_compile_definitions(test-tempSensor PRIVATE DEBUG)
# set_property(TARGET test-tempSensor PROPERTY OPT_BOARD stm32f429zi_skyward_homeone)
# sbs_target(test-tempSensor)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-spi2
#     src/tests/drivers/old_examples/test-spi2.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-spi2 PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-spi2 PRIVATE DEBUG SDRAM_ISSI)
# set_property(TARGET test-spi2 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(test-spi2)

#-------------------------------#
#             Test              #
#-------------------------------#

## This works
# add_executable(test-i2c-mpu9255
#     src/tests/drivers/old_examples/test-i2c-mpu9255.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-i2c-mpu9255 PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-i2c-mpu9255 PRIVATE DEBUG SDRAM_ISSI)
# set_property(TARGET test-i2c-mpu9255 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(test-i2c-mpu9255)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(test-ad7994-bare
#     src/tests/drivers/old_examples/test-ad7994-bare.cpp
#     ${SHARED_SOURCES}
# )
# target_link_libraries(test-ad7994-bare PRIVATE fmt::fmt-header-only)
# target_compile_definitions(test-ad7994-bare PRIVATE DEBUG SDRAM_ISSI)
# set_property(TARGET test-ad7994-bare PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(test-ad7994-bare)

#-------------------------------#
#             Test              #
#-------------------------------#

# add_executable(calibrate-mpu9250
#     src/tests/drivers/old_examples/calibrate-mpu9250.cpp
#     ${SHARED_SOURCES}
#     ${SENSORMANAGER_SOURCES}
# )
# set_property(TARGET calibrate-mpu9250 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack)
# sbs_target(calibrate-mpu9250)
