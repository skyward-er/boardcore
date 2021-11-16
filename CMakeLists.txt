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

#-----------------------------------------------------------------------------#
#                                   Project                                   #
#-----------------------------------------------------------------------------#

project(SkywardBoardcore)

#-----------------------------------------------------------------------------#
#                                 Entrypoints                                 #
#-----------------------------------------------------------------------------#

#src/entrypoints/anakin/anakin-accel.cpp
#src/entrypoints/anakin/anakin-demo-board.cpp
#src/entrypoints/anakin/anakin-test-canbus.cpp
#src/entrypoints/anakin/anakin-test-dma.cpp
#src/entrypoints/anakin/anakin-test-suite.cpp
#src/entrypoints/anakin/canbus-test.cpp
#src/entrypoints/anakin/dma-lowlevel-test.cpp
#src/entrypoints/anakin/test-busfault.cpp
#src/entrypoints/anakin/test-scheduler.cpp
#src/entrypoints/anakin/test_ethernet.cpp
#src/entrypoints/anakin/test_i2c.cpp

add_executable(bmx160-calibration-entry src/entrypoints/bmx160-calibration-entry.cpp)
set_property(TARGET bmx160-calibration-entry PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(bmx160-calibration-entry)

add_executable(config-dsgamma src/entrypoints/config-dsgamma.cpp)
set_property(TARGET config-dsgamma PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(config-dsgamma)

#src/entrypoints/examples/example-fsm.cpp

add_executable(mxgui-helloworld src/entrypoints/examples/mxgui-helloworld.cpp)
set_property(TARGET mxgui-helloworld PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(mxgui-helloworld)

add_executable(kernel-testsuite src/entrypoints/kernel-testsuite.cpp)
set_property(TARGET kernel-testsuite PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(kernel-testsuite)

add_executable(sdcard-benchmark src/entrypoints/sdcard-benchmark.cpp)
set_property(TARGET sdcard-benchmark PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(sdcard-benchmark)

#-----------------------------------------------------------------------------#
#                                Tests - Catch                                #
#-----------------------------------------------------------------------------#

add_executable(test-calibration-benchmark src/tests/calibration/test-calibration-benchmark.cpp)
set_property(TARGET test-calibration-benchmark PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-calibration-benchmark)

add_executable(test-calibration-stats src/tests/calibration/test-calibration-stats.cpp)
set_property(TARGET test-calibration-stats PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-calibration-stats)

add_executable(test-calibration src/tests/calibration/test-calibration.cpp)
set_property(TARGET test-calibration PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-calibration)

add_executable(tests-catch src/tests/catch/catch-tests-entry.cpp)
target_compile_definitions(tests-catch PUBLIC USE_MOCK_PERIPHERALS)
set_property(TARGET tests-catch PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(tests-catch)

#src/tests/catch/examples/example-test-factorial.cpp
#src/tests/catch/examples/example-test-fsm.cpp
#src/tests/catch/spidriver/test-spidriver.cpp
#src/tests/catch/test-aero.cpp
#src/tests/catch/test-buttonhandler.cpp

add_executable(test-circularbuffer src/tests/catch/test-circularbuffer.cpp)
target_compile_definitions(test-circularbuffer PRIVATE STANDALONE_CATCH1_TEST)
set_property(TARGET test-circularbuffer PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-circularbuffer)

add_executable(test-eventbroker src/tests/catch/test-eventbroker.cpp)
target_compile_definitions(test-eventbroker PRIVATE STANDALONE_CATCH1_TEST)
set_property(TARGET test-eventbroker PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-eventbroker)

#src/tests/catch/test-hardwaretimer.cpp

add_executable(test-kalman-eigen src/tests/catch/test-kalman-eigen.cpp)
target_compile_definitions(test-kalman-eigen PRIVATE STANDALONE_CATCH1_TEST)
set_property(TARGET test-kalman-eigen PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-kalman-eigen)

#src/tests/catch/test-packetqueue.cpp

add_executable(test-sensormanager-catch src/tests/catch/test-sensormanager-catch.cpp)
target_compile_definitions(test-sensormanager-catch PRIVATE STANDALONE_CATCH1_TEST)
set_property(TARGET test-sensormanager-catch PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-sensormanager-catch)

#src/tests/catch/test-xbee.cpp
#src/tests/catch/xbee/test-xbee-driver.cpp
#src/tests/catch/xbee/test-xbee-parser.cpp

#-----------------------------------------------------------------------------#
#                               Tests - Drivers                               #
#-----------------------------------------------------------------------------#

add_executable(test-analog-pressure-sensors src/tests/drivers/analog/test-analog-pressure-sensors.cpp)
set_property(TARGET test-analog-pressure-sensors PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-analog-pressure-sensors)

add_executable(test-battery-voltage src/tests/drivers/analog/test-battery-voltage.cpp)
set_property(TARGET test-battery-voltage PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-battery-voltage)

add_executable(test-current-sensor src/tests/drivers/analog/test-current-sensor.cpp)
set_property(TARGET test-current-sensor PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-current-sensor)

#src/tests/drivers/old_examples/test-ad7994-bare.cpp
#src/tests/drivers/old_examples/test-ad7994.cpp
#src/tests/drivers/old_examples/test-i2c-mpu9255.cpp
#src/tests/drivers/old_examples/test-imu-adis.cpp
#src/tests/drivers/old_examples/test-lsm.cpp
#src/tests/drivers/old_examples/test-spi2.cpp
#src/tests/drivers/old_examples/test-tempSensor.cpp
#src/tests/drivers/old_examples/test-unit-LM75B.cpp

add_executable(test-ads1118 src/tests/drivers/test-ads1118.cpp)
set_property(TARGET test-ads1118 PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-ads1118)

add_executable(test-bme280 src/tests/drivers/test-bme280.cpp)
set_property(TARGET test-bme280 PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-bme280)

add_executable(test-bmx160 src/tests/drivers/test-bmx160.cpp)
set_property(TARGET test-bmx160 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-bmx160)

add_executable(test-bmx160-with-correction src/tests/drivers/test-bmx160-with-correction.cpp)
set_property(TARGET test-bmx160-with-correction PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-bmx160-with-correction)

add_executable(test-canbus src/tests/drivers/test-canbus.cpp)
set_property(TARGET test-canbus PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-canbus)

add_executable(test-dsgamma src/tests/drivers/test-dsgamma.cpp)
set_property(TARGET test-dsgamma PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-dsgamma)

add_executable(test-hbridge src/tests/drivers/test-hbridge.cpp)
set_property(TARGET test-hbridge PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-hbridge)

add_executable(test-hwtimer-chain src/tests/drivers/test-hwtimer-chain.cpp)
set_property(TARGET test-hwtimer-chain PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-hwtimer-chain)

add_executable(test-internal-adc src/tests/drivers/test-internal-adc.cpp)
set_property(TARGET test-internal-adc PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-internal-adc)

add_executable(test-internal-adc-dma src/tests/drivers/test-internal-adc-dma.cpp)
set_property(TARGET test-internal-adc-dma PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-internal-adc-dma)

add_executable(test-l3gd20 src/tests/drivers/test-l3gd20.cpp)
set_property(TARGET test-l3gd20 PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-l3gd20)

add_executable(test-l3gd20-fifo src/tests/drivers/test-l3gd20-fifo.cpp)
set_property(TARGET test-l3gd20-fifo PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-l3gd20-fifo)

add_executable(test-lis3dsh src/tests/drivers/test-lis3dsh.cpp)
set_property(TARGET test-lis3dsh PROPERTY OPT_BOARD stm32f407vg_stm32f4discovery)
sbs_target(test-lis3dsh)

add_executable(test-lis3mdl src/tests/drivers/test-lis3mdl.cpp)
set_property(TARGET test-lis3mdl PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-lis3mdl)

#src/tests/drivers/test-mavlink-multi.cpp

add_executable(test-mavlink src/tests/drivers/test-mavlink.cpp)
set_property(TARGET test-mavlink PROPERTY OPT_BOARD stm32f429zi_skyward_homeone)
sbs_target(test-mavlink)

add_executable(test-mpu9250 src/tests/drivers/test-mpu9250.cpp)
set_property(TARGET test-mpu9250 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-mpu9250)

add_executable(test-ms5803 src/tests/drivers/test-ms5803.cpp)
set_property(TARGET test-ms5803 PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-ms5803)

#src/tests/drivers/test-piksi.cpp

add_executable(test-pwm src/tests/drivers/test-pwm.cpp)
set_property(TARGET test-pwm PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-pwm)

add_executable(test-servo src/tests/drivers/test-servo.cpp)
set_property(TARGET test-servo PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-servo)

add_executable(test-spi src/tests/drivers/test-spi.cpp)
set_property(TARGET test-spi PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-spi)

add_executable(test-ublox src/tests/drivers/test-ublox.cpp)
set_property(TARGET test-ublox PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-ublox)

#src/tests/drivers/xbee/logdecoder/logdecoder.cpp

add_executable(test-xbee-bidir src/tests/drivers/xbee/test-xbee-bidir.cpp)
set_property(TARGET test-xbee-bidir PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-xbee-bidir)

add_executable(test-xbee-gui
    src/tests/drivers/xbee/test-xbee-gui.cpp
    src/tests/drivers/xbee/gui/res/respect.cpp
)
set_property(TARGET test-xbee-gui PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-xbee-gui)

add_executable(test-xbee-rcv src/tests/drivers/xbee/test-xbee-rcv.cpp)
set_property(TARGET test-xbee-rcv PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-xbee-rcv)

add_executable(test-xbee-snd src/tests/drivers/xbee/test-xbee-snd.cpp)
set_property(TARGET test-xbee-snd PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-xbee-snd)

add_executable(test-kalman-eigen-benchmark src/tests/kalman/test-kalman-eigen-benchmark.cpp)
set_property(TARGET test-kalman-eigen-benchmark PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-kalman-eigen-benchmark)

add_executable(test-logger src/tests/logger/test-logger.cpp)
set_property(TARGET test-logger PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-logger)

#src/tests/misc/xbee-bitrate.cpp
#src/tests/misc/xbee-send-rcv.cpp
#src/tests/misc/xbee-time-to-send.cpp
#src/tests/test-eventinjector.cpp

add_executable(test-hsm src/tests/test-hsm.cpp)
set_property(TARGET test-hsm PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-hsm)

add_executable(test-interrupt src/tests/test-interrupt.cpp)
set_property(TARGET test-interrupt PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-interrupt)

add_executable(test-pinobserver src/tests/test-pinobserver.cpp)
set_property(TARGET test-pinobserver PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-pinobserver)

add_executable(test-sensormanager src/tests/test-sensormanager.cpp)
set_property(TARGET test-sensormanager PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-sensormanager)

add_executable(test-serial src/tests/test-serial.cpp)
set_property(TARGET test-serial PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-serial)

add_executable(test-taskscheduler src/tests/test-taskscheduler.cpp)
set_property(TARGET test-taskscheduler PROPERTY OPT_BOARD stm32f429zi_skyward_death_stack_x)
sbs_target(test-taskscheduler)

add_executable(test-timestamp-timer src/tests/test-timestamp-timer.cpp)
set_property(TARGET test-timestamp-timer PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-timestamp-timer)

add_executable(test-trace-logger src/tests/test-trace-logger.cpp)
set_property(TARGET test-trace-logger PROPERTY OPT_BOARD stm32f429zi_stm32f4discovery)
sbs_target(test-trace-logger)
