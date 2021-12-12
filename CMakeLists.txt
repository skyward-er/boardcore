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

add_executable(bmx160-calibration-entry src/entrypoints/bmx160-calibration-entry.cpp)
sbs_target(bmx160-calibration-entry stm32f429zi_skyward_death_stack_x)

add_executable(config-dsgamma src/entrypoints/config-dsgamma.cpp)
sbs_target(config-dsgamma stm32f429zi_stm32f4discovery)

add_executable(mxgui-helloworld src/entrypoints/examples/mxgui-helloworld.cpp)
sbs_target(mxgui-helloworld stm32f429zi_stm32f4discovery)

add_executable(kernel-testsuite src/entrypoints/kernel-testsuite.cpp)
sbs_target(kernel-testsuite stm32f429zi_stm32f4discovery)

#src/entrypoints/old_examples/anakin/anakin-accel.cpp
#src/entrypoints/old_examples/anakin/anakin-demo-board.cpp
#src/entrypoints/old_examples/anakin/anakin-test-canbus.cpp
#src/entrypoints/old_examples/anakin/anakin-test-dma.cpp
#src/entrypoints/old_examples/anakin/anakin-test-suite.cpp
#src/entrypoints/old_examples/anakin/canbus-test.cpp
#src/entrypoints/old_examples/anakin/dma-lowlevel-test.cpp
#src/entrypoints/old_examples/anakin/test-busfault.cpp
#src/entrypoints/old_examples/anakin/test-scheduler.cpp
#src/entrypoints/old_examples/anakin/test_ethernet.cpp
#src/entrypoints/old_examples/anakin/test_i2c.cpp
#src/entrypoints/old_examples/example-fsm.cpp

add_executable(runcam-settings src/entrypoints/runcam-settings.cpp)
sbs_target(runcam-settings stm32f407vg_stm32f4discovery)

add_executable(sdcard-benchmark src/entrypoints/sdcard-benchmark.cpp)
sbs_target(sdcard-benchmark stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                Tests - Catch                                #
#-----------------------------------------------------------------------------#

add_executable(test-calibration-benchmark src/tests/calibration/test-calibration-benchmark.cpp)
sbs_target(test-calibration-benchmark stm32f407vg_stm32f4discovery)

add_executable(test-calibration-stats src/tests/calibration/test-calibration-stats.cpp)
sbs_target(test-calibration-stats stm32f407vg_stm32f4discovery)

add_executable(test-calibration src/tests/calibration/test-calibration.cpp)
sbs_target(test-calibration stm32f407vg_stm32f4discovery)

#add_executable(catch-tests-entry src/tests/catch/catch-tests-entry.cpp)
#target_compile_definitions(catch-tests-entry PUBLIC USE_MOCK_PERIPHERALS)
#sbs_target(catch-tests-entry stm32f429zi_stm32f4discovery)

add_executable(example-test-factorial src/tests/catch/examples/example-test-factorial.cpp)
target_compile_definitions(example-test-factorial PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(example-test-factorial stm32f429zi_stm32f4discovery)

#src/tests/catch/old_examples/example-test-fsm.cpp
#src/tests/catch/old_examples/spidriver/test-spidriver.cpp
#src/tests/catch/old_examples/test-xbee.cpp
#src/tests/catch/old_examples/xbee/test-xbee-driver.cpp

add_executable(test-aero src/tests/catch/test-aero.cpp)
target_compile_definitions(test-aero PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-aero stm32f429zi_stm32f4discovery)

add_executable(test-buttonhandler src/tests/catch/test-buttonhandler.cpp)
target_compile_definitions(test-buttonhandler PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-buttonhandler stm32f429zi_stm32f4discovery)

add_executable(test-circularbuffer src/tests/catch/test-circularbuffer.cpp)
target_compile_definitions(test-circularbuffer PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-circularbuffer stm32f429zi_stm32f4discovery)

add_executable(test-eventbroker src/tests/catch/test-eventbroker.cpp)
target_compile_definitions(test-eventbroker PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-eventbroker stm32f429zi_stm32f4discovery)

add_executable(test-hardwaretimer src/tests/catch/test-hardwaretimer.cpp)
target_compile_definitions(test-hardwaretimer PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-hardwaretimer stm32f429zi_stm32f4discovery)

add_executable(test-kalman-eigen src/tests/catch/test-kalman-eigen.cpp)
target_compile_definitions(test-kalman-eigen PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-kalman-eigen stm32f429zi_stm32f4discovery)

add_executable(test-packetqueue src/tests/catch/test-packetqueue.cpp)
target_compile_definitions(test-packetqueue PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-packetqueue stm32f429zi_skyward_death_stack_x)

add_executable(test-sensormanager-catch src/tests/catch/test-sensormanager-catch.cpp)
target_compile_definitions(test-sensormanager-catch PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-sensormanager-catch stm32f429zi_skyward_death_stack_x)

add_executable(test-xbee-parser src/tests/catch/xbee/test-xbee-parser.cpp)
target_compile_definitions(test-xbee-parser PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-xbee-parser stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                               Tests - Drivers                               #
#-----------------------------------------------------------------------------#

add_executable(test-analog-pressure-sensors src/tests/drivers/analog/test-analog-pressure-sensors.cpp)
sbs_target(test-analog-pressure-sensors stm32f429zi_stm32f4discovery)

add_executable(test-battery-voltage src/tests/drivers/analog/test-battery-voltage.cpp)
sbs_target(test-battery-voltage stm32f429zi_stm32f4discovery)

add_executable(test-current-sensor src/tests/drivers/analog/test-current-sensor.cpp)
sbs_target(test-current-sensor stm32f429zi_stm32f4discovery)

#src/tests/drivers/old_examples/test-ad7994-bare.cpp
#src/tests/drivers/old_examples/test-ad7994.cpp
#src/tests/drivers/old_examples/test-i2c-mpu9255.cpp
#src/tests/drivers/old_examples/test-imu-adis.cpp
#src/tests/drivers/old_examples/test-lsm.cpp
#src/tests/drivers/old_examples/test-mavlink-multi.cpp
#src/tests/drivers/old_examples/test-piksi.cpp
#src/tests/drivers/old_examples/test-spi2.cpp
#src/tests/drivers/old_examples/test-tempSensor.cpp
#src/tests/drivers/old_examples/test-unit-LM75B.cpp

add_executable(test-ads1118 src/tests/drivers/test-ads1118.cpp)
sbs_target(test-ads1118 stm32f407vg_stm32f4discovery)

add_executable(test-bme280 src/tests/drivers/test-bme280.cpp)
sbs_target(test-bme280 stm32f407vg_stm32f4discovery)

add_executable(test-bmx160 src/tests/drivers/test-bmx160.cpp)
sbs_target(test-bmx160 stm32f429zi_skyward_death_stack_x)

add_executable(test-bmx160-with-correction src/tests/drivers/test-bmx160-with-correction.cpp)
sbs_target(test-bmx160-with-correction stm32f429zi_skyward_death_stack_x)

add_executable(test-canbus src/tests/drivers/test-canbus.cpp)
sbs_target(test-canbus stm32f429zi_skyward_death_stack_x)

add_executable(test-dsgamma src/tests/drivers/test-dsgamma.cpp)
sbs_target(test-dsgamma stm32f429zi_stm32f4discovery)

add_executable(test-hbridge src/tests/drivers/test-hbridge.cpp)
sbs_target(test-hbridge stm32f429zi_stm32f4discovery)

add_executable(test-hwtimer-chain src/tests/drivers/test-hwtimer-chain.cpp)
sbs_target(test-hwtimer-chain stm32f407vg_stm32f4discovery)

add_executable(test-internal-adc src/tests/drivers/test-internal-adc.cpp)
sbs_target(test-internal-adc stm32f429zi_skyward_death_stack_x)

add_executable(test-internal-adc-dma src/tests/drivers/test-internal-adc-dma.cpp)
sbs_target(test-internal-adc-dma stm32f429zi_stm32f4discovery)

add_executable(test-l3gd20 src/tests/drivers/test-l3gd20.cpp)
sbs_target(test-l3gd20 stm32f429zi_stm32f4discovery)

add_executable(test-l3gd20-fifo src/tests/drivers/test-l3gd20-fifo.cpp)
sbs_target(test-l3gd20-fifo stm32f429zi_stm32f4discovery)

add_executable(test-lis3dsh src/tests/drivers/test-lis3dsh.cpp)
sbs_target(test-lis3dsh stm32f407vg_stm32f4discovery)

add_executable(test-lis3mdl src/tests/drivers/test-lis3mdl.cpp)
sbs_target(test-lis3mdl stm32f429zi_skyward_death_stack_x)

add_executable(test-mavlink src/tests/drivers/test-mavlink.cpp)
sbs_target(test-mavlink stm32f429zi_skyward_homeone)

add_executable(test-mpu9250 src/tests/drivers/test-mpu9250.cpp)
sbs_target(test-mpu9250 stm32f429zi_skyward_death_stack_x)

add_executable(test-ms5803 src/tests/drivers/test-ms5803.cpp)
sbs_target(test-ms5803 stm32f429zi_skyward_death_stack_x)

add_executable(test-pwm src/tests/drivers/test-pwm.cpp)
sbs_target(test-pwm stm32f429zi_stm32f4discovery)

add_executable(test-servo src/tests/drivers/test-servo.cpp)
sbs_target(test-servo stm32f429zi_skyward_death_stack_x)

add_executable(test-spi src/tests/drivers/test-spi.cpp)
sbs_target(test-spi stm32f429zi_stm32f4discovery)

add_executable(test-ublox src/tests/drivers/test-ublox.cpp)
sbs_target(test-ublox stm32f429zi_skyward_death_stack_x)

#src/tests/drivers/xbee/logdecoder/logdecoder.cpp

add_executable(test-xbee-bidir src/tests/drivers/xbee/test-xbee-bidir.cpp)
sbs_target(test-xbee-bidir stm32f429zi_skyward_death_stack_x)

add_executable(test-xbee-gui
    src/tests/drivers/xbee/test-xbee-gui.cpp
    src/tests/drivers/xbee/gui/res/respect.cpp
)
sbs_target(test-xbee-gui stm32f429zi_stm32f4discovery)

add_executable(test-xbee-rcv src/tests/drivers/xbee/test-xbee-rcv.cpp)
sbs_target(test-xbee-rcv stm32f429zi_stm32f4discovery)

add_executable(test-xbee-snd src/tests/drivers/xbee/test-xbee-snd.cpp)
sbs_target(test-xbee-snd stm32f429zi_stm32f4discovery)

add_executable(test-kalman-eigen-benchmark src/tests/kalman/test-kalman-eigen-benchmark.cpp)
sbs_target(test-kalman-eigen-benchmark stm32f429zi_stm32f4discovery)

add_executable(test-logger src/tests/logger/test-logger.cpp)
sbs_target(test-logger stm32f429zi_stm32f4discovery)

#src/tests/old_examples/misc/xbee-bitrate.cpp
#src/tests/old_examples/misc/xbee-send-rcv.cpp
#src/tests/old_examples/misc/xbee-time-to-send.cpp

add_executable(test-eventinjector src/tests/test-eventinjector.cpp)
sbs_target(test-eventinjector stm32f429zi_stm32f4discovery)

add_executable(test-hsm src/tests/test-hsm.cpp)
sbs_target(test-hsm stm32f429zi_stm32f4discovery)

add_executable(test-interrupt src/tests/test-interrupt.cpp)
sbs_target(test-interrupt stm32f429zi_stm32f4discovery)

add_executable(test-pinobserver src/tests/test-pinobserver.cpp)
sbs_target(test-pinobserver stm32f429zi_stm32f4discovery)

add_executable(test-sensormanager src/tests/test-sensormanager.cpp)
sbs_target(test-sensormanager stm32f429zi_skyward_death_stack_x)

add_executable(test-serial src/tests/test-serial.cpp)
sbs_target(test-serial stm32f407vg_stm32f4discovery)

add_executable(test-taskscheduler src/tests/test-taskscheduler.cpp)
sbs_target(test-taskscheduler stm32f429zi_skyward_death_stack_x)

add_executable(test-timestamp-timer src/tests/test-timestamp-timer.cpp)
sbs_target(test-timestamp-timer stm32f429zi_stm32f4discovery)

add_executable(test-trace-logger src/tests/test-trace-logger.cpp)
sbs_target(test-trace-logger stm32f429zi_stm32f4discovery)

add_executable(test-max6675 src/tests/sensors/MAX6675/test-max6675.cpp)
sbs_target(test-max6675 stm32f407vg_stm32f4discovery)
