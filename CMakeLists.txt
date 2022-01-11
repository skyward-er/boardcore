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
enable_testing()
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

add_executable(runcam-settings src/entrypoints/runcam-settings.cpp)
sbs_target(runcam-settings stm32f407vg_stm32f4discovery)

add_executable(sdcard-benchmark src/entrypoints/sdcard-benchmark.cpp)
sbs_target(sdcard-benchmark stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                    Tests                                    #
#-----------------------------------------------------------------------------#

add_executable(test-kalman-eigen-benchmark src/tests/kalman/test-kalman-eigen-benchmark.cpp)
sbs_target(test-kalman-eigen-benchmark stm32f429zi_stm32f4discovery)

add_executable(test-logger src/tests/logger/test-logger.cpp)
sbs_target(test-logger stm32f429zi_stm32f4discovery)

add_executable(test-eventinjector src/tests/test-eventinjector.cpp)
sbs_target(test-eventinjector stm32f429zi_stm32f4discovery)

add_executable(test-hsm src/tests/test-hsm.cpp)
sbs_target(test-hsm stm32f429zi_stm32f4discovery)

add_executable(test-interrupt src/tests/test-interrupt.cpp)
sbs_target(test-interrupt stm32f429zi_stm32f4discovery)

add_executable(test-max485 src/tests/test-max485.cpp)
sbs_target(test-max485 stm32f407vg_stm32f4discovery)

add_executable(test-pinobserver src/tests/test-pinobserver.cpp)
sbs_target(test-pinobserver stm32f429zi_stm32f4discovery)

add_executable(test-sensormanager src/tests/test-sensormanager.cpp)
sbs_target(test-sensormanager stm32f429zi_skyward_death_stack_x)

add_executable(test-serial src/tests/test-serial.cpp)
sbs_target(test-serial stm32f429zi_skyward_death_stack_x)

add_executable(test-taskscheduler src/tests/test-taskscheduler.cpp)
sbs_target(test-taskscheduler stm32f429zi_skyward_death_stack_x)

add_executable(test-timestamp-timer src/tests/test-timestamp-timer.cpp)
sbs_target(test-timestamp-timer stm32f429zi_stm32f4discovery)

add_executable(test-trace-logger src/tests/test-trace-logger.cpp)
sbs_target(test-trace-logger stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                Tests - Catch                                #
#-----------------------------------------------------------------------------#

add_executable(catch-tests-entry
    src/tests/catch/catch-tests-entry.cpp
    src/tests/catch/examples/example-test-factorial.cpp
    src/tests/catch/test-aero.cpp
    src/tests/catch/test-buttonhandler.cpp
    src/tests/catch/test-circularbuffer.cpp
    src/tests/catch/test-eventbroker.cpp
    src/tests/catch/test-hardwaretimer.cpp
    src/tests/catch/test-kalman-eigen.cpp
    src/tests/catch/test-packetqueue.cpp
    src/tests/catch/test-sensormanager-catch.cpp
    src/tests/catch/xbee/test-xbee-parser.cpp
)
target_compile_definitions(catch-tests-entry PRIVATE USE_MOCK_PERIPHERALS)
sbs_target(catch-tests-entry stm32f429zi_stm32f4discovery)
sbs_catch_test(catch-tests-entry)

#-----------------------------------------------------------------------------#
#                               Tests - Drivers                               #
#-----------------------------------------------------------------------------#

add_executable(test-canbus-loopback src/tests/drivers/canbus/test-canbus-loopback.cpp)
sbs_target(test-canbus-loopback stm32f429zi_stm32f4discovery)

add_executable(test-canbus-2way src/tests/drivers/canbus/test-canbus-2way.cpp)
sbs_target(test-canbus-2way stm32f429zi_stm32f4discovery)

add_executable(test-dma-spi src/tests/drivers/dma/test-dma-spi.cpp)
sbs_target(test-dma-spi stm32f429zi_stm32f4discovery)

add_executable(test-dsgamma src/tests/drivers/test-dsgamma.cpp)
sbs_target(test-dsgamma stm32f429zi_stm32f4discovery)

add_executable(test-general-purpose-timer src/tests/drivers/timer/test-general-purpose-timer.cpp)
sbs_target(test-general-purpose-timer stm32f429zi_stm32f4discovery)

add_executable(test-hbridge src/tests/drivers/test-hbridge.cpp)
sbs_target(test-hbridge stm32f429zi_stm32f4discovery)

add_executable(test-internal-adc src/tests/drivers/test-internal-adc.cpp)
sbs_target(test-internal-adc stm32f429zi_skyward_death_stack_x)

add_executable(test-internal-adc-dma src/tests/drivers/test-internal-adc-dma.cpp)
sbs_target(test-internal-adc-dma stm32f429zi_stm32f4discovery)

add_executable(test-mavlink src/tests/drivers/test-mavlink.cpp)
sbs_target(test-mavlink stm32f429zi_skyward_homeone)

add_executable(test-MBLoadCell src/tests/drivers/test-MBLoadCell.cpp)
sbs_target(test-MBLoadCell stm32f407vg_stm32f4discovery)

add_executable(test-pwm src/tests/drivers/test-pwm.cpp)
sbs_target(test-pwm stm32f429zi_stm32f4discovery)

add_executable(test-servo src/tests/drivers/test-servo.cpp)
sbs_target(test-servo stm32f429zi_skyward_death_stack_x)

add_executable(test-spi src/tests/drivers/spi/test-spi.cpp)
sbs_target(test-spi stm32f429zi_stm32f4discovery)

add_executable(test-spi-as-slave src/tests/drivers/spi/test-spi-as-slave.cpp)
sbs_target(test-spi-as-slave stm32f429zi_stm32f4discovery)

add_executable(test-spi-signal-generator src/tests/drivers/spi/test-spi-signal-generator.cpp)
sbs_target(test-spi-signal-generator stm32f429zi_stm32f4discovery)

add_executable(test-spi-slave-bus src/tests/drivers/spi/test-spi-slave-bus.cpp)
sbs_target(test-spi-slave-bus stm32f429zi_stm32f4discovery)

add_executable(test-timer-utils src/tests/drivers/timer/test-timer-utils.cpp)
sbs_target(test-timer-utils stm32f429zi_stm32f4discovery)

add_executable(test-vn100serial src/tests/drivers/test-vn100serial.cpp)
sbs_target(test-vn100serial stm32f407vg_stm32f4discovery)

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

add_executable(test-sx1278 src/tests/drivers/sx1278/test-sx1278-bidir.cpp)
sbs_target(test-sx1278 stm32f407vg_stm32f4discovery)

add_executable(test-sx1278-tx src/tests/drivers/sx1278/test-sx1278-tx.cpp)
sbs_target(test-sx1278-tx stm32f407vg_stm32f4discovery)

add_executable(test-sx1278-rx src/tests/drivers/sx1278/test-sx1278-rx.cpp)
sbs_target(test-sx1278-rx stm32f407vg_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                               Tests - Sensors                               #
#-----------------------------------------------------------------------------#

add_executable(test-ads1118 src/tests/sensors/ADS1118/test-ads1118.cpp)
sbs_target(test-ads1118 stm32f407vg_stm32f4discovery)

add_executable(test-ads131m04 src/tests/sensors/ADS131M04/test-ads131m04.cpp)
sbs_target(test-ads131m04 stm32f429zi_stm32f4discovery)

add_executable(test-ads131m04highfreq src/tests/sensors/ADS131M04/test-ads131m04highfreq.cpp)
sbs_target(test-ads131m04highfreq stm32f429zi_stm32f4discovery)

add_executable(test-ads131m04highfreq-with-logger src/tests/sensors/ADS131M04/test-ads131m04highfreq-with-logger.cpp)
sbs_target(test-ads131m04highfreq-with-logger stm32f429zi_stm32f4discovery)

add_executable(test-analog-pressure-sensors src/tests/sensors/analog/test-analog-pressure-sensors.cpp)
sbs_target(test-analog-pressure-sensors stm32f429zi_stm32f4discovery)

add_executable(test-battery-voltage src/tests/sensors/analog/test-battery-voltage.cpp)
sbs_target(test-battery-voltage stm32f429zi_stm32f4discovery)

add_executable(test-current-sensor src/tests/sensors/analog/test-current-sensor.cpp)
sbs_target(test-current-sensor stm32f429zi_stm32f4discovery)

add_executable(test-calibration-benchmark src/tests/sensors/calibration/test-calibration-benchmark.cpp)
sbs_target(test-calibration-benchmark stm32f407vg_stm32f4discovery)

add_executable(test-calibration-stats src/tests/sensors/calibration/test-calibration-stats.cpp)
sbs_target(test-calibration-stats stm32f407vg_stm32f4discovery)

add_executable(test-calibration src/tests/sensors/calibration/test-calibration.cpp)
sbs_target(test-calibration stm32f407vg_stm32f4discovery)

add_executable(test-bme280 src/tests/sensors/test-bme280.cpp)
sbs_target(test-bme280 stm32f407vg_stm32f4discovery)

add_executable(test-bmx160 src/tests/sensors/test-bmx160.cpp)
sbs_target(test-bmx160 stm32f429zi_skyward_death_stack_x)

add_executable(test-bmx160-with-correction src/tests/sensors/test-bmx160-with-correction.cpp)
sbs_target(test-bmx160-with-correction stm32f429zi_skyward_death_stack_x)

add_executable(test-l3gd20 src/tests/sensors/test-l3gd20.cpp)
sbs_target(test-l3gd20 stm32f429zi_stm32f4discovery)

add_executable(test-l3gd20-fifo src/tests/sensors/test-l3gd20-fifo.cpp)
sbs_target(test-l3gd20-fifo stm32f429zi_stm32f4discovery)

add_executable(test-lis3dsh src/tests/sensors/test-lis3dsh.cpp)
sbs_target(test-lis3dsh stm32f407vg_stm32f4discovery)

add_executable(test-lis3mdl src/tests/sensors/test-lis3mdl.cpp)
sbs_target(test-lis3mdl stm32f429zi_skyward_death_stack_x)

add_executable(test-max6675 src/tests/sensors/test-max6675.cpp)
sbs_target(test-max6675 stm32f429zi_stm32f4discovery)

add_executable(test-mpu9250 src/tests/sensors/test-mpu9250.cpp)
sbs_target(test-mpu9250 stm32f429zi_skyward_death_stack_x)

add_executable(test-ms5803 src/tests/sensors/test-ms5803.cpp)
sbs_target(test-ms5803 stm32f429zi_skyward_death_stack_x)

add_executable(test-ubloxgps-serial src/tests/sensors/test-ubloxgps.cpp)
sbs_target(test-ubloxgps-serial stm32f407vg_stm32f4discovery)

add_executable(test-ubloxgps-spi src/tests/sensors/test-ubloxgps.cpp)
target_compile_definitions(test-ubloxgps-spi PRIVATE USE_SPI)
sbs_target(test-ubloxgps-spi stm32f407vg_stm32f4discovery)

add_executable(test-vn100 src/tests/sensors/test-vn100.cpp)
sbs_target(test-vn100 stm32f407vg_stm32f4discovery)
