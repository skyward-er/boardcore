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

add_executable(bmx160-calibration-v2 src/entrypoints/bmx160-calibration-entry.cpp)
sbs_target(bmx160-calibration-v2 stm32f429zi_skyward_death_stack_x)

add_executable(bmx160-calibration-v3 src/entrypoints/bmx160-calibration-entry.cpp)
sbs_target(bmx160-calibration-v3 stm32f429zi_skyward_death_stack_v3)

add_executable(config-dsgamma src/entrypoints/config-dsgamma.cpp)
sbs_target(config-dsgamma stm32f429zi_stm32f4discovery)

add_executable(imu-calibration src/entrypoints/imu-calibration.cpp)
sbs_target(imu-calibration stm32f429zi_skyward_parafoil)

add_executable(mxgui-helloworld src/entrypoints/examples/mxgui-helloworld.cpp)
sbs_target(mxgui-helloworld stm32f429zi_stm32f4discovery)

add_executable(kernel-testsuite src/entrypoints/kernel-testsuite.cpp)
sbs_target(kernel-testsuite stm32f767zi_compute_unit)

add_executable(runcam-settings src/entrypoints/runcam-settings.cpp)
sbs_target(runcam-settings stm32f407vg_stm32f4discovery)

add_executable(sdcard-benchmark src/entrypoints/sdcard-benchmark.cpp)
sbs_target(sdcard-benchmark stm32f429zi_skyward_death_stack_x)

add_executable(sx1278fsk-serial src/entrypoints/sx1278-serial.cpp)
target_compile_definitions(sx1278fsk-serial PRIVATE SX1278_IS_FSK)
sbs_target(sx1278fsk-serial stm32f429zi_skyward_groundstation_v2)

add_executable(sx1278lora-serial src/entrypoints/sx1278-serial.cpp)
target_compile_definitions(sx1278lora-serial PRIVATE SX1278_IS_LORA)
sbs_target(sx1278lora-serial stm32f429zi_skyward_groundstation_v2)

#-----------------------------------------------------------------------------#
#                                    Tests                                    #
#-----------------------------------------------------------------------------#

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

add_executable(test-rtc src/tests/test-rtc.cpp)
sbs_target(test-rtc stm32f767zi_compute_unit)

add_executable(test-sensormanager src/tests/test-sensormanager.cpp)
sbs_target(test-sensormanager stm32f429zi_skyward_death_stack_x)

add_executable(test-serial src/tests/test-serial.cpp)
sbs_target(test-serial stm32f756zg_nucleo)

add_executable(test-taskscheduler src/tests/scheduler/test-taskscheduler.cpp)
sbs_target(test-taskscheduler stm32f407vg_stm32f4discovery)

add_executable(test-trace-logger src/tests/test-trace-logger.cpp)
sbs_target(test-trace-logger stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                Tests - Catch                                #
#-----------------------------------------------------------------------------#

add_executable(catch-tests-boardcore
    src/tests/catch/catch-tests-entry.cpp
    src/tests/catch/examples/example-test-factorial.cpp
    src/tests/catch/test-aero.cpp
    # src/tests/catch/test-buttonhandler.cpp
    src/tests/catch/test-circularbuffer.cpp
    src/tests/catch/test-eventbroker.cpp
    src/tests/catch/test-timestamptimer.cpp
    src/tests/catch/test-kalman.cpp
    src/tests/catch/test-packetqueue.cpp
    src/tests/catch/test-sensormanager-catch.cpp
    src/tests/catch/xbee/test-xbee-parser.cpp
    src/tests/catch/test-modulemanager.cpp
)
target_compile_definitions(catch-tests-boardcore PRIVATE USE_MOCK_PERIPHERALS)
sbs_target(catch-tests-boardcore stm32f429zi_stm32f4discovery)
sbs_catch_test(catch-tests-boardcore)

#-----------------------------------------------------------------------------#
#                             Tests - Actuators                               #
#-----------------------------------------------------------------------------#

add_executable(test-hbridge src/tests/actuators/test-hbridge.cpp)
sbs_target(test-hbridge stm32f429zi_stm32f4discovery)

add_executable(test-servo src/tests/actuators/test-servo.cpp)
sbs_target(test-servo stm32f429zi_stm32f4discovery)

add_executable(test-buzzer src/tests/actuators/test-buzzer.cpp)
sbs_target(test-buzzer stm32f429zi_hre_test_stand)

add_executable(test-stepper src/tests/actuators/test-stepper.cpp)
sbs_target(test-stepper stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                             Tests - Algorithms                              #
#-----------------------------------------------------------------------------#

add_executable(test-kalman-benchmark src/tests/algorithms/Kalman/test-kalman-benchmark.cpp)
sbs_target(test-kalman-benchmark stm32f429zi_stm32f4discovery)

add_executable(test-attitude-parafoil src/tests/algorithms/NAS/test-attitude-parafoil.cpp)
sbs_target(test-attitude-parafoil stm32f429zi_skyward_parafoil)

add_executable(test-attitude-stack src/tests/algorithms/NAS/test-attitude-stack.cpp)
sbs_target(test-attitude-stack stm32f429zi_skyward_death_stack_x)

add_executable(test-nas-parafoil src/tests/algorithms/NAS/test-nas-parafoil.cpp)
sbs_target(test-nas-parafoil stm32f429zi_skyward_parafoil)

add_executable(test-nas src/tests/algorithms/NAS/test-nas.cpp)
sbs_target(test-nas stm32f429zi_skyward_death_stack_v3)

add_executable(test-nas-with-triad src/tests/algorithms/NAS/test-nas-with-triad.cpp)
sbs_target(test-nas-with-triad stm32f429zi_skyward_death_stack_x)

add_executable(test-triad src/tests/algorithms/NAS/test-triad.cpp)
sbs_target(test-triad stm32f429zi_skyward_death_stack_v3)

add_executable(test-triad-parafoil src/tests/algorithms/NAS/test-triad-parafoil.cpp)
sbs_target(test-triad-parafoil stm32f429zi_skyward_parafoil)

add_executable(test-ada src/tests/algorithms/ADA/test-ada.cpp)
sbs_target(test-ada stm32f429zi_skyward_death_stack_v3)

#-----------------------------------------------------------------------------#
#                               Tests - Boards                                #
#-----------------------------------------------------------------------------#

add_executable(test-compute-unit src/tests/boards/test-compute-unit.cpp)
sbs_target(test-compute-unit stm32f767zi_compute_unit)

add_executable(test-qspi-flash src/tests/boards/test-qspi-flash.cpp)
sbs_target(test-qspi-flash stm32f767zi_compute_unit)

#-----------------------------------------------------------------------------#
#                               Tests - Drivers                               #
#-----------------------------------------------------------------------------#

add_executable(test-ad5204 src/tests/drivers/test-ad5204.cpp)
sbs_target(test-ad5204 stm32f205rc_skyward_ciuti)

add_executable(test-can-2way src/tests/drivers/canbus/CanDriver/test-can-2way.cpp)
sbs_target(test-can-2way stm32f429zi_skyward_pyxis_auxiliary)

add_executable(test-can-filters src/tests/drivers/canbus/CanDriver/test-can-filters.cpp)
sbs_target(test-can-filters stm32f429zi_skyward_pyxis_auxiliary)

add_executable(test-can-loopback src/tests/drivers/canbus/CanDriver/test-can-loopback.cpp)
sbs_target(test-can-loopback stm32f429zi_skyward_death_stack_x)

add_executable(test-can-protocol src/tests/drivers/canbus/CanProtocol/test-can-protocol.cpp)
sbs_target(test-can-protocol stm32f429zi_skyward_death_stack_x)

add_executable(test-dsgamma src/tests/drivers/test-dsgamma.cpp)
sbs_target(test-dsgamma stm32f429zi_stm32f4discovery)

add_executable(test-general-purpose-timer src/tests/drivers/timer/test-general-purpose-timer.cpp)
sbs_target(test-general-purpose-timer stm32f429zi_stm32f4discovery)

add_executable(test-internal-adc src/tests/drivers/test-internal-adc.cpp)
sbs_target(test-internal-adc stm32f767zi_compute_unit)

add_executable(test-mavlink src/tests/drivers/test-mavlink.cpp)
sbs_target(test-mavlink stm32f407vg_stm32f4discovery)

add_executable(test-MBLoadCell src/tests/drivers/test-MBLoadCell.cpp)
sbs_target(test-MBLoadCell stm32f407vg_stm32f4discovery)

add_executable(test-pwm src/tests/drivers/timer/test-pwm.cpp)
sbs_target(test-pwm stm32f429zi_stm32f4discovery)

add_executable(test-spi src/tests/drivers/spi/test-spi.cpp)
sbs_target(test-spi stm32f407vg_stm32f4discovery)

add_executable(test-timer-utils src/tests/drivers/timer/test-timer-utils.cpp)
sbs_target(test-timer-utils stm32f429zi_stm32f4discovery)

add_executable(test-timestamptimer src/tests/drivers/timer/test-timestamptimer.cpp)
sbs_target(test-timestamptimer stm32f429zi_stm32f4discovery)

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

add_executable(test-usart src/tests/drivers/usart/test-usart.cpp)
sbs_target(test-usart stm32f407vg_stm32f4discovery)

add_executable(test-i2c-driver-f4 src/tests/drivers/i2c/test-i2c-driver.cpp)
sbs_target(test-i2c-driver-f4 stm32f429zi_stm32f4discovery)

add_executable(test-i2c-f4 src/tests/drivers/i2c/test-i2c.cpp)
sbs_target(test-i2c-f4 stm32f429zi_stm32f4discovery)

add_executable(test-i2c-driver-f7 src/tests/drivers/i2c/test-i2c-driver.cpp)
sbs_target(test-i2c-driver-f7 stm32f767zi_nucleo)

add_executable(test-i2c-f7 src/tests/drivers/i2c/test-i2c.cpp)
sbs_target(test-i2c-f7 stm32f767zi_nucleo)

#-----------------------------------------------------------------------------#
#                               Tests - Events                                #
#-----------------------------------------------------------------------------#

add_executable(test-fsm src/tests/events/fsm/test-fsm.cpp)
sbs_target(test-fsm stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                               Tests - Radio                                 #
#-----------------------------------------------------------------------------#

add_executable(test-sx1278fsk-bidir src/tests/radio/sx1278/fsk/test-sx1278-bidir.cpp)
sbs_target(test-sx1278fsk-bidir stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278fsk-tx src/tests/radio/sx1278/fsk/test-sx1278-bench-serial.cpp)
target_compile_definitions(test-sx1278fsk-tx PRIVATE DISABLE_RX)
sbs_target(test-sx1278fsk-tx stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278fsk-rx src/tests/radio/sx1278/fsk/test-sx1278-bench-serial.cpp)
target_compile_definitions(test-sx1278fsk-rx PRIVATE DISABLE_TX)
sbs_target(test-sx1278fsk-rx stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278fsk-gui src/tests/radio/sx1278/fsk/test-sx1278-bench-gui.cpp)
sbs_target(test-sx1278fsk-gui stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278fsk-gui-rx src/tests/radio/sx1278/fsk/test-sx1278-bench-gui.cpp)
target_compile_definitions(test-sx1278fsk-gui-rx PRIVATE DISABLE_TX)
sbs_target(test-sx1278fsk-gui-rx stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278fsk-gui-tx src/tests/radio/sx1278/fsk/test-sx1278-bench-gui.cpp)
target_compile_definitions(test-sx1278fsk-gui-tx PRIVATE DISABLE_RX)
sbs_target(test-sx1278fsk-gui-tx stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278fsk-mavlink src/tests/radio/sx1278/fsk/test-sx1278-mavlink.cpp)
sbs_target(test-sx1278fsk-mavlink stm32f429zi_skyward_groundstation_v2)

# add_executable(test-mavlinkdriver src/tests/radio/test-mavlinkdriver.cpp)
# sbs_target(test-mavlinkdriver stm32f407vg_stm32f4discovery)

add_executable(test-sx1278lora-bidir src/tests/radio/sx1278/lora/test-sx1278-bidir.cpp)
sbs_target(test-sx1278lora-bidir stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278lora-mavlink src/tests/radio/sx1278/lora/test-sx1278-mavlink.cpp)
sbs_target(test-sx1278lora-mavlink stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278lora-rx src/tests/radio/sx1278/lora/test-sx1278-simple.cpp)
target_compile_definitions(test-sx1278lora-rx PRIVATE ENABLE_RX)
sbs_target(test-sx1278lora-rx stm32f429zi_skyward_groundstation_v2)

add_executable(test-sx1278lora-tx src/tests/radio/sx1278/lora/test-sx1278-simple.cpp)
target_compile_definitions(test-sx1278lora-tx PRIVATE ENABLE_TX)
sbs_target(test-sx1278lora-tx stm32f429zi_skyward_groundstation_v2)

#-----------------------------------------------------------------------------#
#                               Tests - Sensors                               #
#-----------------------------------------------------------------------------#

add_executable(test-ads1118 src/tests/sensors/test-ads1118.cpp)
sbs_target(test-ads1118 stm32f407vg_stm32f4discovery)

add_executable(test-ads131m04 src/tests/sensors/test-ads131m04.cpp)
sbs_target(test-ads131m04 stm32f429zi_stm32f4discovery)

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

add_executable(test-bme280-spi src/tests/sensors/test-bme280-spi.cpp)
sbs_target(test-bme280-spi stm32f429zi_stm32f4discovery)

add_executable(test-bme280-i2c src/tests/sensors/test-bme280-i2c.cpp)
sbs_target(test-bme280-i2c stm32f429zi_stm32f4discovery)

add_executable(test-bmp280-spi src/tests/sensors/test-bmp280-spi.cpp)
sbs_target(test-bmp280-spi stm32f429zi_stm32f4discovery)

add_executable(test-bmp280-i2c src/tests/sensors/test-bmp280-i2c.cpp)
sbs_target(test-bmp280-i2c stm32f429zi_stm32f4discovery)

add_executable(test-bmx160 src/tests/sensors/test-bmx160.cpp)
sbs_target(test-bmx160 stm32f429zi_skyward_death_stack_x)

add_executable(test-bmx160-with-correction src/tests/sensors/test-bmx160-with-correction.cpp)
sbs_target(test-bmx160-with-correction stm32f429zi_skyward_death_stack_x)

add_executable(test-hx711 src/tests/sensors/test-hx711.cpp)
sbs_target(test-hx711 stm32f429zi_stm32f4discovery)

add_executable(test-l3gd20 src/tests/sensors/test-l3gd20.cpp)
sbs_target(test-l3gd20 stm32f429zi_stm32f4discovery)

add_executable(test-l3gd20-fifo src/tests/sensors/test-l3gd20-fifo.cpp)
sbs_target(test-l3gd20-fifo stm32f429zi_stm32f4discovery)

add_executable(test-lis3dsh src/tests/sensors/test-lis3dsh.cpp)
sbs_target(test-lis3dsh stm32f407vg_stm32f4discovery)

add_executable(test-lis3mdl src/tests/sensors/test-lis3mdl.cpp)
sbs_target(test-lis3mdl stm32f429zi_skyward_death_stack_x)

add_executable(test-lis331hh src/tests/sensors/test-lis331hh.cpp)
sbs_target(test-lis331hh stm32f205rc_skyward_ciuti)

add_executable(test-max6675 src/tests/sensors/test-max6675.cpp)
sbs_target(test-max6675 stm32f429zi_stm32f4discovery)

add_executable(test-max31855 src/tests/sensors/test-max31855.cpp)
sbs_target(test-max31855 stm32f429zi_stm32f4discovery)

add_executable(test-mpu9250 src/tests/sensors/test-mpu9250.cpp)
sbs_target(test-mpu9250 stm32f429zi_skyward_parafoil)

add_executable(test-ms5803-spi src/tests/sensors/test-ms5803-spi.cpp)
sbs_target(test-ms5803-spi stm32f429zi_skyward_death_stack_x)

add_executable(test-ms5803-i2c src/tests/sensors/test-ms5803-i2c.cpp)
sbs_target(test-ms5803-i2c stm32f429zi_stm32f4discovery)

add_executable(test-ubxgps-serial src/tests/sensors/test-ubxgps-serial.cpp)
sbs_target(test-ubxgps-serial stm32f429zi_skyward_death_stack_x)

add_executable(test-ubxgps-spi src/tests/sensors/test-ubxgps-spi.cpp)
sbs_target(test-ubxgps-spi stm32f429zi_skyward_death_stack_x)

add_executable(test-vn100 src/tests/sensors/test-vn100.cpp)
sbs_target(test-vn100 stm32f407vg_stm32f4discovery)

add_executable(test-lis2mdl src/tests/sensors/test-lis2mdl.cpp)
sbs_target(test-lis2mdl stm32f429zi_stm32f4discovery)


#-----------------------------------------------------------------------------#
#                                Tests - Utils                                #
#-----------------------------------------------------------------------------#

add_executable(test-csvreader src/tests/utils/test-csvreader.cpp)
sbs_target(test-csvreader stm32f429zi_stm32f4discovery)

add_executable(test-buttonhandler src/tests/utils/test-buttonhandler.cpp)
sbs_target(test-buttonhandler stm32f429zi_stm32f4discovery)

add_executable(test-syncpacketqueue src/tests/utils/test-syncpacketqueue.cpp)
sbs_target(test-syncpacketqueue stm32f407vg_stm32f4discovery)
