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

cmake_minimum_required(VERSION 3.25)
enable_testing()

#-----------------------------------------------------------------------------#
#                                   Project                                   #
#-----------------------------------------------------------------------------#

project(Boardcore)
include(cmake/sbs.cmake)

#-----------------------------------------------------------------------------#
#                                 Entrypoints                                 #
#-----------------------------------------------------------------------------#

sbs_declare_target(bmx160-calibration-v2 stm32f429zi_death_stack_v2 src/entrypoints/bmx160-calibration-entry.cpp)
sbs_declare_target(bmx160-calibration-v3 stm32f429zi_death_stack_v3 src/entrypoints/bmx160-calibration-entry.cpp)
sbs_declare_target(config-dsgamma stm32f429zi_stm32f4discovery src/entrypoints/config-dsgamma.cpp)
sbs_declare_target(imu-calibration stm32f429zi_parafoil src/entrypoints/imu-calibration.cpp)
sbs_declare_target(compute-unit-v2-testsuite stm32f767zi_skyward_compute_unit src/entrypoints/compute-unit-v2-testsuite.cpp)
sbs_declare_target(runcam-settings stm32f407vg_stm32f4discovery src/entrypoints/runcam-settings.cpp)
sbs_declare_target(sdcard-benchmark stm32f429zi_death_stack_v2 src/entrypoints/sdcard-benchmark.cpp)
sbs_declare_target(sx1278fsk-ra01-serial stm32f429zi_nokia src/entrypoints/sx1278-serial.cpp DEFS SX1278_IS_FSK)
sbs_declare_target(sx1278lora-ra01-serial stm32f429zi_nokia src/entrypoints/sx1278-serial.cpp DEFS SX1278_IS_LORA)
sbs_declare_target(sx1278fsk-skyward433-serial stm32f767zi_gemini_gs src/entrypoints/sx1278-serial.cpp DEFS SX1278_IS_FSK SX1278_IS_SKYWARD433)
sbs_declare_target(sx1278lora-skyward433-serial stm32f429zi_nokia src/entrypoints/sx1278-serial.cpp DEFS SX1278_IS_LORA SX1278_IS_SKYWARD433)
sbs_declare_target(sx1278fsk-ebyterig-serial stm32f429zi_rig src/entrypoints/sx1278-serial.cpp DEFS SX1278_IS_FSK)
sbs_declare_target(system-show stm32f767zi_skyward_compute_unit src/entrypoints/system-show.cpp)

#-----------------------------------------------------------------------------#
#                                  Tests                                      #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-logger stm32f429zi_stm32f4discovery src/tests/logger/test-logger.cpp)
sbs_declare_target(test-eventinjector stm32f429zi_stm32f4discovery src/tests/test-eventinjector.cpp)
sbs_declare_target(test-hsm stm32f429zi_stm32f4discovery src/tests/test-hsm.cpp)
sbs_declare_target(test-interrupt stm32f429zi_stm32f4discovery src/tests/test-interrupt.cpp)
sbs_declare_target(test-max485 stm32f407vg_stm32f4discovery src/tests/test-max485.cpp)
sbs_declare_target(test-pinobserver stm32f429zi_stm32f4discovery src/tests/test-pinobserver.cpp)
# Disabled: source removed from the updated boardcore tree
# sbs_declare_target(test-externalpinobserver stm32f767zi_hydra_biscotto src/tests/test-externalpinobserver.cpp)
sbs_declare_target(test-rtc stm32f767zi_skyward_compute_unit src/tests/test-rtc.cpp)
sbs_declare_target(test-sensormanager stm32f429zi_death_stack_v2 src/tests/test-sensormanager.cpp)
sbs_declare_target(test-serial stm32f767zi_lyra_biscotto src/tests/test-serial.cpp)
sbs_declare_target(test-taskscheduler stm32f407vg_stm32f4discovery src/tests/scheduler/test-taskscheduler.cpp)
sbs_declare_target(test-trace-logger stm32f429zi_stm32f4discovery src/tests/test-trace-logger.cpp)
sbs_declare_target(test-PCA9685 stm32f767zi_lyra_biscotto src/tests/drivers/test-PCA9685.cpp)
sbs_declare_target(test-hil stm32f767zi_death_stack_v4
    src/tests/hil/test-hil.cpp
    src/tests/hil/Sensors/Sensors.cpp
)
sbs_declare_target(test-mcp23s17 stm32f767zi_rig_v2 src/tests/drivers/test-mcp23s17.cpp)

#-----------------------------------------------------------------------------#
#                              Tests - Catch                                  #
#-----------------------------------------------------------------------------#

sbs_declare_target(catch-tests-boardcore host
    src/tests/catch/catch-tests-entry.cpp
    src/tests/catch/skyQuaternion/test-skyquaternion.cpp
    src/tests/catch/examples/example-test-factorial.cpp
    src/tests/catch/test-aero.cpp
    src/tests/catch/test-buttonhandler.cpp
    src/tests/catch/test-circularbuffer.cpp
    src/tests/catch/test-eventbroker.cpp
    src/tests/catch/test-gptimer.cpp
    src/tests/catch/test-kalman.cpp
    src/tests/catch/test-numeric.cpp
    src/tests/catch/test-packetqueue.cpp
    src/tests/catch/test-sensormanager-catch.cpp
    src/tests/catch/xbee/test-xbee-parser.cpp
    src/tests/catch/test-modulemanager.cpp
    src/tests/catch/test-dependencymanager.cpp
    src/tests/catch/test-MEA.cpp
    src/tests/catch/test-airbrakesInterp.cpp
    src/tests/catch/test-airbrakesInterpPID.cpp
    src/tests/catch/test-pitot.cpp
    src/tests/catch/test-units.cpp
    src/tests/catch/test-registry-frontend.cpp
    src/tests/catch/propagator/test-propagator.cpp DEFS USE_MOCK_PERIPHERALS HOST)

#-----------------------------------------------------------------------------#
#                             Tests - Actuators                               #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-hbridge stm32f429zi_stm32f4discovery src/tests/actuators/test-hbridge.cpp)
sbs_declare_target(test-servo stm32f429zi_stm32f4discovery src/tests/actuators/test-servo.cpp)
# Disabled: source removed from the updated boardcore tree
# sbs_declare_target(test-servo-winch stm32f767zi_lyra_biscotto src/tests/actuators/test-servo-winch.cpp)

#-----------------------------------------------------------------------------#
#                             Tests - Algorithms                              #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-stepper stm32f767zi_skyward_compute_unit src/tests/actuators/test-stepper.cpp)
sbs_declare_target(test-stepper-pwm stm32f767zi_skyward_compute_unit src/tests/actuators/test-stepper-pwm.cpp)

#-----------------------------------------------------------------------------#
#                               Tests - Boards                                #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-kalman-benchmark stm32f429zi_stm32f4discovery src/tests/algorithms/Kalman/test-kalman-benchmark.cpp)
sbs_declare_target(test-attitude-parafoil stm32f429zi_parafoil src/tests/algorithms/NAS/test-attitude-parafoil.cpp)
sbs_declare_target(test-attitude-stack stm32f429zi_death_stack_v2 src/tests/algorithms/NAS/test-attitude-stack.cpp)
sbs_declare_target(test-nas-parafoil stm32f429zi_parafoil src/tests/algorithms/NAS/test-nas-parafoil.cpp)
sbs_declare_target(test-nas stm32f429zi_death_stack_v3 src/tests/algorithms/NAS/test-nas.cpp)
sbs_declare_target(test-nas-with-triad stm32f429zi_death_stack_v2 src/tests/algorithms/NAS/test-nas-with-triad.cpp)
sbs_declare_target(test-triad stm32f429zi_death_stack_v3 src/tests/algorithms/NAS/test-triad.cpp)
sbs_declare_target(test-triad-parafoil stm32f429zi_parafoil src/tests/algorithms/NAS/test-triad-parafoil.cpp)
sbs_declare_target(test-ada stm32f429zi_death_stack_v3 src/tests/algorithms/ADA/test-ada.cpp)
sbs_declare_target(test-compute-unit stm32f767zi_skyward_compute_unit src/tests/boards/test-compute-unit.cpp)
sbs_declare_target(test-qspi-flash stm32f767zi_skyward_compute_unit src/tests/boards/test-qspi-flash.cpp)

#-----------------------------------------------------------------------------#
#                               Tests - Drivers                               #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-ad5204 stm32f205rc_ciuti src/tests/drivers/test-ad5204.cpp)
sbs_declare_target(test-can-2way stm32f429zi_pyxis_auxiliary src/tests/drivers/canbus/CanDriver/test-can-2way.cpp)
sbs_declare_target(test-can-filters stm32f429zi_pyxis_auxiliary src/tests/drivers/canbus/CanDriver/test-can-filters.cpp)
sbs_declare_target(test-can-loopback stm32f429zi_death_stack_v2 src/tests/drivers/canbus/CanDriver/test-can-loopback.cpp)
sbs_declare_target(test-can-protocol stm32f429zi_death_stack_v2 src/tests/drivers/canbus/CanProtocol/test-can-protocol.cpp)
sbs_declare_target(test-dsgamma stm32f429zi_stm32f4discovery src/tests/drivers/test-dsgamma.cpp)
sbs_declare_target(test-general-purpose-timer stm32f429zi_stm32f4discovery src/tests/drivers/timer/test-general-purpose-timer.cpp)
sbs_declare_target(test-internal-adc stm32f767zi_skyward_compute_unit src/tests/drivers/test-internal-adc.cpp)
sbs_declare_target(test-mavlink stm32f407vg_stm32f4discovery src/tests/drivers/test-mavlink.cpp)
sbs_declare_target(test-MBLoadCell stm32f407vg_stm32f4discovery src/tests/drivers/test-MBLoadCell.cpp)
sbs_declare_target(test-pwm stm32f429zi_stm32f4discovery src/tests/drivers/timer/test-pwm.cpp)
sbs_declare_target(test-counted-pwm stm32f429zi_stm32f4discovery src/tests/drivers/timer/test-counted-pwm.cpp)
sbs_declare_target(test-spi stm32f407vg_stm32f4discovery src/tests/drivers/spi/test-spi.cpp)
sbs_declare_target(test-timer-utils stm32f429zi_stm32f4discovery src/tests/drivers/timer/test-timer-utils.cpp)
sbs_declare_target(test-timestamptimer stm32f429zi_stm32f4discovery src/tests/drivers/timer/test-timestamptimer.cpp)
sbs_declare_target(test-xbee-bidir stm32f429zi_death_stack_v2 src/tests/drivers/xbee/test-xbee-bidir.cpp)
# Disabled: requires mxgui (not ported to the Miosix 3 model yet)
# sbs_declare_target(test-xbee-gui stm32f429zi_stm32f4discovery
#     src/tests/drivers/xbee/test-xbee-gui.cpp
#     src/tests/drivers/xbee/gui/res/respect.cpp
# )
sbs_declare_target(test-xbee-rcv stm32f429zi_stm32f4discovery src/tests/drivers/xbee/test-xbee-rcv.cpp)
sbs_declare_target(test-xbee-snd stm32f429zi_stm32f4discovery src/tests/drivers/xbee/test-xbee-snd.cpp)
sbs_declare_target(test-usart-f4 stm32f429zi_stm32f4discovery src/tests/drivers/usart/test-usart.cpp)
sbs_declare_target(test-usart-f7 stm32f767zi_skyward_compute_unit src/tests/drivers/usart/test-usart.cpp)
sbs_declare_target(test-dma-mem-to-mem stm32f767zi_skyward_compute_unit src/tests/drivers/test-dma-mem-to-mem.cpp)
sbs_declare_target(test-i2c-driver-f4 stm32f429zi_stm32f4discovery src/tests/drivers/i2c/test-i2c-driver.cpp)
sbs_declare_target(test-i2c-f4 stm32f429zi_stm32f4discovery src/tests/drivers/i2c/test-i2c.cpp)
sbs_declare_target(test-i2c-driver-f7 stm32f767zi_skyward_compute_unit src/tests/drivers/i2c/test-i2c-driver.cpp)
sbs_declare_target(test-i2c-f7 stm32f767zi_skyward_compute_unit src/tests/drivers/i2c/test-i2c.cpp)
sbs_declare_target(test-wiz5500 stm32f767zi_gemini_gs src/tests/drivers/test-wiz5500.cpp)
sbs_declare_target(test-bsram stm32f767zi_lyra_biscotto src/tests/drivers/test-bsram.cpp)
sbs_declare_target(test-dipswitch stm32f767zi_lyra_gs src/tests/drivers/test-dipswitch.cpp)

#-----------------------------------------------------------------------------#
#                               Tests - Events                                #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-fsm stm32f429zi_stm32f4discovery src/tests/events/fsm/test-fsm.cpp)

#-----------------------------------------------------------------------------#
#                               Tests - Radio                                 #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-sx1278fsk-bidir stm32f429zi_nokia src/tests/radio/sx1278/fsk/test-sx1278-bidir.cpp)
sbs_declare_target(test-sx1278fsk-tx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-serial.cpp DEFS DISABLE_RX)
sbs_declare_target(test-sx1278fsk-rx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-serial.cpp DEFS DISABLE_TX)
# Disabled: requires mxgui (not ported to the Miosix 3 model yet)
# sbs_declare_target(test-sx1278fsk-gui-tx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-gui.cpp DEFS DISABLE_RX)
# Disabled: requires mxgui (not ported to the Miosix 3 model yet)
# sbs_declare_target(test-sx1278fsk-gui-rx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-gui.cpp DEFS DISABLE_TX)
sbs_declare_target(test-sx1278fsk-mavlink stm32f429zi_nokia src/tests/radio/sx1278/fsk/test-sx1278-mavlink.cpp)
sbs_declare_target(test-sx1278lora-bidir stm32f429zi_nokia src/tests/radio/sx1278/lora/test-sx1278-bidir.cpp)
sbs_declare_target(test-sx1278lora-mavlink stm32f429zi_nokia src/tests/radio/sx1278/lora/test-sx1278-mavlink.cpp)
sbs_declare_target(test-sx1278lora-simple-rx stm32f429zi_nokia src/tests/radio/sx1278/lora/test-sx1278-simple.cpp DEFS ENABLE_RX)
sbs_declare_target(test-sx1278lora-simple-tx stm32f429zi_nokia src/tests/radio/sx1278/lora/test-sx1278-simple.cpp DEFS ENABLE_TX)
sbs_declare_target(test-sx1278lora-tx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-serial.cpp DEFS DISABLE_RX SX1278_IS_LORA)
sbs_declare_target(test-sx1278lora-rx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-serial.cpp DEFS DISABLE_TX SX1278_IS_LORA)
# Disabled: requires mxgui (not ported to the Miosix 3 model yet)
# sbs_declare_target(test-sx1278lora-gui-rx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-gui.cpp DEFS DISABLE_TX SX1278_IS_LORA)
# Disabled: requires mxgui (not ported to the Miosix 3 model yet)
# sbs_declare_target(test-sx1278lora-gui-tx stm32f429zi_nokia src/tests/radio/sx1278/test-sx1278-bench-gui.cpp DEFS DISABLE_RX SX1278_IS_LORA)

#-----------------------------------------------------------------------------#
#                               Tests - Sensors                               #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-ads1118 stm32f407vg_stm32f4discovery src/tests/sensors/test-ads1118.cpp)
sbs_declare_target(test-ads131m04 stm32f429zi_death_stack_v3 src/tests/sensors/test-ads131m04.cpp)
sbs_declare_target(test-ads131m08 stm32f767zi_skyward_compute_unit src/tests/sensors/test-ads131m08.cpp)
sbs_declare_target(test-analog-pressure-sensors stm32f429zi_stm32f4discovery src/tests/sensors/analog/test-analog-pressure-sensors.cpp)
# Disabled: source removed from the updated boardcore tree
# sbs_declare_target(test-as5047d stm32f767zi_lyra_biscotto src/tests/sensors/test-as5047d.cpp)
sbs_declare_target(test-battery-voltage stm32f429zi_stm32f4discovery src/tests/sensors/analog/test-battery-voltage.cpp)
sbs_declare_target(test-current-sensor stm32f429zi_stm32f4discovery src/tests/sensors/analog/test-current-sensor.cpp)
sbs_declare_target(test-calibration-benchmark stm32f407vg_stm32f4discovery src/tests/sensors/calibration/test-calibration-benchmark.cpp)
sbs_declare_target(test-calibration-stats stm32f407vg_stm32f4discovery src/tests/sensors/calibration/test-calibration-stats.cpp)
sbs_declare_target(test-bme280-spi stm32f429zi_stm32f4discovery src/tests/sensors/test-bme280-spi.cpp)
sbs_declare_target(test-bme280-i2c stm32f429zi_stm32f4discovery src/tests/sensors/test-bme280-i2c.cpp)
sbs_declare_target(test-bmp280-spi stm32f429zi_stm32f4discovery src/tests/sensors/test-bmp280-spi.cpp)
sbs_declare_target(test-bmp280-i2c stm32f429zi_stm32f4discovery src/tests/sensors/test-bmp280-i2c.cpp)
sbs_declare_target(test-bmx160 stm32f429zi_death_stack_v2 src/tests/sensors/test-bmx160.cpp)
sbs_declare_target(test-bmx160-with-correction stm32f429zi_death_stack_v2 src/tests/sensors/test-bmx160-with-correction.cpp)
sbs_declare_target(test-hx711 stm32f429zi_stm32f4discovery src/tests/sensors/test-hx711.cpp)
sbs_declare_target(test-l3gd20 stm32f429zi_stm32f4discovery src/tests/sensors/test-l3gd20.cpp)
sbs_declare_target(test-l3gd20-fifo stm32f429zi_stm32f4discovery src/tests/sensors/test-l3gd20-fifo.cpp)
sbs_declare_target(test-lis3dsh stm32f407vg_stm32f4discovery src/tests/sensors/test-lis3dsh.cpp)
sbs_declare_target(test-lis3mdl stm32f429zi_death_stack_v2 src/tests/sensors/test-lis3mdl.cpp)
sbs_declare_target(test-lis331hh stm32f205rc_ciuti src/tests/sensors/test-lis331hh.cpp)
sbs_declare_target(test-lps331ap stm32f429zi_stm32f4discovery src/tests/sensors/test-lps331ap.cpp)
sbs_declare_target(test-max6675 stm32f429zi_stm32f4discovery src/tests/sensors/test-max6675.cpp)
sbs_declare_target(test-max31855 stm32f429zi_stm32f4discovery src/tests/sensors/test-max31855.cpp)
sbs_declare_target(test-max31856 stm32f767zi_skyward_compute_unit src/tests/sensors/test-max31856.cpp)
sbs_declare_target(test-mpu9250 stm32f429zi_parafoil src/tests/sensors/test-mpu9250.cpp)
sbs_declare_target(test-ms5803-spi stm32f429zi_death_stack_v2 src/tests/sensors/test-ms5803-spi.cpp)
sbs_declare_target(test-ms5803-i2c stm32f429zi_stm32f4discovery src/tests/sensors/test-ms5803-i2c.cpp)
sbs_declare_target(test-ubxgps-serial stm32f429zi_death_stack_v2 src/tests/sensors/test-ubxgps-serial.cpp)
sbs_declare_target(test-ubxgps-spi stm32f429zi_death_stack_v2 src/tests/sensors/test-ubxgps-spi.cpp)
sbs_declare_target(test-vn100-serial stm32f407vg_stm32f4discovery src/tests/sensors/test-vn100-serial.cpp)
sbs_declare_target(test-vn100-spi stm32f407vg_stm32f4discovery src/tests/sensors/test-vn100-spi.cpp)
sbs_declare_target(test-vn300 stm32f767zi_skyward_compute_unit src/tests/sensors/test-vn300.cpp)
sbs_declare_target(test-lis2mdl stm32f429zi_stm32f4discovery src/tests/sensors/test-lis2mdl.cpp)
sbs_declare_target(test-h3lis331dl stm32f407vg_stm32f4discovery src/tests/sensors/test-h3lis331dl.cpp)
sbs_declare_target(test-lps28dfw stm32f767zi_skyward_compute_unit src/tests/sensors/test-lps28dfw.cpp)
sbs_declare_target(test-lps22df stm32f767zi_skyward_compute_unit src/tests/sensors/test-lps22df.cpp)
sbs_declare_target(test-lsm6dsrx stm32f407vg_stm32f4discovery src/tests/sensors/test-lsm6dsrx.cpp)
sbs_declare_target(test-nd015x stm32f767zi_lyra_biscotto src/tests/sensors/test-nd015x.cpp)
sbs_declare_target(test-nd030d stm32f767zi_lyra_biscotto src/tests/sensors/test-nd030d.cpp)

#-----------------------------------------------------------------------------#
#                                Tests - Utils                                #
#-----------------------------------------------------------------------------#

sbs_declare_target(test-csvreader stm32f429zi_stm32f4discovery src/tests/utils/test-csvreader.cpp)
sbs_declare_target(test-buttonhandler stm32f429zi_stm32f4discovery src/tests/utils/test-buttonhandler.cpp)
sbs_declare_target(test-syncpacketqueue stm32f407vg_stm32f4discovery src/tests/utils/test-syncpacketqueue.cpp)
