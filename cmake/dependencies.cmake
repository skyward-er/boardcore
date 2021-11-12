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

add_subdirectory(${KPATH} EXCLUDE_FROM_ALL)

add_subdirectory(${SBS_BASE}/libs/mxgui EXCLUDE_FROM_ALL)

set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(BUILD_TESTING OFF CACHE BOOL "Enable creation of Eigen tests.")
set(EIGEN_TEST_NOQT ON CACHE BOOL "Disable Qt support in unit tests")
set(CMAKE_Fortran_COMPILER NOTFOUND)
add_subdirectory(${SBS_BASE}/libs/eigen EXCLUDE_FROM_ALL)
target_compile_definitions(eigen INTERFACE EIGEN_MAX_ALIGN_BYTES=0)

add_subdirectory(${SBS_BASE}/libs/fmt EXCLUDE_FROM_ALL)
target_compile_definitions(fmt-header-only INTERFACE _GLIBCXX_USE_WCHAR_T FMT_UNICODE=0 FMT_STATIC_THOUSANDS_SEPARATOR=0)
target_compile_options(fmt-header-only INTERFACE -fno-math-errno)

set(SHARED_SOURCES
    ${SBS_BASE}/src/shared/scheduler/TaskScheduler.cpp
    ${SBS_BASE}/src/shared/diagnostic/CpuMeter.cpp
    ${SBS_BASE}/src/shared/events/EventBroker.cpp
    ${SBS_BASE}/src/shared/math/Stats.cpp
    ${SBS_BASE}/src/shared/drivers/interrupt/external_interrupts.cpp
    ${SBS_BASE}/src/shared/utils/aero/AeroUtils.cpp
    ${SBS_BASE}/src/shared/Debug.cpp
    ${SBS_BASE}/src/shared/TimestampTimer.cpp
    ${SBS_BASE}/src/shared/diagnostic/PrintLogger.cpp
    ${SBS_BASE}/src/shared/logger/Logger.cpp
    ${SBS_BASE}/libs/tscpp/buffer.cpp
    ${SBS_BASE}/libs/tscpp/stream.cpp
)
set(UBLOXGPS_SOURCES
    ${SBS_BASE}/src/shared/drivers/gps/ublox/UbloxGPS.cpp
)
set(CANBUS_SOURCES
    ${SBS_BASE}/src/shared/drivers/canbus/CanManager.cpp
    ${SBS_BASE}/src/shared/drivers/canbus/CanBus.cpp
    ${SBS_BASE}/src/shared/drivers/canbus/CanInterrupt.cpp
)
set(CALIBRATION_SOURCES
    ${SBS_BASE}/src/shared/sensors/calibration/SensorDataExtra.cpp
)
set(PIKSI_SOURCES
    ${SBS_BASE}/src/shared/drivers/old_examples/piksi/piksi.cpp
)
set(PWM_SOURCES
    ${SBS_BASE}/src/shared/drivers/pwm/pwm.cpp
)
set(SPI_SOURCES
    ${SBS_BASE}/src/shared/drivers/spi/SPITransaction.cpp
)
set(I2C_SOURCES
    ${SBS_BASE}/src/shared/drivers/i2c/stm32f2_f4_i2c.cpp
)
set(ETHERNET_SOURCES
    ${SBS_BASE}/src/shared/drivers/ethernet/UdpManager.cpp
    ${SBS_BASE}/src/shared/drivers/ethernet/W5200/w5200.cpp
    ${SBS_BASE}/src/shared/drivers/ethernet/W5200/spi_impl.cpp
    ${SBS_BASE}/src/shared/drivers/ethernet/PacketBuffer.cpp
    ${SBS_BASE}/src/shared/drivers/ethernet/WatchdogTimer.cpp
)
set(ANAKIN_DATA_SOURCES
    ${SBS_BASE}/src/shared/boards/AnakinBoard.cpp
    ${SBS_BASE}/src/shared/drivers/Leds.cpp
)
set(SENSORMANAGER_SOURCES
    ${SBS_BASE}/src/shared/sensors/SensorManager.cpp
    ${SBS_BASE}/src/shared/sensors/SensorSampler.cpp
)
set(MATH_SOURCES
    ${SBS_BASE}/src/shared/math/SkyQuaternion.cpp
    ${SBS_BASE}/src/shared/math/Matrix.cpp
)
set(GAMMA868_SOURCES
    ${SBS_BASE}/src/shared/drivers/gamma868/Gamma868.cpp
)
set(XBEE_SOURCES
    ${SBS_BASE}/src/shared/drivers/Xbee/APIFrameParser.cpp
    ${SBS_BASE}/src/shared/drivers/Xbee/Xbee.cpp
)
set(EVENTS_SOURCES
    ${SBS_BASE}/src/shared/events/EventBroker.cpp
)
set(SERVO_SOURCES
    ${SBS_BASE}/src/shared/drivers/servo/servo.cpp
)
set(HBRIDGE_SOURCES
    ${SBS_BASE}/src/shared/drivers/hbridge/HBridge.cpp
)
set(TEST_UTILS_SOURCES
    ${SBS_BASE}/src/shared/utils/testutils/TestHelper.cpp
)
set(TESTS_BOARDCORE_SOURCES
    ${SBS_BASE}/src/tests/catch/test-aero.cpp
    ${SBS_BASE}/src/tests/catch/test-buttonhandler.cpp
    ${SBS_BASE}/src/tests/catch/test-circularbuffer.cpp
    ${SBS_BASE}/src/tests/catch/test-eventbroker.cpp
    ${SBS_BASE}/src/tests/catch/test-sensormanager-catch.cpp
    ${SBS_BASE}/src/tests/catch/test-hardwaretimer.cpp
    #${SBS_BASE}/src/tests/catch/test-kalman.cpp
    #${SBS_BASE}/src/tests/catch/test-kalman-eigen.cpp
    #${SBS_BASE}/src/tests/catch/test-matrix.cpp
    ${SBS_BASE}/src/tests/catch/test-packetqueue.cpp
    ${SBS_BASE}/src/tests/catch/spidriver/test-spidriver.cpp
    ${SBS_BASE}/src/tests/catch/xbee/test-xbee-parser.cpp
    ${SBS_BASE}/src/tests/catch/xbee/test-xbee-driver.cpp
)
set(INTERNAL_ADC_SOURCES
    ${SBS_BASE}/src/shared/drivers/adc/InternalADC/InternalADC.cpp
)
set(ADS1118_SOURCES
    ${SBS_BASE}/src/shared/drivers/adc/ADS1118/ADS1118.cpp
)
set(MPU9250_SOURCES
    ${SBS_BASE}/src/shared/sensors/MPU9250/MPU9250.cpp
)
set(BME280_SOURCES
    ${SBS_BASE}/src/shared/sensors/BME280/BME280.cpp
)
set(BMX160_SOURCES
    ${SBS_BASE}/src/shared/sensors/BMX160/BMX160.cpp
)
set(BMX160WITHCORRECTION_SOURCES
    ${SBS_BASE}/src/shared/sensors/BMX160/BMX160WithCorrection.cpp
)
set(MS5803_SOURCES
    ${SBS_BASE}/src/shared/sensors/MS5803/MS5803.cpp
)
