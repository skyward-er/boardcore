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

include(dependencies)

add_custom_target(boardcore)

foreach(OPT_BOARD ${BOARDS})
    set(BOARDCORE_LIBRARY boardcore-${OPT_BOARD})
    add_library(${BOARDCORE_LIBRARY} STATIC EXCLUDE_FROM_ALL
        # Debug
        ${SBS_BASE}/src/shared/Debug.cpp
        ${SBS_BASE}/src/shared/diagnostic/CpuMeter.cpp
        ${SBS_BASE}/src/shared/diagnostic/PrintLogger.cpp

        # Drivers
        ${SBS_BASE}/src/shared/drivers/adc/ADS1118/ADS1118.cpp
        ${SBS_BASE}/src/shared/drivers/adc/InternalADC/InternalADC.cpp
        ${SBS_BASE}/src/shared/drivers/canbus/Canbus.cpp
        ${SBS_BASE}/src/shared/drivers/canbus/CanInterrupt.cpp
        ${SBS_BASE}/src/shared/drivers/gamma868/Gamma868.cpp
        ${SBS_BASE}/src/shared/drivers/gps/ublox/UbloxGPS.cpp
        ${SBS_BASE}/src/shared/drivers/hbridge/HBridge.cpp
        ${SBS_BASE}/src/shared/drivers/i2c/stm32f2_f4_i2c.cpp
        ${SBS_BASE}/src/shared/drivers/interrupt/external_interrupts.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ethernet/PacketBuffer.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ethernet/UdpManager.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ethernet/W5200/spi_impl.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ethernet/W5200/w5200.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ethernet/WatchdogTimer.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ISB_protocol/IsbProtocol_serial2.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/ISB_protocol/IsbProtocol_serial3.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/interrupt/InterruptManager.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/Leds.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/memory/MultiFlashController.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/modbus/PDU.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/modbus/slave/RtuSlave.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/modbus/slave/SlaveEngine.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/modbus/slave/old/SlaveInterface.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/modbus/slave/old/Timer.cpp
        #${SBS_BASE}/src/shared/drivers/old_examples/piksi/piksi.cpp
        ${SBS_BASE}/src/shared/drivers/pwm/pwm.cpp
        ${SBS_BASE}/src/shared/drivers/servo/servo.cpp
        ${SBS_BASE}/src/shared/drivers/spi/SPITransaction.cpp
        ${SBS_BASE}/src/shared/drivers/spi/SensorSpi.cpp
        ${SBS_BASE}/src/shared/drivers/Xbee/APIFrameParser.cpp
        ${SBS_BASE}/src/shared/drivers/Xbee/Xbee.cpp

        # Events
        ${SBS_BASE}/src/shared/events/EventBroker.cpp

        # Logger
        ${SBS_BASE}/src/shared/logger/Logger.cpp
        #${SBS_BASE}/src/shared/logger/decoder/logdecoder.cpp

        # Math
        ${SBS_BASE}/src/shared/math/Matrix.cpp
        ${SBS_BASE}/src/shared/math/SkyQuaternion.cpp
        ${SBS_BASE}/src/shared/math/Stats.cpp

        # Scheduler
        ${SBS_BASE}/src/shared/scheduler/TaskScheduler.cpp

        # Sensors
        ${SBS_BASE}/src/shared/sensors/BME280/BME280.cpp
        ${SBS_BASE}/src/shared/sensors/BMX160/BMX160.cpp
        ${SBS_BASE}/src/shared/sensors/BMX160/BMX160WithCorrection.cpp
        ${SBS_BASE}/src/shared/sensors/calibration/SensorDataExtra.cpp
        ${SBS_BASE}/src/shared/sensors/MPU9250/MPU9250.cpp
        ${SBS_BASE}/src/shared/sensors/MS5803/MS5803.cpp
        ${SBS_BASE}/src/shared/sensors/SensorManager.cpp
        ${SBS_BASE}/src/shared/sensors/SensorSampler.cpp

        # Timer
        ${SBS_BASE}/src/shared/TimestampTimer.cpp

        # AeroUtils
        ${SBS_BASE}/src/shared/utils/aero/AeroUtils.cpp

        # TestUtils
        ${SBS_BASE}/src/shared/utils/testutils/TestHelper.cpp
    )
    add_library(SkywardBoardcore::Boardcore-${OPT_BOARD} ALIAS ${BOARDCORE_LIBRARY})
    target_include_directories(${BOARDCORE_LIBRARY} PUBLIC ${SBS_BASE}/src/shared)
    target_link_libraries(${BOARDCORE_LIBRARY} PUBLIC
        Miosix::Miosix-${OPT_BOARD}
        Mxgui::Mxgui-${OPT_BOARD}
        TSCPP::TSCPP
        Eigen3::Eigen
        fmt::fmt-header-only
        Catch2::Catch2
        Mavlink::Mavlink
    )
    add_dependencies(boardcore boardcore-${OPT_BOARD})
endforeach()
