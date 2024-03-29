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

include(boardcore-host)

foreach(OPT_BOARD ${BOARDS})
    set(BOARDCORE_LIBRARY boardcore-${OPT_BOARD})
    add_library(${BOARDCORE_LIBRARY} STATIC EXCLUDE_FROM_ALL

        # Actuators
        ${SBS_BASE}/src/shared/actuators/HBridge/HBridge.cpp
        ${SBS_BASE}/src/shared/actuators/Servo/Servo.cpp
        ${SBS_BASE}/src/shared/actuators/stepper/Stepper.cpp
        ${SBS_BASE}/src/shared/actuators/stepper/StepperPWM.cpp

        # Algorithms
        ${SBS_BASE}/src/shared/algorithms/ADA/ADA.cpp
        ${SBS_BASE}/src/shared/algorithms/MEA/MEA.cpp
        ${SBS_BASE}/src/shared/algorithms/AirBrakes/AirBrakes.cpp
        ${SBS_BASE}/src/shared/algorithms/AirBrakes/AirBrakesPI.cpp
        ${SBS_BASE}/src/shared/algorithms/AirBrakes/AirBrakesInterp.cpp
        ${SBS_BASE}/src/shared/algorithms/NAS/NAS.cpp
        ${SBS_BASE}/src/shared/algorithms/NAS/StateInitializer.cpp

        # Debug
        ${SBS_BASE}/src/shared/utils/Debug.cpp
        ${SBS_BASE}/src/shared/diagnostic/CpuMeter/CpuMeter.cpp
        ${SBS_BASE}/src/shared/diagnostic/PrintLogger.cpp

        # Drivers
        ${SBS_BASE}/src/shared/drivers/AD5204/AD5204.cpp
        ${SBS_BASE}/src/shared/drivers/adc/InternalADC.cpp
        ${SBS_BASE}/src/shared/drivers/canbus/CanDriver/CanDriver.cpp
        ${SBS_BASE}/src/shared/drivers/canbus/CanDriver/CanInterrupt.cpp
        ${SBS_BASE}/src/shared/drivers/canbus/CanProtocol/CanProtocol.cpp
        ${SBS_BASE}/src/shared/drivers/interrupt/external_interrupts.cpp
        ${SBS_BASE}/src/shared/drivers/timer/PWM.cpp
        ${SBS_BASE}/src/shared/drivers/timer/CountedPWM.cpp
        ${SBS_BASE}/src/shared/drivers/timer/TimestampTimer.cpp
        ${SBS_BASE}/src/shared/drivers/runcam/Runcam.cpp
        ${SBS_BASE}/src/shared/drivers/spi/SPITransaction.cpp
        ${SBS_BASE}/src/shared/drivers/usart/USART.cpp
        ${SBS_BASE}/src/shared/drivers/i2c/I2CDriver-f4.cpp
        ${SBS_BASE}/src/shared/drivers/i2c/I2CDriver-f7.cpp
        ${SBS_BASE}/src/shared/drivers/i2c/I2C.cpp
        ${SBS_BASE}/src/shared/drivers/WIZ5500/WIZ5500.cpp

        # Events
        ${SBS_BASE}/src/shared/events/EventBroker.cpp

        # Logger
        ${SBS_BASE}/src/shared/logger/Logger.cpp

        # Radio
        ${SBS_BASE}/src/shared/radio/gamma868/Gamma868.cpp
        ${SBS_BASE}/src/shared/radio/Xbee/APIFrameParser.cpp
        ${SBS_BASE}/src/shared/radio/Xbee/Xbee.cpp
        ${SBS_BASE}/src/shared/radio/SX1278/SX1278Fsk.cpp
        ${SBS_BASE}/src/shared/radio/SX1278/SX1278Lora.cpp
        ${SBS_BASE}/src/shared/radio/SX1278/SX1278Common.cpp

        # Scheduler
        ${SBS_BASE}/src/shared/scheduler/TaskScheduler.cpp

        # Sensors
        ${SBS_BASE}/src/shared/sensors/ADS1118/ADS1118.cpp
        ${SBS_BASE}/src/shared/sensors/ADS131M04/ADS131M04.cpp
        ${SBS_BASE}/src/shared/sensors/ADS131M08/ADS131M08.cpp
        ${SBS_BASE}/src/shared/sensors/BME280/BME280.cpp
        ${SBS_BASE}/src/shared/sensors/BME280/BME280I2C.cpp
        ${SBS_BASE}/src/shared/sensors/BMP280/BMP280.cpp
        ${SBS_BASE}/src/shared/sensors/BMP280/BMP280I2C.cpp
        ${SBS_BASE}/src/shared/sensors/BMX160/BMX160.cpp
        ${SBS_BASE}/src/shared/sensors/BMX160/BMX160WithCorrection.cpp
        ${SBS_BASE}/src/shared/sensors/H3LIS331DL/H3LIS331DL.cpp
        ${SBS_BASE}/src/shared/sensors/HX711/HX711.cpp
        ${SBS_BASE}/src/shared/sensors/LIS3MDL/LIS3MDL.cpp
        ${SBS_BASE}/src/shared/sensors/LIS331HH/LIS331HH.cpp
        ${SBS_BASE}/src/shared/sensors/LPS331AP/LPS331AP.cpp
        ${SBS_BASE}/src/shared/sensors/MAX6675/MAX6675.cpp
        ${SBS_BASE}/src/shared/sensors/MAX31855/MAX31855.cpp
        ${SBS_BASE}/src/shared/sensors/MAX31856/MAX31856.cpp
        ${SBS_BASE}/src/shared/sensors/MBLoadCell/MBLoadCell.cpp
        ${SBS_BASE}/src/shared/sensors/MPU9250/MPU9250.cpp
        ${SBS_BASE}/src/shared/sensors/MS5803/MS5803.cpp
        ${SBS_BASE}/src/shared/sensors/MS5803/MS5803I2C.cpp
        ${SBS_BASE}/src/shared/sensors/SensorManager.cpp
        ${SBS_BASE}/src/shared/sensors/SensorSampler.cpp
        ${SBS_BASE}/src/shared/sensors/UBXGPS/UBXGPSSerial.cpp
        ${SBS_BASE}/src/shared/sensors/UBXGPS/UBXGPSSpi.cpp
        ${SBS_BASE}/src/shared/sensors/VN100/VN100.cpp
        ${SBS_BASE}/src/shared/sensors/LIS2MDL/LIS2MDL.cpp
        ${SBS_BASE}/src/shared/sensors/LPS28DFW/LPS28DFW.cpp
        ${SBS_BASE}/src/shared/sensors/LPS22DF/LPS22DF.cpp
        ${SBS_BASE}/src/shared/sensors/LSM6DSRX/LSM6DSRX.cpp

        # Calibration
        ${SBS_BASE}/src/shared/sensors/calibration/BiasCalibration/BiasCalibration.cpp
        ${SBS_BASE}/src/shared/sensors/calibration/SensorDataExtra/SensorDataExtra.cpp
        ${SBS_BASE}/src/shared/sensors/calibration/SixParameterCalibration/SixParameterCalibration.cpp
        ${SBS_BASE}/src/shared/sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.cpp

        # Correction
        ${SBS_BASE}/src/shared/sensors/correction/BiasCorrector/BiasCorrector.cpp
        ${SBS_BASE}/src/shared/sensors/correction/SixParametersCorrector/SixParametersCorrector.cpp

        # Utils
        ${SBS_BASE}/src/shared/utils/AeroUtils/AeroUtils.cpp
        ${SBS_BASE}/src/shared/utils/ButtonHandler/ButtonHandler.cpp
        ${SBS_BASE}/src/shared/utils/PinObserver/PinObserver.cpp
        ${SBS_BASE}/src/shared/utils/SkyQuaternion/SkyQuaternion.cpp
        ${SBS_BASE}/src/shared/utils/Stats/Stats.cpp
        ${SBS_BASE}/src/shared/utils/TestUtils/TestHelper.cpp
    )
    add_library(SkywardBoardcore::Boardcore::${OPT_BOARD} ALIAS ${BOARDCORE_LIBRARY})
    target_include_directories(${BOARDCORE_LIBRARY} PUBLIC ${SBS_BASE}/src/shared)
    target_link_libraries(${BOARDCORE_LIBRARY} PUBLIC
        Miosix::Miosix::${OPT_BOARD}
        TSCPP::TSCPP
        Eigen3::Eigen
        fmt::fmt-header-only
        Catch2::Catch2
        Mavlink::Mavlink
    )

    # Link MxGui only if supported by the target
    if(${OPT_BOARD} IN_LIST MXGUI_BOARDS)
        target_link_libraries(${BOARDCORE_LIBRARY} PUBLIC Mxgui::Mxgui::${OPT_BOARD})
    endif()
endforeach()
