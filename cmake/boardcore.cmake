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

# Load in BOARDCORE_PATH the project path
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH BOARDCORE_PATH)

# Include dependencies and board list
include(${BOARDCORE_PATH}/cmake/dependencies.cmake)
include(${BOARDCORE_PATH}/cmake/boardcore-host.cmake)
include(${BOARDCORE_PATH}/cmake/boards.cmake)

# Boardcore source files
set(BOARDCORE_SRC
    # Actuators
    ${BOARDCORE_PATH}/src/shared/actuators/HBridge/HBridge.cpp
    ${BOARDCORE_PATH}/src/shared/actuators/Servo/Servo.cpp
    ${BOARDCORE_PATH}/src/shared/actuators/stepper/Stepper.cpp
    ${BOARDCORE_PATH}/src/shared/actuators/stepper/StepperPWM.cpp

    # Algorithms
    ${BOARDCORE_PATH}/src/shared/algorithms/ADA/ADA.cpp
    ${BOARDCORE_PATH}/src/shared/algorithms/MEA/MEA.cpp
    ${BOARDCORE_PATH}/src/shared/algorithms/AirBrakes/AirBrakes.cpp
    ${BOARDCORE_PATH}/src/shared/algorithms/AirBrakes/AirBrakesPI.cpp
    ${BOARDCORE_PATH}/src/shared/algorithms/AirBrakes/AirBrakesInterp.cpp
    ${BOARDCORE_PATH}/src/shared/algorithms/NAS/NAS.cpp
    ${BOARDCORE_PATH}/src/shared/algorithms/NAS/StateInitializer.cpp

    # Debug
    ${BOARDCORE_PATH}/src/shared/utils/Debug.cpp
    ${BOARDCORE_PATH}/src/shared/diagnostic/CpuMeter/CpuMeter.cpp
    ${BOARDCORE_PATH}/src/shared/diagnostic/PrintLogger.cpp

    # Drivers
    ${BOARDCORE_PATH}/src/shared/drivers/AD5204/AD5204.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/adc/InternalADC.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/canbus/CanDriver/CanDriver.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/canbus/CanDriver/CanInterrupt.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/canbus/CanProtocol/CanProtocol.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/interrupt/external_interrupts.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/timer/PWM.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/timer/CountedPWM.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/timer/TimestampTimer.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/runcam/Runcam.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/spi/SPITransaction.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/usart/USART.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/i2c/I2CDriver-f4.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/i2c/I2CDriver-f7.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/i2c/I2C.cpp
    ${BOARDCORE_PATH}/src/shared/drivers/WIZ5500/WIZ5500.cpp

    # Events
    ${BOARDCORE_PATH}/src/shared/events/EventBroker.cpp

    # Logger
    ${BOARDCORE_PATH}/src/shared/logger/Logger.cpp

    # Radio
    ${BOARDCORE_PATH}/src/shared/radio/gamma868/Gamma868.cpp
    ${BOARDCORE_PATH}/src/shared/radio/Xbee/APIFrameParser.cpp
    ${BOARDCORE_PATH}/src/shared/radio/Xbee/Xbee.cpp
    ${BOARDCORE_PATH}/src/shared/radio/SX1278/SX1278Fsk.cpp
    ${BOARDCORE_PATH}/src/shared/radio/SX1278/SX1278Lora.cpp
    ${BOARDCORE_PATH}/src/shared/radio/SX1278/SX1278Common.cpp

    # Scheduler
    ${BOARDCORE_PATH}/src/shared/scheduler/TaskScheduler.cpp

    # Sensors
    ${BOARDCORE_PATH}/src/shared/sensors/ADS1118/ADS1118.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/ADS131M04/ADS131M04.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/ADS131M08/ADS131M08.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/BME280/BME280.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/BME280/BME280I2C.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/BMP280/BMP280.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/BMP280/BMP280I2C.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/BMX160/BMX160.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/BMX160/BMX160WithCorrection.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/H3LIS331DL/H3LIS331DL.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/HX711/HX711.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LIS3MDL/LIS3MDL.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LIS331HH/LIS331HH.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LPS331AP/LPS331AP.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MAX6675/MAX6675.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MAX31855/MAX31855.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MAX31856/MAX31856.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MBLoadCell/MBLoadCell.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MPU9250/MPU9250.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MS5803/MS5803.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/MS5803/MS5803I2C.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/SensorManager.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/SensorSampler.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/UBXGPS/UBXGPSSerial.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/UBXGPS/UBXGPSSpi.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/VN100/VN100.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LIS2MDL/LIS2MDL.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LPS28DFW/LPS28DFW.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LPS22DF/LPS22DF.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/LSM6DSRX/LSM6DSRX.cpp

    # Calibration
    ${BOARDCORE_PATH}/src/shared/sensors/calibration/BiasCalibration/BiasCalibration.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/calibration/SensorDataExtra/SensorDataExtra.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/calibration/SixParameterCalibration/SixParameterCalibration.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.cpp

    # Correction
    ${BOARDCORE_PATH}/src/shared/sensors/correction/BiasCorrector/BiasCorrector.cpp
    ${BOARDCORE_PATH}/src/shared/sensors/correction/SixParametersCorrector/SixParametersCorrector.cpp

    # Utils
    ${BOARDCORE_PATH}/src/shared/utils/AeroUtils/AeroUtils.cpp
    ${BOARDCORE_PATH}/src/shared/utils/ButtonHandler/ButtonHandler.cpp
    ${BOARDCORE_PATH}/src/shared/utils/PinObserver/PinObserver.cpp
    ${BOARDCORE_PATH}/src/shared/utils/SkyQuaternion/SkyQuaternion.cpp
    ${BOARDCORE_PATH}/src/shared/utils/Stats/Stats.cpp
    ${BOARDCORE_PATH}/src/shared/utils/TestUtils/TestHelper.cpp
)

# Creates the Skyward::Boardcore::${BOARD_NAME} library
function(add_boardcore_library BOARD_OPTIONS_FILE)
    # Get board options
    include(${BOARD_OPTIONS_FILE})

    # Create a library for the board
    set(BOARDCORE_LIB boardcore-${BOARD_NAME})
    add_library(${BOARDCORE_LIB} STATIC EXCLUDE_FROM_ALL ${BOARDCORE_SRC})

    # Only one include directory for Boardcore!
    target_include_directories(${BOARDCORE_LIB} PUBLIC ${BOARDCORE_PATH}/src/shared)

    # Define DEBUG when in Debug mode
    target_compile_definitions(${BOARDCORE_LIB} PUBLIC $<$<CONFIG:Debug>:DEBUG>) 

    # Link libraries
    target_link_libraries(${BOARDCORE_LIB} PUBLIC
        $<TARGET_OBJECTS:Miosix::Boot::${BOARD_NAME}>
        $<LINK_GROUP:RESCAN,Miosix::Kernel::${BOARD_NAME},stdc++,c,m,gcc,atomic>
        TSCPP::TSCPP
        Eigen3::Eigen
        fmt::fmt-header-only
        Catch2::Catch2
        Mavlink::Mavlink
    )

    # Link MxGui if supported by the target
    if(DEFINED MXGUI_BASE_BOARD_NAME)
        target_link_libraries(${BOARDCORE_LIB} PUBLIC MxGui::${MXGUI_BASE_BOARD_NAME})
    elseif(TARGET MxGui::${BOARD_NAME})
        target_link_libraries(${BOARDCORE_LIB} PUBLIC MxGui::${BOARD_NAME})
    endif()

    # Create a nice alias for the library
    add_library(Skyward::Boardcore::${BOARD_NAME} ALIAS ${BOARDCORE_LIB})
endfunction()

# Create the Miosix libraries for Boardcore custom boards
foreach(BOARD_OPTIONS_FILE ${BOARDCORE_BOARDS_OPTIONS_FILES})
    add_miosix_libraries(${BOARD_OPTIONS_FILE})
endforeach()

# Create Boardcore library for each board
foreach(BOARD_OPTIONS_FILE ${MIOSIX_BOARDS_OPTIONS_FILES} ${BOARDCORE_BOARDS_OPTIONS_FILES})
    add_boardcore_library(${BOARD_OPTIONS_FILE})
endforeach()
