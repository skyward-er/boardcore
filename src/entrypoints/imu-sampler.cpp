/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <utils/Debug.h>

#include <iostream>

using namespace Boardcore;
using namespace Eigen;
using namespace miosix;

void printData(LSM6DSRXData data)
{
    std::cout << data.accelerationTimestamp << "," << data.accelerationX << ","
              << data.accelerationY << "," << data.accelerationZ << ","
              << data.angularSpeedTimestamp << "," << data.angularSpeedX << ","
              << data.angularSpeedY << "," << data.angularSpeedZ << ","
              << data.temperatureTimestamp << "," << data.temperature;
}

void printHeader()
{
    std::cout << "accelerationTimestamp," << "accelerationX,"
              << "accelerationY," << "accelerationZ,"
              << "angularSpeedTimestamp," << "angularSpeedX,"
              << "angularSpeedY," << "angularSpeedZ,"
              << "temperatureTimestamp," << "temperature,"

              << "accelerationTimestamp," << "accelerationX,"
              << "accelerationY," << "accelerationZ,"
              << "angularSpeedTimestamp," << "angularSpeedX,"
              << "angularSpeedY," << "angularSpeedZ,"
              << "temperatureTimestamp," << "temperature" << std::endl;
}

int main()
{
    SPIBus bus(SPI3);

    GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK (SCL)
    clockPin.mode(Mode::ALTERNATE);
    clockPin.alternateFunction(6);
    GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO (SDO)
    misoPin.alternateFunction(6);
    misoPin.mode(Mode::ALTERNATE);
    GpioPin mosiPin(GPIOD_BASE, 6);  // PD6 MOSI (SDA)
    mosiPin.alternateFunction(6);
    mosiPin.mode(Mode::ALTERNATE);

    // LSM6DRSX_0
    GpioPin csPin0(GPIOD_BASE, 12);  // PD12 CS
    csPin.mode(Mode::OUTPUT);

    // LSM6DRSX_1
    GpioPin csPin1(GPIOC_BASE, 7);  // PC7 CS
    csPin.mode(Mode::OUTPUT);

    SPIBusConfig busConfiguration;  // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_64;
    busConfiguration.mode =
        SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

    // using the same sensor config for both sensors
    LSM6DSRXConfig sensConfig;
    sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    // acc
    sensConfig.fsAcc     = LSM6DSRXConfig::ACC_FULLSCALE::G16;
    sensConfig.odrAcc    = LSM6DSRXConfig::ACC_ODR::HZ_416;
    sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

    // gyr
    sensConfig.fsGyr     = LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000;
    sensConfig.odrGyr    = LSM6DSRXConfig::GYR_ODR::HZ_416;
    sensConfig.opModeGyr = LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

    LSM6DSRX imu0(bus, csPin0, busConfiguration, sensConfig);
    LSM6DSRX imu1(bus, csPin1, busConfiguration, sensConfig);

    if (imu0.init() == false)
    {
        std::cout << "Error, lsm6dsrx_0 failed to initialized\n\n";
        return 0;
    }

    if (imu1.init() == false)
    {
        std::cout << "Error, lsm6dsrx_1 failed to initialized\n\n";
        return 0;
    }

    if (imu0.selfTest() == false)
    {
        std::cout << "lsm6dsrx_0 failed self test\n\n";
        return 0;
    }

    if (imu1.selfTest() == false)
    {
        std::cout << "lsm6dsrx_1 failed self test\n\n";
        return 0;
    }

    printHeader();

    while (true)
    {
        printData(imu0.getLastSample());
        std::cout << ",";

        printData(imu1.getLastSample());
        std::cout << std::endl;

        miosix::Thread::sleep(100);
    }

    return 0;
}

