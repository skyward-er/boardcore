/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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
#include <miosix.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

int main()
{
    SPIBus bus(SPI3);

    GpioPin csPin(GPIOE_BASE, 3);  // PE3 CS
    csPin.mode(Mode::OUTPUT);

    GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK (SCL)
    clockPin.mode(Mode::ALTERNATE);
    clockPin.alternateFunction(6);
    GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO (SDO)
    misoPin.alternateFunction(6);
    misoPin.mode(Mode::ALTERNATE);
    GpioPin mosiPin(GPIOB_BASE, 5);  // PB5 MOSI (SDA)
    mosiPin.alternateFunction(6);
    mosiPin.mode(Mode::ALTERNATE);

    SPIBusConfig busConfiguration;  // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_256;
    busConfiguration.mode =
        SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

    LSM6DSRXConfig sensConfig;
    sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    sensConfig.fsAcc     = LSM6DSRXConfig::ACC_FULLSCALE::G4;
    sensConfig.odrAcc    = LSM6DSRXConfig::ACC_ODR::HZ_416;
    sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    sensConfig.fsGyr     = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
    sensConfig.odrGyr    = LSM6DSRXConfig::GYR_ODR::HZ_416;
    sensConfig.opModeGyr = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    sensConfig.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    sensConfig.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DISABLED;
    sensConfig.fifoTemperatureBdr =
        LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

    LSM6DSRX sens(bus, csPin, busConfiguration, sensConfig);

    if (sens.init() == false)
    {
        while (true)
        {
            TRACE("Error, sensor not initialized\n\n");
            Thread::sleep(2000);
        }
    }

    if (sens.selfTest())
    {
        TRACE("Self test successful\n\n");
        Thread::sleep(2000);
        // while(true)
        // {
        //     Thread::sleep(2000);
        // }
    }
    else
    {
        TRACE("Self test failed\n\n");
        while (true)
        {
            Thread::sleep(2000);
        }
    }

    Thread::sleep(
        1000);  // sleep in order to produce some data before reading from FIFO.

    // LSM6DSRX::SensorData data{0.0, 0.0, 0.0};
    const int SIZE = 4;
    LSM6DSRX::FifoData buf[SIZE];

    while (true)
    {
        // sens.getAccelerometerData(data);
        // TRACE("Accelerometer:\n");
        // TRACE("x: %f\n", data.x);
        // TRACE("y: %f\n", data.y);
        // TRACE("z: %f\n\n", data.z);

        // sens.getGyroscopeData(data);
        // TRACE("Gyroscope:\n");
        // TRACE("x: %f\n", data.x);
        // TRACE("y: %f\n", data.y);
        // TRACE("z: %f\n\n\n", data.z);

        const int numBatchRed = sens.readFromFifo(buf, SIZE);

        TRACE("Number of batch in buf: %d\n", numBatchRed);
        float sensitivity = 0.0;
        for (int i = 0; i < numBatchRed; ++i)
        {
            // read sensor tag
            uint8_t tagSensor = (buf[i].tag >> 3) & 31;
            switch (tagSensor)
            {
                case 0x01:
                    // gyroscope
                    TRACE("%d) GYROSCOPE\n", i);
                    sensitivity = 4.375;
                    break;
                case 0x02:
                    // accelerometer
                    TRACE("%d) ACCELEROMETER\n", i);
                    sensitivity = 0.122;
                    break;
                default:
                    TRACE("%d) UNRECOGNIZED DATA\n", i);
                    sensitivity = 1.0;
                    break;
            }

            // print data
            TRACE("x: %f\n", static_cast<float>(buf[i].x) * sensitivity);
            TRACE("y: %f\n", static_cast<float>(buf[i].y) * sensitivity);
            TRACE("z: %f\n\n\n", static_cast<float>(buf[i].z) * sensitivity);
        }

        Thread::sleep(2000);
    }

    return 0;
}