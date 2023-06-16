/* Copyright (c) 2023 Skyward Experimental Rocketry
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
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <utils/Debug.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>

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

    GpioPin int1Pin(GPIOC_BASE, 15);  // PC15 interrupt pin 1
    int1Pin.mode(Mode::INPUT);
    GpioPin int2Pin(GPIOC_BASE, 13);  // PC13 interrupt pin 2
    int2Pin.mode(Mode::INPUT);

    SPIBusConfig busConfiguration;  // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_8;
    busConfiguration.mode =
        SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

    LSM6DSRXConfig sensConfig;
    sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    // acc
    sensConfig.fsAcc     = LSM6DSRXConfig::ACC_FULLSCALE::G2;
    sensConfig.odrAcc    = LSM6DSRXConfig::ACC_ODR::HZ_833;
    sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    // gyr
    sensConfig.fsGyr     = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
    sensConfig.odrGyr    = LSM6DSRXConfig::GYR_ODR::HZ_833;
    sensConfig.opModeGyr = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    // fifo
    sensConfig.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    sensConfig.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    sensConfig.fifoTemperatureBdr =
        LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

    // interrupt
    sensConfig.int1InterruptSelection = LSM6DSRXConfig::INTERRUPT::NOTHING;
    sensConfig.int2InterruptSelection =
        LSM6DSRXConfig::INTERRUPT::FIFO_THRESHOLD;
    sensConfig.fifoWatermark = 511;

    // LSM6DSRX* sens = new LSM6DSRX(bus, csPin, busConfiguration, sensConfig);

    // if (sens->init() == false)
    // {
    //     while (true)
    //     {
    //         TRACE("Error, sensor not initialized\n\n");
    //         Thread::sleep(2000);
    //     }
    // }

    // if (sens->selfTest())
    // {
    //     TRACE("Self test successful\n\n");
    //     Thread::sleep(2000);
    // }
    // else
    // {
    //     TRACE("Self test failed\n\n");
    //     while (true)
    //     {
    //         Thread::sleep(2000);
    //     }
    // }

    struct odrs
    {
        LSM6DSRXConfig::ACC_ODR acc;
        LSM6DSRXConfig::GYR_ODR gyr;
    };

    odrs arr[] = {
        {LSM6DSRXConfig::ACC_ODR::HZ_12_5, LSM6DSRXConfig::GYR_ODR::HZ_12_5},
        {LSM6DSRXConfig::ACC_ODR::HZ_26, LSM6DSRXConfig::GYR_ODR::HZ_26},
        {LSM6DSRXConfig::ACC_ODR::HZ_52, LSM6DSRXConfig::GYR_ODR::HZ_52},
        {LSM6DSRXConfig::ACC_ODR::HZ_104, LSM6DSRXConfig::GYR_ODR::HZ_104},
        {LSM6DSRXConfig::ACC_ODR::HZ_208, LSM6DSRXConfig::GYR_ODR::HZ_208},
        {LSM6DSRXConfig::ACC_ODR::HZ_416, LSM6DSRXConfig::GYR_ODR::HZ_416},
        {LSM6DSRXConfig::ACC_ODR::HZ_833, LSM6DSRXConfig::GYR_ODR::HZ_833},
        {LSM6DSRXConfig::ACC_ODR::HZ_1660, LSM6DSRXConfig::GYR_ODR::HZ_1660},
        {LSM6DSRXConfig::ACC_ODR::HZ_3330, LSM6DSRXConfig::GYR_ODR::HZ_3330},
        {LSM6DSRXConfig::ACC_ODR::HZ_6660, LSM6DSRXConfig::GYR_ODR::HZ_6660},
    };

    for(unsigned int odrIdx = 0; odrIdx < sizeof(arr) / sizeof(odrs); ++odrIdx)
    {
        sensConfig.odrAcc = arr[odrIdx].acc;
        sensConfig.odrGyr = arr[odrIdx].gyr;

        std::unique_ptr<LSM6DSRX> sens = std::make_unique<LSM6DSRX>(bus, csPin, busConfiguration, sensConfig);
        if (sens->init() == false)
        {
            while (true)
            {
                TRACE("Error, sensor not initialized at index %d\n\n", odrIdx);
                Thread::sleep(2000);
            }
        }

        // empty the fifo
        sens->sampleImpl();

        // test time needed to fill the fifo
        uint64_t t0 = TimestampTimer::getTimestamp();
        int dataReady = int2Pin.value();
        while (dataReady != 1)
        {
            Thread::sleep(1);
            dataReady = int2Pin.value();
        }
        uint64_t t1 = TimestampTimer::getTimestamp();

        uint64_t diff = t1 - t0;

        std::cout << odrIdx << ") Filling time(us): " << diff << "\n\n";
    }

    while(true)
    {
        Thread::sleep(5000);
    }

    // std::cout << "sensor initialized\n";
    // while (true)
    // {
    //     uint64_t t0 = TimestampTimer::getTimestamp();

    //     auto d = sens->sampleImpl();

    //     uint64_t t1 = TimestampTimer::getTimestamp();

    //     uint64_t diff = t1 - t0;

    //     std::cout << "sampleImpl() execution time(us): " << diff << "\n";
    //     std::cout << "last fifo sample:\n";
    //     d.print(std::cout);
    //     std::cout << "\n\n\n";

    //     miosix::Thread::sleep(1000);
    // }

    // while (true)
    // {
    //     // wait for fifo full interrupt
    //     int dataReady = int2Pin.value();
    //     while (dataReady != 1)
    //     {
    //         Thread::sleep(20);
    //         dataReady = int2Pin.value();
    //     }

    //     sens->sample();

    //     const std::array<LSM6DSRXData, LSM6DSRXDefs::FIFO_SIZE>& buf =
    //         sens->getLastFifo();

    //     // print last element from fifo
    //     // std::cout << buf[sens->getLastFifoSize() - 1].header() << "\n";
    //     // buf[sens->getLastFifoSize() - 1].print(std::cout);
    //     for(uint16_t i = 0; i < sens->getLastFifoSize(); ++i)
    //     {
    //         buf[i].print(std::cout);
    //         std::cout << "\n";
    //     }
    //     std::cout << "\n\n\n";
    // }

    // delete sens;
    return 0;
}
