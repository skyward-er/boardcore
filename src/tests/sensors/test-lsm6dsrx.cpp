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
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <utils/Debug.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

using namespace Boardcore;
using namespace miosix;

int main()
{
    // printf("ciao\n");
    // return 0;

    SCB_DisableDCache();
    SCB_DisableICache();

    SPIBus bus(SPI3);

    bus.enableRxDMARequest();
    bus.enableTxDMARequest();

    auto streamRx = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_SPI3_RX);
    if(!streamRx.isValid())
    {
        std::cout << "Error, cannot obtain streamRx" << std::endl;
        return 0;
    }

    auto streamTx = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_SPI3_TX);
    if(!streamTx.isValid())
    {
        std::cout << "Error, cannot obtain streamTx" << std::endl;
        return 0;
    }

    // GpioPin csPin(GPIOE_BASE, 3);  // PE3 CS
    GpioPin csPin = miosix::sensors::LSM6DSRX0::cs::getPin();  // PE3 CS
    csPin.mode(Mode::OUTPUT);

    // GpioPin clockPin(GPIOB_BASE, 3);  // PB3 CK (SCL)
    // clockPin.mode(Mode::ALTERNATE);
    // clockPin.alternateFunction(6);
    // GpioPin misoPin(GPIOB_BASE, 4);  // PB4 MISO (SDO)
    // misoPin.alternateFunction(6);
    // misoPin.mode(Mode::ALTERNATE);
    // GpioPin mosiPin(GPIOB_BASE, 5);  // PB5 MOSI (SDA)
    // mosiPin.alternateFunction(6);
    // mosiPin.mode(Mode::ALTERNATE);

    // GpioPin int1Pin(GPIOC_BASE, 15);  // PC15 interrupt pin 1
    // int1Pin.mode(Mode::INPUT);
    // GpioPin int2Pin(GPIOC_BASE, 13);  // PC13 interrupt pin 2
    // int2Pin.mode(Mode::INPUT);

    SPIBusConfig busConfiguration;  // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_64;
    busConfiguration.mode =
        SPI::Mode::MODE_0;  // Set clock polarity to 0 and phase to 1

    LSM6DSRXConfig sensConfig;
    sensConfig.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    // acc
    sensConfig.fsAcc     = LSM6DSRXConfig::ACC_FULLSCALE::G2;
    sensConfig.odrAcc    = LSM6DSRXConfig::ACC_ODR::HZ_52;
    sensConfig.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    // gyr
    sensConfig.fsGyr     = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
    sensConfig.odrGyr    = LSM6DSRXConfig::GYR_ODR::HZ_52;
    sensConfig.opModeGyr = LSM6DSRXConfig::OPERATING_MODE::NORMAL;

    // fifo
    sensConfig.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    sensConfig.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    sensConfig.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::HZ_52;

    // interrupt
    sensConfig.int1InterruptSelection = LSM6DSRXConfig::INTERRUPT::NOTHING;
    sensConfig.int2InterruptSelection =
        LSM6DSRXConfig::INTERRUPT::FIFO_THRESHOLD;
    sensConfig.fifoWatermark = 170;

    std::unique_ptr<LSM6DSRX> sens =
        std::make_unique<LSM6DSRX>(bus, csPin, streamRx, streamTx, busConfiguration, sensConfig);

    if (sens->init() == false)
    {
        std::cout << "Error, sensor not initialized\n\n";
        return 0;
    }

    if (sens->selfTest())
    {
        std::cout << "Self test successful\n\n";
        Thread::sleep(500);
    }
    else
    {
        std::cout << "Self test failed\n\n";
        return 0;
    }

    sens->checkWhoamiDma();
    std::cout << "Terminating\n";
    return 0;

    while (true)
    {
        // wait for fifo full interrupt
        // int dataReady = intPin.value();
        // while (dataReady != 1)
        // {
        //     Thread::sleep(20);
        //     dataReady = intPin.value();
        // }

        const auto begin = std::chrono::steady_clock::now();
        sens->sample();
        const auto end = std::chrono::steady_clock::now();

        uint16_t fifoSize = 0;
        const std::array<LSM6DSRXData, LSM6DSRXDefs::FIFO_SIZE>& buf =
            sens->getLastFifo(fifoSize);

        // Print fifo
        std::cout << "first fifo element:\n";
        buf[0].print(std::cout);
        // std::cout << "last fifo element:\n";
        // buf[sens->getLastFifoSize() - 1].print(std::cout);
        std::cout << "time elapsed: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         end - begin)
                         .count()
                  << " ms\n";

        Thread::sleep(1000);
    }

    return 0;
}
