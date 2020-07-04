/**
 * test LSM9DS1 axel + gyro
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "drivers/spi/SPIDriver.h"
#include "sensors/LSM9DS1/LSM9DS1_AxelGyro.h"
#include "sensors/LSM9DS1/LSM9DS1_Magneto.h"

using namespace miosix;

// pin f407 discovery SPI1
typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;

static const bool FIFO_ENABLED     = false;
static const uint8_t TEMP_DIV_FREQ = 20;

// SPI
SPIBus bus(SPI1);
GpioPin cs_XLG(GPIOE_BASE, 7);
GpioPin cs_M(GPIOE_BASE, 9);

// LEDs
GpioPin LED1(GPIOD_BASE, 15);
GpioPin LED2(GPIOD_BASE, 13);

int main()
{
    lsm9ds1XLGSample agdata;
    lsm9ds1MSample mdata;
    lsm9ds1TSample tdata;

    {
        FastInterruptDisableLock dLock;

        // SPI1 ENABLE
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        cs_XLG.mode(Mode::OUTPUT);
        cs_M.mode(Mode::OUTPUT);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        GpioSck::speed(Speed::_25MHz);

        LED1.mode(Mode::OUTPUT);
        LED2.mode(Mode::OUTPUT);
    }

    // chip select high
    cs_XLG.high();
    cs_M.high();

    LSM9DS1_XLG lsm9ds1X(bus, cs_XLG, LSM9DS1_XLG::AxelFSR::FS_8,
                         LSM9DS1_XLG::GyroFSR::FS_245,
                         LSM9DS1_XLG::ODR::ODR_952, TEMP_DIV_FREQ);

    LSM9DS1_M lsm9ds1M(bus, cs_M, LSM9DS1_M::MagFSR::FS_12,
                       LSM9DS1_M::ODR::ODR_80);

    lsm9ds1M.selfTest();
    
    while (!lsm9ds1X.init())
    {
    }
    LED1.high();

    while (!lsm9ds1M.init())
    {
    }
    LED2.high();

    Thread::sleep(5000);

    printf("time,ax,ay,az,gx,gy,gz,mx,my,mz,t\n");
    for (;;)
    {

        // get axel+gyro+temp data
        lsm9ds1X.updateTimestamp(miosix::getTick());
        lsm9ds1X.onSimpleUpdate();
        agdata = lsm9ds1X.getXLGSample();
        tdata  = lsm9ds1X.getTSample();

        // get magneto data
        lsm9ds1M.updateTimestamp(miosix::getTick());
        lsm9ds1M.onSimpleUpdate();
        mdata = lsm9ds1M.getSample();

        // clang-format off
               
        printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%d,%.1f\n",
               (int)agdata.timestamp,
               agdata.axelData.getX(), agdata.axelData.getY(), agdata.axelData.getZ(),
               agdata.gyroData.getX(), agdata.gyroData.getY(), agdata.gyroData.getZ(),
               (int)mdata.timestamp,
               mdata.magData.getX(), mdata.magData.getY(), mdata.magData.getZ(),
               (int)tdata.timestamp, 
               tdata.tempData);
        
        // clang-format on        

    }

    return 0;
}