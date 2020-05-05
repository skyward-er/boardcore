/* LSM9DS1 magnetometer Driver
 *
 * Copyright (c) 2016,2020 Skyward Experimental Rocketry
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

#include "LSM9DS1_Magneto.h"

#include <math.h>

#include <iostream>

#include "math/Stats.h"

using miosix::GpioPin;
using std::vector;

LSM9DS1_M::LSM9DS1_M(SPIBusInterface& bus, GpioPin cs, MagFSR magRange, ODR odr)
    : spislave(bus, cs, {}), magFSR(magRange), odr(odr)
{
    // SPI config
    spislave.config.clock_div = SPIClockDivider::DIV64;
}

LSM9DS1_M::LSM9DS1_M(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config,
                     MagFSR magRange, ODR odr)
    : spislave(bus, cs, config), magFSR(magRange), odr(odr)
{
}

bool LSM9DS1_M::init()
{

#ifdef DEBUG
    assert(sensor_initialized == false);
#endif

    SPITransaction spi(spislave);

    // Who Am I check:
    uint8_t whoami = spi.read(regMapM::WHO_AM_I_M);

    if (whoami != WHO_AM_I_M_VAL)
    {
        TRACE("[LSM9DS1 MAG] init() : unexpected WAMI -> %02X\n", whoami);
        last_error = ERR_NOT_ME;
        return false;
    }

    // X,Y axes in ultra-high performance mode, ODR defined by constructor
    uint8_t CTRL_REG1_M_VAL = 0x60 | odr << 2;
    spi.write(regMapM::CTRL_REG1_M, CTRL_REG1_M_VAL);

    // FSR defined by constructor
    uint8_t CTRL_REG2_M_VAL = magFSR << 5;
    spi.write(regMapM::CTRL_REG2_M, CTRL_REG2_M_VAL);

    // Z axis in ultra-high performance mode
    spi.write(regMapM::CTRL_REG4_M, CTRL_REG4_M_VAL);

    // I2C disabled, SPI mode: read/write
    spi.write(regMapM::CTRL_REG3_M, CTRL_REG3_M_VAL);

    // disable all interrupts
    spi.write(regMapM::INT_CFG_M, INT_CFG_M_VAL);

    // check that all registers have been written correctly

    if (spi.read(regMapM::CTRL_REG1_M) != CTRL_REG1_M_VAL)
    {
        TRACE("[LSM9DS1 MAG] init() : CTRL_REG1_M readback failed\n");
        return false;
    }
    if (spi.read(regMapM::CTRL_REG2_M) != CTRL_REG2_M_VAL)
    {
        TRACE("[LSM9DS1 MAG] init() : CTRL_REG2_M readback failed\n");
        return false;
    }
    if (spi.read(regMapM::CTRL_REG3_M) != CTRL_REG3_M_VAL)
    {
        TRACE("[LSM9DS1 MAG] init() : CTRL_REG3_M readback failed\n");
        return false;
    }
    if (spi.read(regMapM::CTRL_REG4_M) != CTRL_REG4_M_VAL)
    {
        TRACE("[LSM9DS1 MAG] init() : CTRL_REG4_M readback failed\n");
        return false;
    }
    if (spi.read(regMapM::INT_CFG_M) != INT_CFG_M_VAL)
    {
        TRACE("[LSM9DS1 MAG] init() : INT_CFG_M readback failed\n");
        return false;
    }

    // wait 20ms for stable output
    miosix::Thread::sleep(20);

    TRACE("[LSM9DS1 MAG] init() : done\n");

    sensor_initialized = true;

    return true;
}

bool LSM9DS1_M::selfTest()
{
    TRACE("[LSM9DS1 MAG] selfTest() : starting self-test\n");

    Stats nostX, nostY, nostZ;
    Stats stX, stY, stZ;
    bool selfTestResult = false;

    nostX.reset();
    nostY.reset();
    nostZ.reset();

    selfTest_mode = true;

    // if already initialized
    if (sensor_initialized == true)
    {
        TRACE("[LSM9DS1 MAG] selfTest() : sensor already initialized!\n");
#ifdef DEBUG
        assert(sensor_initialized == false);
#endif
        return false;
    }

    {
        SPITransaction spi(spislave);

        // Reset all registers
        spi.write(regMapM::CTRL_REG2_M, SOFT_RESET);
        miosix::Thread::sleep(20);

        // self-test ROUTINE of LIS3MDL - seems to be same magnetometer as
        // LSM9DS1. init sensor for self-test: FSR = +/-12 Gauss, ODR = 80Hz
        uint8_t CTRL_REG1_M_VAL = (ODR::ODR_80 << 2);
        spi.write(regMapM::CTRL_REG1_M, CTRL_REG1_M_VAL);

        uint8_t CTRL_REG2_M_VAL = MagFSR::FS_12 << 5;
        spi.write(regMapM::CTRL_REG2_M, CTRL_REG2_M_VAL);

        // wait 20ms for stable output
        miosix::Thread::sleep(20);

        // out enable - continuous mode
        spi.write(regMapM::CTRL_REG3_M, 0x00);

        // wait data-ready bit on STATUS REG - read first sample and discard
        while ((spi.read(regMapM::STATUS_REG_M) & DRDY_MASK) == 0)
        {
        }

        uint8_t data[6];
        spi.read(regMapM::OUT_X_L_M | AUTO_INCREMENT_ADDR, data, 6);
        UNUSED(data);
    }

    // wait data-ready bit on STATUS REG - read NOST sample at least 5 times
    LSM9DS1_M::getSelfTestData(nostX, nostY, nostZ);

    // enable self-test mode and wait 60ms
    {
        SPITransaction spi(spislave);

        uint8_t CTRL_REG1_M_VAL = (ODR::ODR_80 << 2) | SELFTEST_ENABLE;
        spi.write(regMapM::CTRL_REG1_M, CTRL_REG1_M_VAL);

        miosix::Thread::sleep(60);

        // wait data-ready bit on STATUS REG - read first sample and discard
        while ((spi.read(regMapM::STATUS_REG_M) & DRDY_MASK) == 0)
        {
        }

        uint8_t data[6];
        spi.read(regMapM::OUT_X_L_M | AUTO_INCREMENT_ADDR, 6);
        UNUSED(data);
    }

    LSM9DS1_M::getSelfTestData(stX, stY, stZ);

    float deltaX = fabsf(stX.getStats().mean - nostX.getStats().mean);
    float deltaY = fabsf(stY.getStats().mean - nostY.getStats().mean);
    float deltaZ = fabsf(stZ.getStats().mean - nostZ.getStats().mean);

    // verify if sensor is inside parameters

    // print stats
#ifdef DEBUG
    std::cout << "[LSM9DS1 MAG] selfTest() : statistics"
              << "\n"
              << "X-AXIS stats : " << nostX.getStats() << "\n"
              << "Y-AXIS stats : " << nostY.getStats() << "\n"
              << "Z-AXIS stats : " << nostZ.getStats() << "\n"
              << "deltaX : " << deltaX << "\n"
              << "deltaY : " << deltaY << "\n"
              << "deltaZ : " << deltaZ << "\n";
#endif

    // clang-format off

    if(ST_XY_MIN < deltaX && deltaX < ST_XY_MAX &&
       ST_XY_MIN < deltaY && deltaY < ST_XY_MAX &&
       ST_Z_MIN  < deltaZ && deltaZ < ST_Z_MAX)
    {
        TRACE("[LSM9DS1 MAG] selfTest() : self-test passed\n");
        selfTestResult = true;
    }
    else
    {
        TRACE("[LSM9DS1 MAG] selfTest() : self-test failed\n");    
        selfTestResult = false;
    }

    // clang-format on 
    {
        SPITransaction spi(spislave);
        spi.write(regMapM::CTRL_REG2_M, SOFT_RESET);
        miosix::Thread::sleep(20);
    }
    selfTest_mode = false;    
    
    return selfTestResult;
}

void LSM9DS1_M::getSelfTestData(Stats& outxStats, Stats& outyStats,
                                Stats& outzStats)
{
    for (int i = 0; i < SELFTEST_MAX_SAMPLES; i++)
    {
        {
            SPITransaction spi(spislave);
            while ((spi.read(regMapM::STATUS_REG_M) & DRDY_MASK) == 0)
            {
            }
        }
        LSM9DS1_M::onSimpleUpdate();

        // compute statistics
        outxStats.add(lastMagneto.magData.getX());
        outyStats.add(lastMagneto.magData.getY());
        outzStats.add(lastMagneto.magData.getZ());
    }
}

bool LSM9DS1_M::onSimpleUpdate()
{
#ifdef DEBUG
    assert(sensor_initialized == true || selfTest_mode == true);
#endif

    uint8_t magData[6];

    // read output magneto raw data X,Y,Z and timestamp
    {
        SPITransaction spi(spislave);
        spi.read(regMapM::OUT_X_L_M | AUTO_INCREMENT_ADDR, magData, 6);
    }

    lastMagneto.timestamp = miosix::getTick();

    // compose signed 16-bit raw data as 2 bytes from the sensor
    // clang-format off
    int16_t x = magData[0] | magData[1] << 8;
    int16_t y = magData[2] | magData[3] << 8;
    int16_t z = magData[4] | magData[5] << 8;

    //convert raw data
    lastMagneto.magData = Vec3(x * magFSR_SMap.at(magFSR), 
                               y * magFSR_SMap.at(magFSR),
                               z * magFSR_SMap.at(magFSR));
    // clang-format on

    return true;
}

const lsm9ds1MSample& LSM9DS1_M::getSample() const { return lastMagneto; }