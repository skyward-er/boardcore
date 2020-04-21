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
    if (sensor_initialized)
    {
        TRACE("[LSM9DS1 MAG] init() : already initialized\n");
        return false;
    }

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
    uint8_t CTRL_REG1_M_VAL = 0x60 | (int)odr << 2;
    spi.write(regMapM::CTRL_REG1_M, CTRL_REG1_M_VAL);

    // FSR defined by constructor
    uint8_t CTRL_REG2_M_VAL = (int)magFSR << 5;
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
        return false;
    }
    if (spi.read(regMapM::CTRL_REG2_M) != CTRL_REG2_M_VAL)
    {
        return false;
    }
    if (spi.read(regMapM::CTRL_REG3_M) != CTRL_REG3_M_VAL)
    {
        return false;
    }
    if (spi.read(regMapM::CTRL_REG4_M) != CTRL_REG4_M_VAL)
    {
        return false;
    }
    if (spi.read(regMapM::INT_CFG_M) != INT_CFG_M_VAL)
    {
        return false;
    }

    // select Sensitivity
    switch (magFSR)
    {
        case MagFSR::FS_4:
            magSensitivity = 0.14f;
            break;
        case MagFSR::FS_8:
            magSensitivity = 0.29f;
            break;
        case MagFSR::FS_12:
            magSensitivity = 0.43f;
            break;
        case MagFSR::FS_16:
            magSensitivity = 0.58f;
            break;
        default:
            magSensitivity = 0.14f;
            break;
    }

    TRACE("[LSM9DS1 XLG] init() : done\n");

    sensor_initialized = true;

    return true;
}

bool LSM9DS1_M::selfTest() { return true; }

bool LSM9DS1_M::onSimpleUpdate()
{

    uint8_t magData[6];

    // read output magneto raw data X,Y,Z
    {
        SPITransaction spi(spislave);
        // bit 1 of SPI transaction = 1 means "auto-increment address"
        spi.read(regMapM::OUT_X_L_M | 0x40, magData, 6);
    }

    // compose signed 16-bit raw data as 2 bytes from the sensor
    // clang-format off
    int16_t x = magData[0] | magData[1] << 8;
    int16_t y = magData[2] | magData[3] << 8;
    int16_t z = magData[4] | magData[5] << 8;

    //convert raw data
    mLastCompass = Vec3(x * magSensitivity / 1000, 
                        y * magSensitivity / 1000,
                        z * magSensitivity / 1000);
    // clang-format on

    return true;
}

bool LSM9DS1_M::setOffset(vector<uint16_t>& offVect)
{
    if (offVect.size() != 3)
        return false;

    uint8_t toStore[6];
    
    //separate each byte (MSB first)
    for (int i = 6; i > 0; i = i - 2)
    {
        toStore[i - 1] = offVect.back() & 0x00FF;  // LSB
        toStore[i - 2] = offVect.back() >> 8;      // MSB
        offVect.pop_back();
    }

    SPITransaction spi(spislave);
    // bit 1 of SPI transaction = 1 means "auto-increment address".
    spi.write(regMapM::OFFSET_X_REG_L_M | 0x40, toStore, 6);
}