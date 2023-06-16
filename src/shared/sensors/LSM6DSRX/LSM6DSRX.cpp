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

#include "LSM6DSRX.h"

#include <assert.h>
#include <utils/Debug.h>

namespace Boardcore
{

LSM6DSRX::LSM6DSRX(SPIBus& bus, miosix::GpioPin csPin,
                   SPIBusConfig busConfiguration, BDU blockDataUpdate,
                   ACC_ODR odrAccelerometer, OPERATING_MODE opModeAccelerometer,
                   ACC_FULLSCALE fsAccelerator)
    : spiSlave(bus, csPin, busConfiguration), bdu(blockDataUpdate),
      odrAcc(odrAccelerometer), opModeAcc(opModeAccelerometer),
      fsAcc(fsAccelerator)
{
    switch (fsAcc)
    {
        case ACC_FULLSCALE::G2:
            sensitivityAcc = 0.061;
            break;
        case ACC_FULLSCALE::G4:
            sensitivityAcc = 0.122;
            break;
        case ACC_FULLSCALE::G8:
            sensitivityAcc = 0.244;
            break;
        case ACC_FULLSCALE::G16:
            sensitivityAcc = 0.488;
            break;
        default:
            fsAcc          = ACC_FULLSCALE::G2;
            sensitivityAcc = 0.061;
            break;
    };
}

bool LSM6DSRX::init()
{
    if (checkWhoAmI() == false)
    {
        return false;
    }

    SPITransaction spiTransaction{spiSlave};

    // set BDU pag. 54
    spiTransaction.writeRegister(REG_CTRL3_C, static_cast<uint8_t>(bdu));

    // Setup accelerometer (pag. 28)

    // set accelerometer odr (pag. 52)
    uint8_t accSetup = static_cast<uint8_t>(odrAcc)
                       << 4  // odr
                             // full scale (defaul = 00 --> +-2g)
        ;                    // high resolution selection (default)
    spiTransaction.writeRegister(REG_CTRL1_XL, accSetup);

    // set accelerometer performance mode (pag. 57)
    // high performance disabled
    uint8_t accPerformanceMode = static_cast<uint8_t>(opModeAcc) << 4;
    spiTransaction.writeRegister(REG_CTRL6_C, accPerformanceMode);

    // setup Fifo (pag. 33)
    // set fifo mode: bypass_mode = 0 (pag. 48)
    uint8_t fifoMode = 0;
    spiTransaction.writeRegister(REG_FIFO_CTRL4, fifoMode);

    return true;
}

bool LSM6DSRX::checkWhoAmI()
{
    uint8_t regValue = 0;
    {
        SPITransaction transaction{spiSlave};
        regValue = transaction.readRegister(Registers::REG_WHO_AM_I);
    }

    return regValue == WHO_AM_I_VALUE;
}

int16_t LSM6DSRX::combineHighLowBits(uint8_t low, uint8_t high)
{
    int16_t ret = high;
    ret <<= 8;
    ret |= low;
    return ret;
}

void LSM6DSRX::getAccelerometerData(AccData& data)
{
    // #ifdef DEBUG    // NOT WORKING
    //     assert(isInit && "init() was not called");  // linter off
    // #endif

    data.x = getAxisData(REG_OUTX_L_A, REG_OUTX_H_A);
    data.y = getAxisData(REG_OUTY_L_A, REG_OUTY_H_A);
    data.z = getAxisData(REG_OUTZ_L_A, REG_OUTZ_H_A);
}

float LSM6DSRX::getAxisData(Registers lowReg, Registers highReg)
{
    int8_t low = 0, high = 0;
    int16_t sample = 0;

    SPITransaction transaction{spiSlave};

    high = transaction.readRegister(highReg);
    low  = transaction.readRegister(lowReg);

    sample = combineHighLowBits(low, high);

    float ret = static_cast<float>(sample) * sensitivityAcc;
    return ret;
}

}  // namespace Boardcore