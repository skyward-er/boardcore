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
                   SPIBusConfig busConfiguration, LSM6DSRXConfig& configuration)
    : m_spiSlave(bus, csPin, busConfiguration), m_config(configuration)
{
    switch (m_config.fsAcc)// pag. 10
    {
        case LSM6DSRXConfig::ACC_FULLSCALE::G2:
            m_sensitivityAcc = 0.061;
            break;
        case LSM6DSRXConfig::ACC_FULLSCALE::G4:
            m_sensitivityAcc = 0.122;
            break;
        case LSM6DSRXConfig::ACC_FULLSCALE::G8:
            m_sensitivityAcc = 0.244;
            break;
        case LSM6DSRXConfig::ACC_FULLSCALE::G16:
            m_sensitivityAcc = 0.488;
            break;
        default:
            m_config.fsAcc   = LSM6DSRXConfig::ACC_FULLSCALE::G2;
            m_sensitivityAcc = 0.061;
            break;
    };
}

bool LSM6DSRX::init()
{
    if (checkWhoAmI() == false)
    {
        return false;
    }

    // set BDU pag. 54
    {
        SPITransaction spiTransaction{m_spiSlave};
        spiTransaction.writeRegister(REG_CTRL3_C,
                                 static_cast<uint8_t>(m_config.bdu));
    }

    // Setup accelerometer (pag. 28)
    if(!initAccelerometer())
    {
        return false;
    }

    // Setup gyroscope (pag. 28)
    if(!initGyroscope())
    {
        return false;
    }

    // setup Fifo (pag. 33)
    if(!initFifo())
    {
        return false;
    }

    return true;
}

bool LSM6DSRX::initAccelerometer()
{
    SPITransaction spiTransaction{m_spiSlave};

    // Setup accelerometer (pag. 28)

    // set accelerometer odr (pag. 52)
    uint8_t accSetup = static_cast<uint8_t>(m_config.odrAcc) << 4 |  // odr
                       static_cast<uint8_t>(m_config.fsAcc) << 2 |  // fullscale
                       0 << 1;  // high resolution selection
    spiTransaction.writeRegister(REG_CTRL1_XL, accSetup);

    // set accelerometer performance mode (pag. 57)
    uint8_t accPerformanceMode = static_cast<uint8_t>(m_config.opModeAcc) << 4;
    spiTransaction.writeRegister(REG_CTRL6_C, accPerformanceMode);

    return true;
}

bool LSM6DSRX::initGyroscope()
{
    SPITransaction spiTransaction{m_spiSlave};

    // setup pag. 28

    // set odr, fullscale pag. 53
    uint8_t gyrSetup = 1 << 6 | 0 << 2; // odr: 104 Hz | fullscale: +250dps
    spiTransaction.writeRegister(REG_CTRL2_G, gyrSetup);
    // warning: lowest and highest fullscale (125 & 4000 dps) are selectable from a different bit

    // set performance mode pag. 58
    // for now: normal mode
    uint8_t gyrPerformanceMode = 1 << 7;
    spiTransaction.writeRegister(REG_CTRL7_G, gyrPerformanceMode);

    return true;
}

bool LSM6DSRX::initFifo()
{
    // setup Fifo (pag. 33)
    SPITransaction spiTransaction{m_spiSlave};

    // set fifo mode: bypass_mode = 0 (pag. 48)
    uint8_t fifoMode = 0;
    spiTransaction.writeRegister(REG_FIFO_CTRL4, fifoMode);

    return true;
}

bool LSM6DSRX::checkWhoAmI()
{
    uint8_t regValue = 0;
    {
        SPITransaction transaction{m_spiSlave};
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

    data.x = getAxisData(REG_OUTX_L_A, REG_OUTX_H_A, m_sensitivityAcc);
    data.y = getAxisData(REG_OUTY_L_A, REG_OUTY_H_A, m_sensitivityAcc);
    data.z = getAxisData(REG_OUTZ_L_A, REG_OUTZ_H_A, m_sensitivityAcc);
}

float LSM6DSRX::getAxisData(Registers lowReg, Registers highReg, float sensitivity)
{
    int8_t low = 0, high = 0;
    int16_t sample = 0;

    SPITransaction transaction{m_spiSlave};

    high = transaction.readRegister(highReg);
    low  = transaction.readRegister(lowReg);

    sample = combineHighLowBits(low, high);

    float ret = static_cast<float>(sample) * sensitivity;
    return ret;
}

}  // namespace Boardcore