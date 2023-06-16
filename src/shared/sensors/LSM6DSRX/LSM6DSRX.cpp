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

#include <cmath>

namespace Boardcore
{

LSM6DSRX::LSM6DSRX(SPIBus& bus, miosix::GpioPin csPin,
                   SPIBusConfig busConfiguration, LSM6DSRXConfig& configuration)
    : m_spiSlave(bus, csPin, busConfiguration), m_config(configuration)
{
    switch (m_config.fsAcc)  // pag. 10
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

    switch (m_config.fsGyr)
    {
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_125:
            m_sensitivityGyr = 4.375;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_250:
            m_sensitivityGyr = 8.75;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_500:
            m_sensitivityGyr = 17.5;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_1000:
            m_sensitivityGyr = 35.0;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000:
            m_sensitivityGyr = 70.0;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_4000:
            m_sensitivityGyr = 140.0;
            break;
        default:
            m_config.fsGyr   = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
            m_sensitivityGyr = 4.375;
            break;
    }

    m_isInit = false;
}

bool LSM6DSRX::init()
{
#ifdef DEBUG
    assert(!m_isInit && "init() should be called once");
#endif

    if (checkWhoAmI() == false)
    {
        return false;
    }

    // set BDU pag. 54
    {
        SPITransaction spiTransaction{m_spiSlave};
        spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL3_C,
                                     static_cast<uint8_t>(m_config.bdu));
    }

    // Setup accelerometer (pag. 28)
    if (!initAccelerometer())
    {
        return false;
    }

    // Setup gyroscope (pag. 28)
    if (!initGyroscope())
    {
        return false;
    }

    // setup Fifo (pag. 33)
    if (!initFifo())
    {
        return false;
    }

    // enable timestamp (pag. 61 datasheet)
    {
        SPITransaction spiTransaction{m_spiSlave};
        spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL10_C, 1 << 5);
    }

    // set interrupt
    initInterrupts();

    m_isInit = true;
    return true;
}

bool LSM6DSRX::initAccelerometer()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{m_spiSlave};

    // Setup accelerometer (pag. 28)

    // set accelerometer odr, fullscale and high resolution (pag. 52)
    configByte = static_cast<uint8_t>(m_config.odrAcc) << 4 |  // odr
                 static_cast<uint8_t>(m_config.fsAcc) << 2 |   // fullscale
                 0 << 1;  // high resolution selection
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL1_XL, configByte);

    // set accelerometer performance mode (pag. 57)
    configByte = static_cast<uint8_t>(m_config.opModeAcc) << 4;
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL6_C, configByte);

    return true;
}

bool LSM6DSRX::initGyroscope()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{m_spiSlave};

    // setup pag. 28

    // set odr, fullscale pag. 53
    configByte = static_cast<uint8_t>(m_config.odrGyr) << 4 |  // odr
                 static_cast<uint8_t>(m_config.fsGyr);         // fullscale
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL2_G, configByte);

    // set performance mode pag. 58
    configByte = static_cast<uint8_t>(m_config.opModeGyr) << 7;
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL7_G, configByte);

    return true;
}

bool LSM6DSRX::initFifo()
{
    // setup Fifo (pag. 33)
    uint8_t configByte = 0;
    SPITransaction spiTransaction{m_spiSlave};

    // select batch data rate in FIFO_CTRL3
    configByte = static_cast<uint8_t>(m_config.odrAcc) |  // accelerometer bdr
                 (static_cast<uint8_t>(m_config.odrGyr) << 4);  // gyroscope bdr
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_FIFO_CTRL3, configByte);

    // set fifo mode, batch data rate for temperature sensor and the decimation
    // factor for timestamp batching
    configByte =
        static_cast<uint8_t>(m_config.fifoMode) |  // fifo operating mode
        static_cast<uint8_t>(m_config.fifoTemperatureBdr)
            << 4 |  // batch data rate for temperature data
        static_cast<uint8_t>(m_config.fifoTimestampDecimation)
            << 6;  // timestamp decimation
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_FIFO_CTRL4, configByte);

    return true;
}

void LSM6DSRX::initInterrupts()
{
    uint8_t ui8Value = 0;
    SPITransaction spi{m_spiSlave};

    // set interrupt on pin INT1
    spi.writeRegister(LSM6DSRXDefs::REG_INT1_CTRL,
                      static_cast<uint8_t>(m_config.int1InterruptSelection));
    // set interrupt on pin INT2
    spi.writeRegister(LSM6DSRXDefs::REG_INT2_CTRL,
                      static_cast<uint8_t>(m_config.int2InterruptSelection));

    // set watermark level
    ui8Value = static_cast<uint8_t>(m_config.fifoWatermark & 255);
    spi.writeRegister(LSM6DSRXDefs::REG_FIFO_CTRL1, ui8Value);
    ui8Value = static_cast<uint8_t>(m_config.fifoWatermark & 768);
    spi.writeRegister(LSM6DSRXDefs::REG_FIFO_CTRL2, ui8Value);
}

bool LSM6DSRX::selfTestAcc()
{
    bool returnValue        = false;
    uint8_t byteValue       = 0;  // used to read and write in registers
    uint8_t idx             = 0;
    const uint8_t SIZE_DATA = 5;  // s
    SPITransaction spi{m_spiSlave};

    SensorData averageSF{0.0, 0.0, 0.0};      // average data during self test
    SensorData averageNormal{0.0, 0.0, 0.0};  // average normal data

    // set registers
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL1_XL, 0x38);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL2_G, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL3_C, 0x44);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL4_C, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL6_C, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL7_G, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL8_XL, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL9_XL, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL10_C, 0x00);

    // wait for stable output
    miosix::Thread::sleep(100);

    // wait for accelerometer data ready
    byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
    byteValue = byteValue & 0x01;
    while (byteValue != 1)
    {
        miosix::Thread::sleep(50);
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = byteValue & 0x01;
    }

    // read and discard data
    getAxisData(LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A);
    getAxisData(LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A);
    getAxisData(LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A);

    // read normal data (self test disabled)
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        // wait for accelerometer data ready
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = byteValue & 0x01;
        while (byteValue != 1)
        {
            miosix::Thread::sleep(50);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = byteValue & 0x01;
        }

        // read data
        averageNormal.x += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A));
        averageNormal.y += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A));
        averageNormal.z += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A));
    }
    averageNormal.x /= SIZE_DATA;
    averageNormal.y /= SIZE_DATA;
    averageNormal.z /= SIZE_DATA;

    // enable accelerometer self test
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x01);

    // wait for stable output
    miosix::Thread::sleep(100);

    // wait for accelerometer data ready
    byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
    byteValue = byteValue & 0x01;
    while (byteValue != 1)
    {
        miosix::Thread::sleep(50);
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = byteValue & 0x01;
    }

    // read and discard data
    getAxisData(LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A);
    getAxisData(LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A);
    getAxisData(LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A);

    // read self test data
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        // wait for accelerometer data ready
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = byteValue & 0x01;
        while (byteValue != 1)
        {
            miosix::Thread::sleep(50);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = byteValue & 0x01;
        }

        // read data
        averageSF.x += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A));
        averageSF.y += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A));
        averageSF.z += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A));
    }
    averageSF.x /= SIZE_DATA;
    averageSF.y /= SIZE_DATA;
    averageSF.z /= SIZE_DATA;

    if (LSM6DSRXDefs::ACC_SELF_TEST_MIN <=
            std::abs(averageSF.x - averageNormal.x) &&
        std::abs(averageSF.x - averageNormal.x) <=
            LSM6DSRXDefs::ACC_SELF_TEST_MAX &&
        LSM6DSRXDefs::ACC_SELF_TEST_MIN <=
            std::abs(averageSF.y - averageNormal.y) &&
        std::abs(averageSF.y - averageNormal.y) <=
            LSM6DSRXDefs::ACC_SELF_TEST_MAX &&
        LSM6DSRXDefs::ACC_SELF_TEST_MIN <=
            std::abs(averageSF.z - averageNormal.z) &&
        std::abs(averageSF.z - averageNormal.z) <=
            LSM6DSRXDefs::ACC_SELF_TEST_MAX)
    {
        returnValue = true;
    }
    else
    {
        returnValue = false;
    }

    // Disable self-test
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x00);
    // Disable sensor
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL1_XL, 0x00);

    return returnValue;
}

bool LSM6DSRX::selfTestGyr()
{
    bool returnValue        = true;
    uint8_t byteValue       = 0;
    uint8_t idx             = 0;
    const uint8_t SIZE_DATA = 5;
    SPITransaction spi{m_spiSlave};

    SensorData averageSF{0.0, 0.0, 0.0};      // average data during self test
    SensorData averageNormal{0.0, 0.0, 0.0};  // average normal data

    // Initialize and turn on sensor
    // Set BDU = 1, ODR = 208 Hz, FS = +-2000 dps
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL1_XL, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL2_G, 0x5C);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL3_C, 0x44);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL4_C, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL6_C, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL7_G, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL8_XL, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL9_XL, 0x00);
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL10_C, 0x00);

    // sleep for stable output
    miosix::Thread::sleep(100);

    // wait for gyroscope data ready
    byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
    byteValue = (byteValue & 0x02) >> 1;
    while (byteValue != 1)
    {
        miosix::Thread::sleep(50);
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = (byteValue & 0x02) >> 1;
    }
    // read and discard data
    getAxisData(LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G);
    getAxisData(LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G);
    getAxisData(LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G);

    // read normal data (self test disabled)
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        // wait for gyroscope data ready
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = (byteValue & 0x02) >> 1;
        while (byteValue != 1)
        {
            miosix::Thread::sleep(50);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = (byteValue & 0x02) >> 1;
        }

        // read data
        averageNormal.x += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G));
        averageNormal.y += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G));
        averageNormal.z += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G));
    }
    averageNormal.x /= SIZE_DATA;
    averageNormal.y /= SIZE_DATA;
    averageNormal.z /= SIZE_DATA;

    // enable gyroscope self test
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x04);

    // wait for stable output
    miosix::Thread::sleep(100);

    // wait for gyroscope data ready
    byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
    byteValue = (byteValue & 0x02) >> 1;
    while (byteValue != 1)
    {
        miosix::Thread::sleep(50);
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = (byteValue & 0x02) >> 1;
    }
    // read and discard data
    getAxisData(LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G);
    getAxisData(LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G);
    getAxisData(LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G);

    // read self test data
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        // wait for gyroscope data ready
        byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
        byteValue = (byteValue & 0x02) >> 1;
        while (byteValue != 1)
        {
            miosix::Thread::sleep(50);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = (byteValue & 0x02) >> 1;
        }

        // read data
        averageSF.x += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G));
        averageSF.y += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G));
        averageSF.z += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G));
    }
    averageSF.x /= SIZE_DATA;
    averageSF.y /= SIZE_DATA;
    averageSF.z /= SIZE_DATA;

    if (LSM6DSRXDefs::GYR_SELF_TEST_MIN <=
            std::abs(averageSF.x - averageNormal.x) &&
        std::abs(averageSF.x - averageNormal.x) <=
            LSM6DSRXDefs::GYR_SELF_TEST_MAX &&
        LSM6DSRXDefs::GYR_SELF_TEST_MIN <=
            std::abs(averageSF.y - averageNormal.y) &&
        std::abs(averageSF.y - averageNormal.y) <=
            LSM6DSRXDefs::GYR_SELF_TEST_MAX &&
        LSM6DSRXDefs::GYR_SELF_TEST_MIN <=
            std::abs(averageSF.z - averageNormal.z) &&
        std::abs(averageSF.z - averageNormal.z) <=
            LSM6DSRXDefs::GYR_SELF_TEST_MAX)
    {
        returnValue = true;
    }
    else
    {
        returnValue = false;
    }

    // Disable self test
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x00);
    // Disable sensor
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL2_G, 0x00);

    return returnValue;
}

bool LSM6DSRX::checkWhoAmI()
{
    uint8_t regValue = 0;
    {
        SPITransaction transaction{m_spiSlave};
        regValue = transaction.readRegister(LSM6DSRXDefs::REG_WHO_AM_I);
    }

    return regValue == LSM6DSRXDefs::WHO_AM_I_VALUE;
}

int16_t LSM6DSRX::combineHighLowBits(uint8_t low, uint8_t high)
{
    int16_t ret = high;
    ret <<= 8;
    ret |= low;
    return ret;
}

void LSM6DSRX::getAccelerometerData(SensorData& data)
{
#ifdef DEBUG
    assert(m_isInit && "init() was not called");
#endif

    data.x = getAxisData(LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A,
                         m_sensitivityAcc);
    data.y = getAxisData(LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A,
                         m_sensitivityAcc);
    data.z = getAxisData(LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A,
                         m_sensitivityAcc);
}

void LSM6DSRX::getGyroscopeData(SensorData& data)
{
#ifdef DEBUG
    assert(m_isInit && "init() was not called");
#endif

    data.x = getAxisData(LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G,
                         m_sensitivityGyr);
    data.y = getAxisData(LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G,
                         m_sensitivityGyr);
    data.z = getAxisData(LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G,
                         m_sensitivityGyr);
}

uint32_t LSM6DSRX::getSensorTimestamp()
{
#ifdef DEBUG
    assert(m_isInit && "init() was not called");
#endif

    SPITransaction spi{m_spiSlave};

    uint32_t value = spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP0);
    value |= spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP1) << 8;
    value |= spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP2) << 16;
    value |= spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP3) << 24;

    return value;
}

bool LSM6DSRX::selfTest()
{
#ifdef DEBUG
    assert(m_isInit && "init() was not called");
#endif

    m_isInit = false;

    if (!selfTestAcc() || !selfTestGyr())
    {
        return false;
    }

    return init();
}

float LSM6DSRX::getAxisData(LSM6DSRXDefs::Registers lowReg,
                            LSM6DSRXDefs::Registers highReg, float sensitivity)
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