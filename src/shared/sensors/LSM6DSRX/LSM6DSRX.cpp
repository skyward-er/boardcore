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

#include "LSM6DSRX.h"

#include <assert.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/Debug.h>

#include <cmath>

namespace Boardcore
{

LSM6DSRX::LSM6DSRX(SPIBus& bus, miosix::GpioPin csPin,
                   SPIBusConfig busConfiguration, LSM6DSRXConfig& configuration)
    : spiSlave(bus, csPin, busConfiguration), config(configuration)
{
    isInit = false;

    // check that the watermark value is suitable (it should be a 9bits number)
    config.fifoWatermark = std::min(config.fifoWatermark, uint16_t(511));

    // check that ACC_ODR is set to HZ_1_6 only if OPERATING_MODE is equal to
    // NORMAL
    if (config.odrAcc == LSM6DSRXConfig::ACC_ODR::HZ_1_6)
    {
        config.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;
    }
}

bool LSM6DSRX::init()
{
    if (isInit)
    {
        LOG_ERR(logger, "init() should be called once");
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    if (checkWhoAmI() == false)
    {
        LOG_ERR(logger, "Got bad CHIPID");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    // Set BDU and multiple spi read/write
    {
        SPITransaction spiTransaction{spiSlave};

        uint8_t value = static_cast<uint8_t>(config.bdu) << 6;  // set bdu
        value |= 1 << 2;  // set multiple spi read/write

        spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL3_C, value);
    }

    // Setup accelerometer
    initAccelerometer();

    // Setup gyroscope
    initGyroscope();

    // setup Fifo
    initFifo();

    // enable timestamp
    {
        constexpr uint8_t REG_CTRL10_TIMESTAMP_EN = 1 << 5;

        SPITransaction spiTransaction{spiSlave};
        spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL10_C,
                                     REG_CTRL10_TIMESTAMP_EN);
    }

    // set interrupt
    initInterrupts();

    // set timestamps
    correlateTimestamps();
    sensorTimestampResolution =
        getSensorTimestampResolution() *
        1000;  // return value is in milliseconds, need microseconds.

    isInit    = true;
    lastError = SensorErrors::NO_ERRORS;
    return true;
}

void LSM6DSRX::initAccelerometer()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // Setup accelerometer

    // set accelerometer odr, fullscale and high resolution
    configByte = static_cast<uint8_t>(config.odrAcc) << 4 |  // odr
                 static_cast<uint8_t>(config.fsAcc) << 2 |   // fullscale
                 0 << 1;  // high resolution selection
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL1_XL, configByte);

    // set accelerometer performance mode
    configByte = static_cast<uint8_t>(config.opModeAcc) << 4;
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL6_C, configByte);

    // set sensitivity
    switch (config.fsAcc)
    {
        case LSM6DSRXConfig::ACC_FULLSCALE::G2:
            sensitivityAcc = 0.061;
            break;
        case LSM6DSRXConfig::ACC_FULLSCALE::G4:
            sensitivityAcc = 0.122;
            break;
        case LSM6DSRXConfig::ACC_FULLSCALE::G8:
            sensitivityAcc = 0.244;
            break;
        case LSM6DSRXConfig::ACC_FULLSCALE::G16:
            sensitivityAcc = 0.488;
            break;
        default:
            config.fsAcc   = LSM6DSRXConfig::ACC_FULLSCALE::G2;
            sensitivityAcc = 0.061;
            break;
    };
}

void LSM6DSRX::initGyroscope()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // set odr and fullscale
    configByte = static_cast<uint8_t>(config.odrGyr) << 4 |  // odr
                 static_cast<uint8_t>(config.fsGyr);         // fullscale
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL2_G, configByte);

    // set performance mode
    configByte = static_cast<uint8_t>(config.opModeGyr) << 7;
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_CTRL7_G, configByte);

    // set sensitivity
    switch (config.fsGyr)
    {
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_125:
            sensitivityGyr = 4.375;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_250:
            sensitivityGyr = 8.75;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_500:
            sensitivityGyr = 17.5;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_1000:
            sensitivityGyr = 35.0;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000:
            sensitivityGyr = 70.0;
            break;
        case LSM6DSRXConfig::GYR_FULLSCALE::DPS_4000:
            sensitivityGyr = 140.0;
            break;
        default:
            config.fsGyr   = LSM6DSRXConfig::GYR_FULLSCALE::DPS_125;
            sensitivityGyr = 4.375;
            break;
    }
}

void LSM6DSRX::initFifo()
{
    // setup Fifo
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // select batch data rate in FIFO_CTRL3
    configByte = static_cast<uint8_t>(config.odrAcc) |  // accelerometer bdr
                 (static_cast<uint8_t>(config.odrGyr) << 4);  // gyroscope bdr
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_FIFO_CTRL3, configByte);

    // set fifo mode, batch data rate for temperature sensor and the decimation
    // factor for timestamp batching
    configByte = static_cast<uint8_t>(config.fifoMode) |  // fifo operating mode
                 static_cast<uint8_t>(config.fifoTemperatureBdr)
                     << 4 |  // batch data rate for temperature data
                 static_cast<uint8_t>(config.fifoTimestampDecimation)
                     << 6;  // timestamp decimation
    spiTransaction.writeRegister(LSM6DSRXDefs::REG_FIFO_CTRL4, configByte);
}

void LSM6DSRX::initInterrupts()
{
    uint8_t buf[] = {0, 0};
    SPITransaction spi{spiSlave};

    buf[0] = static_cast<uint8_t>(
        config.int1InterruptSelection);  // set interrupt on pin INT1
    buf[1] = static_cast<uint8_t>(
        config.int2InterruptSelection);  // set interrupt on pin INT2
    spi.writeRegisters(LSM6DSRXDefs::REG_INT1_CTRL, buf, 2);

    // set watermark level
    buf[0] = static_cast<uint8_t>(config.fifoWatermark &
                                  255);  // the first 8bits of the number.
    buf[1] = static_cast<uint8_t>((config.fifoWatermark >> 8) &
                                  0x01);  // the 9th bit of the number.
    spi.writeRegisters(LSM6DSRXDefs::REG_FIFO_CTRL1, buf, 2);
}

bool LSM6DSRX::selfTestAcc()
{
    bool returnValue        = false;
    uint8_t byteValue       = 0;  // used to read and write in registers
    uint8_t idx             = 0;
    const uint8_t SIZE_DATA = 5;  // number of sample for the test
    SPITransaction spi{spiSlave};

    // sleep time for data ready interrupt (150% odr during self-test)
    // expressed in milliseconds.
    // for this self-test the odr is 52Hz
    const unsigned int dataReadyWaitTime = 29;

    LSM6DSRXData averageSF;      // average data during self test
    LSM6DSRXData averageNormal;  // average normal data

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
        miosix::Thread::sleep(dataReadyWaitTime);
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
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = byteValue & 0x01;
        }

        // read data
        averageNormal.accelerationX += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A));
        averageNormal.accelerationY += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A));
        averageNormal.accelerationZ += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A));
    }
    averageNormal.accelerationX /= SIZE_DATA;
    averageNormal.accelerationY /= SIZE_DATA;
    averageNormal.accelerationZ /= SIZE_DATA;

    // enable accelerometer self test
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x01);

    // wait for stable output
    miosix::Thread::sleep(100);

    // wait for accelerometer data ready
    byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
    byteValue = byteValue & 0x01;
    while (byteValue != 1)
    {
        miosix::Thread::sleep(dataReadyWaitTime);
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
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = byteValue & 0x01;
        }

        // read data
        averageSF.accelerationX += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A));
        averageSF.accelerationY += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A));
        averageSF.accelerationZ += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A));
    }
    averageSF.accelerationX /= SIZE_DATA;
    averageSF.accelerationY /= SIZE_DATA;
    averageSF.accelerationZ /= SIZE_DATA;

    if (LSM6DSRXDefs::ACC_SELF_TEST_MIN <=
            std::abs(averageSF.accelerationX - averageNormal.accelerationX) &&
        std::abs(averageSF.accelerationX - averageNormal.accelerationX) <=
            LSM6DSRXDefs::ACC_SELF_TEST_MAX &&
        LSM6DSRXDefs::ACC_SELF_TEST_MIN <=
            std::abs(averageSF.accelerationY - averageNormal.accelerationY) &&
        std::abs(averageSF.accelerationY - averageNormal.accelerationY) <=
            LSM6DSRXDefs::ACC_SELF_TEST_MAX &&
        LSM6DSRXDefs::ACC_SELF_TEST_MIN <=
            std::abs(averageSF.accelerationZ - averageNormal.accelerationZ) &&
        std::abs(averageSF.accelerationZ - averageNormal.accelerationZ) <=
            LSM6DSRXDefs::ACC_SELF_TEST_MAX)
    {
        returnValue = true;
    }
    else
    {
        LOG_ERR(logger, "Accelerometer self-test failed!");
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
    SPITransaction spi{spiSlave};

    // sleep time for data ready interrupt (150% odr during self-test)
    // expressed in milliseconds.
    // for this self-test the odr is 208Hz
    const unsigned int dataReadyWaitTime = 8;

    LSM6DSRXData averageSF;      // average data during self test
    LSM6DSRXData averageNormal;  // average normal data

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
        miosix::Thread::sleep(dataReadyWaitTime);
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
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = (byteValue & 0x02) >> 1;
        }

        // read data
        averageNormal.angularSpeedX += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G));
        averageNormal.angularSpeedY += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G));
        averageNormal.angularSpeedZ += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G));
    }
    averageNormal.angularSpeedX /= SIZE_DATA;
    averageNormal.angularSpeedY /= SIZE_DATA;
    averageNormal.angularSpeedZ /= SIZE_DATA;

    // enable gyroscope self test
    spi.writeRegister(LSM6DSRXDefs::REG_CTRL5_C, 0x04);

    // wait for stable output
    miosix::Thread::sleep(100);

    // wait for gyroscope data ready
    byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
    byteValue = (byteValue & 0x02) >> 1;
    while (byteValue != 1)
    {
        miosix::Thread::sleep(dataReadyWaitTime);
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
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue = spi.readRegister(LSM6DSRXDefs::REG_STATUS);
            byteValue = (byteValue & 0x02) >> 1;
        }

        // read data
        averageSF.angularSpeedX += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G));
        averageSF.angularSpeedY += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G));
        averageSF.angularSpeedZ += static_cast<float>(getAxisData(
            LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G));
    }
    averageSF.angularSpeedX /= SIZE_DATA;
    averageSF.angularSpeedY /= SIZE_DATA;
    averageSF.angularSpeedZ /= SIZE_DATA;

    if (LSM6DSRXDefs::GYR_SELF_TEST_MIN <=
            std::abs(averageSF.angularSpeedX - averageNormal.angularSpeedX) &&
        std::abs(averageSF.angularSpeedX - averageNormal.angularSpeedX) <=
            LSM6DSRXDefs::GYR_SELF_TEST_MAX &&
        LSM6DSRXDefs::GYR_SELF_TEST_MIN <=
            std::abs(averageSF.angularSpeedY - averageNormal.angularSpeedY) &&
        std::abs(averageSF.angularSpeedY - averageNormal.angularSpeedY) <=
            LSM6DSRXDefs::GYR_SELF_TEST_MAX &&
        LSM6DSRXDefs::GYR_SELF_TEST_MIN <=
            std::abs(averageSF.angularSpeedZ - averageNormal.angularSpeedZ) &&
        std::abs(averageSF.angularSpeedZ - averageNormal.angularSpeedZ) <=
            LSM6DSRXDefs::GYR_SELF_TEST_MAX)
    {
        returnValue = true;
    }
    else
    {
        LOG_ERR(logger, "Gyroscope self-test failed!");
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
        SPITransaction transaction{spiSlave};
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

uint16_t LSM6DSRX::combineHighLowBitsUnsigned(uint8_t low, uint8_t high)
{
    uint16_t sample = high;
    sample <<= 8;
    sample |= low;
    return sample;
}

void LSM6DSRX::getAccelerometerData(LSM6DSRXData& data)
{
    data.accelerationTimestamp = TimestampTimer::getTimestamp();

    data.accelerationX = getAxisData(
        LSM6DSRXDefs::REG_OUTX_L_A, LSM6DSRXDefs::REG_OUTX_H_A, sensitivityAcc);
    data.accelerationY = getAxisData(
        LSM6DSRXDefs::REG_OUTY_L_A, LSM6DSRXDefs::REG_OUTY_H_A, sensitivityAcc);
    data.accelerationZ = getAxisData(
        LSM6DSRXDefs::REG_OUTZ_L_A, LSM6DSRXDefs::REG_OUTZ_H_A, sensitivityAcc);
}

void LSM6DSRX::getGyroscopeData(LSM6DSRXData& data)
{
    data.angularSpeedTimestamp = TimestampTimer::getTimestamp();

    data.angularSpeedX = getAxisData(
        LSM6DSRXDefs::REG_OUTX_L_G, LSM6DSRXDefs::REG_OUTX_H_G, sensitivityGyr);
    data.angularSpeedY = getAxisData(
        LSM6DSRXDefs::REG_OUTY_L_G, LSM6DSRXDefs::REG_OUTY_H_G, sensitivityGyr);
    data.angularSpeedZ = getAxisData(
        LSM6DSRXDefs::REG_OUTZ_L_G, LSM6DSRXDefs::REG_OUTZ_H_G, sensitivityGyr);
}

uint32_t LSM6DSRX::getSensorTimestamp()
{
    SPITransaction spi{spiSlave};

    uint32_t value = spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP0);
    value |= spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP1) << 8;
    value |= spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP2) << 16;
    value |= spi.readRegister(LSM6DSRXDefs::REG_TIMESTAMP3) << 24;

    return value;
}

LSM6DSRXData LSM6DSRX::getSensorData()
{
    D(assert(isInit && "init() was not called"));

    LSM6DSRXData data;

    getAccelerometerData(data);
    getGyroscopeData(data);

    return data;
}

bool LSM6DSRX::selfTest()
{
    D(assert(isInit && "init() was not called"));

    isInit = false;

    if (!selfTestAcc() || !selfTestGyr())
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        return false;
    }

    return init();
}

LSM6DSRXData LSM6DSRX::sampleImpl()
{
    D(assert(isInit && "init() was not called"));
    // Reset any errors.
    lastError = SensorErrors::NO_ERRORS;

    if (config.fifoMode == LSM6DSRXConfig::FIFO_MODE::BYPASS)
    {
        return getSensorData();
    }

    readFromFifo();

    if (lastFifoLevel > 0)
    {
        return lastFifo[lastFifoLevel - 1];
    }
    else
    {
        // no new data
        lastError = SensorErrors::NO_NEW_DATA;
        return lastValidSample;
    }
}

float LSM6DSRX::getAxisData(LSM6DSRXDefs::Registers lowReg,
                            LSM6DSRXDefs::Registers highReg, float sensitivity)
{
    int8_t low = 0, high = 0;
    int16_t sample = 0;

    SPITransaction transaction{spiSlave};

    high = transaction.readRegister(highReg);
    low  = transaction.readRegister(lowReg);

    sample = combineHighLowBits(low, high);

    float ret = static_cast<float>(sample) * sensitivity;
    return ret;
}

int16_t LSM6DSRX::getAxisData(LSM6DSRXDefs::Registers lowReg,
                              LSM6DSRXDefs::Registers highReg)
{
    int8_t low = 0, high = 0;
    int16_t sample = 0;

    SPITransaction transaction{spiSlave};

    high = transaction.readRegister(highReg);
    low  = transaction.readRegister(lowReg);

    sample = combineHighLowBits(low, high);

    return sample;
}

uint64_t LSM6DSRX::convertTimestamp(const uint64_t sensorTimestamp)
{
    uint64_t deltaSensor = 0;  // difference between 2 sensor timestamps.
    if (sensorTimestamp >= sensorTimestamp0)
    {
        deltaSensor = sensorTimestamp - sensorTimestamp0;
    }
    else
    {
        deltaSensor =
            static_cast<uint64_t>(-1) - sensorTimestamp0 + sensorTimestamp + 1;
    }

    // delta evaluated from TimestampTimer point of view.
    uint64_t delta = deltaSensor * sensorTimestampResolution;

    return timestamp0 + delta;
}

void LSM6DSRX::correlateTimestamps()
{
    timestamp0       = TimestampTimer::getTimestamp();
    sensorTimestamp0 = getSensorTimestamp();
}

float LSM6DSRX::getSensorTimestampResolution()
{
    SPITransaction spi{spiSlave};

    uint8_t value = spi.readRegister(LSM6DSRXDefs::REG_INTERNAL_FREQ_FINE);

    // TS_Res = 1 / (40000 + (0.0015 * INTERNAL_FREQ_FINE * 40000))
    float TS_Res = 1 / (40000 + (0.0015 * value * 40000));
    return TS_Res * 1000;
}

void LSM6DSRX::readFromFifo()
{
    SPITransaction spi{spiSlave};

    // get number of sample to read
    const uint16_t numSamples =
        std::min(LSM6DSRXDefs::FIFO_SIZE, unreadDataInFifo());

    /**
     * Data has 2bits tags that determins the corresponding time slot.
     *
     * 00 -> first element of the array (timestamps[0])
     * 11 -> last element of the array (timestamps[3])
     *
     * When a new timestamp is received from the fifo the array is updated.
     * NOTE: those timestamps are already converted from sensor ones to
     * TimestampTimer class.
     */
    LSM6DSRXDefs::FifoTimeslotData
        timestamps[LSM6DSRXDefs::FIFO_TIMESLOT_NUMBER] = {LSM6DSRXData(), false,
                                                          false};

    // read samples from the sensors
    spi.readRegisters(LSM6DSRXDefs::REG_FIFO_DATA_OUT_TAG,
                      reinterpret_cast<uint8_t*>(rawFifo.data()),
                      numSamples * sizeof(LSM6DSRXDefs::RawFifoData));

    // not all data extracted from fifo is sample data, timestamps are not
    // saved.
    // --> `i` keeps count of the number of elements in `rawFifo`
    //     `idxFifo` keeps track of the samples saved inside `lastFifo`
    uint16_t idxFifo = 0;
    for (uint16_t i = 0; i < numSamples; ++i)
    {
        const uint8_t sensorTag   = (rawFifo[i].sampleTag >> 3) & 31;
        const uint8_t timeslotTag = (rawFifo[i].sampleTag & 6) >> 1;

        const uint8_t xl = rawFifo[i].xl;
        const uint8_t xh = rawFifo[i].xh;
        const uint8_t yl = rawFifo[i].yl;
        const uint8_t yh = rawFifo[i].yh;
        const uint8_t zl = rawFifo[i].zl;
        const uint8_t zh = rawFifo[i].zh;

        switch (sensorTag)
        {
            case 0x01:
                // Gyroscope data

                // Set data
                timestamps[timeslotTag].data.angularSpeedX =
                    static_cast<float>(combineHighLowBits(xl, xh)) *
                    sensitivityGyr;
                timestamps[timeslotTag].data.angularSpeedY =
                    static_cast<float>(combineHighLowBits(yl, yh)) *
                    sensitivityGyr;
                timestamps[timeslotTag].data.angularSpeedZ =
                    static_cast<float>(combineHighLowBits(zl, zh)) *
                    sensitivityGyr;

#ifdef DEBUG
                // Check for debugging
                if (timestamps[timeslotTag].gyrPresent == true)
                {
                    TRACE("ERROR, writing twice gyr data\n");
                }
#endif
                // Set flag
                timestamps[timeslotTag].gyrPresent = true;

                // Check if we can push into fifo (both samples are present)
                if (timestamps[timeslotTag].accPresent)
                {
                    pushIntoFifo(timestamps[timeslotTag], idxFifo);
                }

                break;
            case 0x02:
                // Accelerometer data
                timestamps[timeslotTag].data.accelerationX =
                    static_cast<float>(combineHighLowBits(xl, xh)) *
                    sensitivityAcc;
                timestamps[timeslotTag].data.accelerationY =
                    static_cast<float>(combineHighLowBits(yl, yh)) *
                    sensitivityAcc;
                timestamps[timeslotTag].data.accelerationZ =
                    static_cast<float>(combineHighLowBits(zl, zh)) *
                    sensitivityAcc;

#ifdef DEBUG
                if (timestamps[timeslotTag].accPresent == true)
                {
                    TRACE("ERROR, writing twice acc data\n");
                }
#endif
                timestamps[timeslotTag].accPresent = true;

                // Check if we can push into fifo (both samples are present)
                if (timestamps[timeslotTag].gyrPresent)
                {
                    pushIntoFifo(timestamps[timeslotTag], idxFifo);
                }

                break;
            case 0x04:
                // timestamp data --> update timestamps

                uint32_t t =
                    static_cast<uint32_t>(combineHighLowBitsUnsigned(xl, xh));
                t |= static_cast<uint32_t>(combineHighLowBitsUnsigned(yl, yh))
                     << 16;

                // Check data
                // when both samples are present the data is saved in fifo and
                // flags are restored -> if 'flag' is true it means that only 1
                // of the 2 samples is present, and it is going to be
                // overwritten by this new timestamp
                // #ifdef DEBUG
                //                 bool flag =
                //                 timestamps[timeslotTag].accPresent ||
                //                             timestamps[timeslotTag].gyrPresent;
                //                 if (flag == true)
                //                 {
                //                     TRACE(
                //                         "ERROR, overwriting not pushed sample
                //                         with a new " "timestamp\n");
                //                 }
                // #endif

                // Set new data
                timestamps[timeslotTag].data.accelerationTimestamp =
                    convertTimestamp(static_cast<uint64_t>(t));
                timestamps[timeslotTag].data.angularSpeedTimestamp =
                    timestamps[timeslotTag].data.accelerationTimestamp;

                timestamps[timeslotTag].accPresent = false;
                timestamps[timeslotTag].gyrPresent = false;

                break;
        }
    }

    // update timestamp base values
    correlateTimestamps();

    lastFifoLevel = idxFifo;
}

void LSM6DSRX::pushIntoFifo(LSM6DSRXDefs::FifoTimeslotData& timeslot,
                            uint16_t& fifoIdx)
{
    // reset flags (done even if data gets discarded)
    timeslot.accPresent = false;
    timeslot.gyrPresent = false;

    // check if data can be pushed
    if ((fifoIdx > 0 && timeslot.data.accelerationTimestamp ==
                            lastFifo[fifoIdx - 1].accelerationTimestamp) ||
        timeslot.data.accelerationTimestamp == 0)
    {
        // the new sample has the same timestamp of the previous one or
        // timestamp is 0 --> discarded
        return;
    }

    // push into fifo and update index
    lastFifo[fifoIdx] = timeslot.data;
    ++fifoIdx;

    // update lastValidSample
    lastValidSample = timeslot.data;
}

uint16_t LSM6DSRX::unreadDataInFifo()
{
    uint16_t ris = 0;
    SPITransaction spi{spiSlave};

    ris = spi.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS1);
    ris = ris | (static_cast<uint16_t>(
                     spi.readRegister(LSM6DSRXDefs::REG_FIFO_STATUS2) & 3)
                 << 8);
    return ris;
}

}  // namespace Boardcore
