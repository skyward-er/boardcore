#include "LSM6DSV320X.h"

#include <assert.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/Constants.h>
#include <utils/Debug.h>

#include <algorithm>
#include <cmath>

namespace Boardcore
{

LSM6DSV320X::LSM6DSV320X(SPIBusInterface& bus, miosix::GpioPin csPin,
                         SPIBusConfig busConfiguration,
                         LSM6DSV320XConfig& configuration)
    : spiSlave(bus, csPin, busConfiguration), config(configuration)
{
    isInit = false;

    // Cap watermark to the 8-bit WTM register maximum (255 words)
    config.fifoWatermark = std::min(config.fifoWatermark, uint16_t(255));
}

bool LSM6DSV320X::init()
{
    if (isInit)
    {
        LOG_ERR(logger, "init() should be called once");
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    if (!checkWhoAmI())
    {
        LOG_ERR(logger, "Got bad CHIPID");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    if (!checkConfig())
    {
        LOG_ERR(logger, "Invalid sensor configuration");
        lastError = SensorErrors::INVALID_CONFIG;
        return false;
    }

    // Set BDU 
    {
        SPITransaction spiTransaction{spiSlave};

        uint8_t value = static_cast<uint8_t>(config.bdu) << 6;  // BDU
        value |= 1 << 2;                                        // IF_INC

        spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL3, value);
    }

    // Setup sensors and FIFO
    initLowG();
    initHighG();
    initGyroscope();
    initFifo();

    // enable hardware timestamp 
    {
        constexpr uint8_t REG_CTRL10_TIMESTAMP_EN = 1 << 5;

        SPITransaction spiTransaction{spiSlave};
        spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL10,
                                     REG_CTRL10_TIMESTAMP_EN);
    }

    // set interrupts
    initInterrupts();

    // Correlate internal sensor clock with TimestampTimer
    correlateTimestamps();
    sensorTimestampResolution =
        getSensorTimestampResolution() * 1000.0f;  // Convert ms to us

    isInit    = true;
    lastError = SensorErrors::NO_ERRORS;
    return true;
}

bool LSM6DSV320X::checkConfig()
{
}

void LSM6DSV320X::initLowG()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // Set Low-G accelerometer operating mode and ODR
    configByte = (static_cast<uint8_t>(config.opModeLowG) << 4) |
                 static_cast<uint8_t>(config.odrLowG);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL1, configByte);

    // Set Low-G accelerometer full scale
    configByte = static_cast<uint8_t>(config.fsLowG);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL8, configByte);

    // Set sensitivity (mg/LSB)
    switch (config.fsLowG)
    {
        case LSM6DSV320XConfig::LOW_G_FULLSCALE::G2:
            sensitivityLowG = 0.061f;
            break;
        case LSM6DSV320XConfig::LOW_G_FULLSCALE::G4:
            sensitivityLowG = 0.122f;
            break;
        case LSM6DSV320XConfig::LOW_G_FULLSCALE::G8:
            sensitivityLowG = 0.244f;
            break;
        case LSM6DSV320XConfig::LOW_G_FULLSCALE::G16:
            sensitivityLowG = 0.488f;
            break;
        default:
            config.fsLowG   = LSM6DSV320XConfig::LOW_G_FULLSCALE::G2;
            sensitivityLowG = 0.061f;
            break;
    }

    // Convert sensitivity from milli-g/LSB to (m/s^2)/LSB
    constexpr float accelerationConversion = Constants::g / 1000.0f;
    sensitivityLowG *= accelerationConversion;
}

void LSM6DSV320X::initHighG()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // Set High-G accelerometer ODR, full scale and enable user interface
    // readout (XL_HG_REGOUT_EN = bit 7)
    configByte = (1 << 7) | (static_cast<uint8_t>(config.odrHighG) << 3) |
                 static_cast<uint8_t>(config.fsHighG);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL1_XL_HG,
                                 configByte);

    // Set sensitivity (mg/LSB)
    switch (config.fsHighG)
    {
        case LSM6DSV320XConfig::HIGH_G_FULLSCALE::G32:
            sensitivityHighG = 0.976f;
            break;
        case LSM6DSV320XConfig::HIGH_G_FULLSCALE::G64:
            sensitivityHighG = 1.952f;
            break;
        case LSM6DSV320XConfig::HIGH_G_FULLSCALE::G128:
            sensitivityHighG = 3.904f;
            break;
        case LSM6DSV320XConfig::HIGH_G_FULLSCALE::G256:
            sensitivityHighG = 7.808f;
            break;
        case LSM6DSV320XConfig::HIGH_G_FULLSCALE::G320:
            sensitivityHighG = 10.417f;
            break;
        default:
            config.fsHighG   = LSM6DSV320XConfig::HIGH_G_FULLSCALE::G32;
            sensitivityHighG = 0.976f;
            break;
    }

    // Convert sensitivity from milli-g/LSB to (m/s^2)/LSB
    constexpr float accelerationConversion = Constants::g / 1000.0f;
    sensitivityHighG *= accelerationConversion;
}

void LSM6DSV320X::initGyroscope()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // Set gyroscope operating mode and ODR in CTRL2
    configByte = (static_cast<uint8_t>(config.opModeGyr) << 4) |
                 static_cast<uint8_t>(config.odrGyr);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL2, configByte);

    // Set gyroscope full scale in CTRL6
    configByte = static_cast<uint8_t>(config.fsGyr);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::CTRL6, configByte);

    // Set sensitivity (mdps/LSB)
    switch (config.fsGyr)
    {
        case LSM6DSV320XConfig::GYR_FULLSCALE::DPS_125:
            sensitivityGyr = 4.375f;
            break;
        case LSM6DSV320XConfig::GYR_FULLSCALE::DPS_250:
            sensitivityGyr = 8.75f;
            break;
        case LSM6DSV320XConfig::GYR_FULLSCALE::DPS_500:
            sensitivityGyr = 17.5f;
            break;
        case LSM6DSV320XConfig::GYR_FULLSCALE::DPS_1000:
            sensitivityGyr = 35.0f;
            break;
        case LSM6DSV320XConfig::GYR_FULLSCALE::DPS_2000:
            sensitivityGyr = 70.0f;
            break;
        case LSM6DSV320XConfig::GYR_FULLSCALE::DPS_4000:
            sensitivityGyr = 140.0f;
            break;
        default:
            config.fsGyr   = LSM6DSV320XConfig::GYR_FULLSCALE::DPS_125;
            sensitivityGyr = 4.375f;
            break;
    }

    // Convert sensitivity from milli-dps/LSB to (rad/s)/LSB
    constexpr float angularConversion = Constants::DEGREES_TO_RADIANS / 1000.0f;
    sensitivityGyr *= angularConversion;
}

void LSM6DSV320X::initFifo()
{
    uint8_t configByte = 0;
    SPITransaction spiTransaction{spiSlave};

    // Select Low-G accelerometer and gyroscope batch data rates in FIFO_CTRL3
    configByte = static_cast<uint8_t>(config.fifoBdrLowG) |
                 (static_cast<uint8_t>(config.fifoBdrGyr) << 4);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::FIFO_CTRL3,
                                 configByte);

    // Select FIFO operating mode, temperature BDR and timestamp decimation in
    // FIFO_CTRL4
    configByte = static_cast<uint8_t>(config.fifoMode) |
                 (static_cast<uint8_t>(config.fifoTemperatureBdr) << 4) |
                 (static_cast<uint8_t>(config.fifoTimestampDecimation) << 6);
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::FIFO_CTRL4,
                                 configByte);

    // Enable or disable High-G accelerometer FIFO batching in COUNTER_BDR_REG1
    // (XL_HG_BATCH_EN = bit 3)
    configByte = config.fifoHighGBatchEnabled ? (1 << 3) : 0x00;
    spiTransaction.writeRegister(LSM6DSV320XDefs::Registers::COUNTER_BDR_REG1,
                                 configByte);
}

void LSM6DSV320X::initInterrupts()
{
    uint8_t buf[] = {0, 0};
    SPITransaction spi{spiSlave};

    buf[0] = static_cast<uint8_t>(config.int1InterruptSelection);
    buf[1] = static_cast<uint8_t>(config.int2InterruptSelection);
    spi.writeRegisters(LSM6DSV320XDefs::Registers::INT1_CTRL, buf, 2);

    // Set FIFO watermark threshold (8-bit WTM field in FIFO_CTRL1)
    spi.writeRegister(LSM6DSV320XDefs::Registers::FIFO_CTRL1,
                      static_cast<uint8_t>(config.fifoWatermark & 0xFF));
}

bool LSM6DSV320X::selfTestLowG()
{
    bool returnValue        = false;
    uint8_t byteValue       = 0;
    uint8_t idx             = 0;
    const uint8_t SIZE_DATA = 5;
    SPITransaction spi{spiSlave};

    // Sleep time for data-ready polling at 60 Hz ODR (~17 ms)
    const unsigned int dataReadyWaitTime = 20;

    LSM6DSV320XData averageSF;
    LSM6DSV320XData averageNormal;

    // Set Low-G ODR = 60 Hz (0x05), High-Performance mode (0x00)
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL1, 0x05);
    // Power down gyroscope
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL2, 0x00);
    // Set BDU and auto-increment
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL3, 0x44);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL4, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL5, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL6, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL7, 0x00);
    // Set Low-G full scale = 2g (0x00)
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL8, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL9, 0x00);
    // Disable self-test and timestamp counter
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL10, 0x00);

    // Wait for stable output
    miosix::Thread::sleep(100);

    // Wait for Low-G accelerometer data ready (XLDA = bit 0)
    do
    {
        miosix::Thread::sleep(dataReadyWaitTime);
        byteValue = spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) &
                    0x01;
    } while (byteValue != 1);

    // Read and discard first sample
    readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_A,
                LSM6DSV320XDefs::Registers::OUTX_H_A);
    readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_A,
                LSM6DSV320XDefs::Registers::OUTY_H_A);
    readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_A,
                LSM6DSV320XDefs::Registers::OUTZ_H_A);

    // Read normal data (self-test disabled)
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        do
        {
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue =
                spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) & 0x01;
        } while (byteValue != 1);

        averageNormal.lowG.accelerationX +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_A,
                        LSM6DSV320XDefs::Registers::OUTX_H_A) *
            0.061f;
        averageNormal.lowG.accelerationY +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_A,
                        LSM6DSV320XDefs::Registers::OUTY_H_A) *
            0.061f;
        averageNormal.lowG.accelerationZ +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_A,
                        LSM6DSV320XDefs::Registers::OUTZ_H_A) *
            0.061f;
    }
    averageNormal.lowG.accelerationX /= SIZE_DATA;
    averageNormal.lowG.accelerationY /= SIZE_DATA;
    averageNormal.lowG.accelerationZ /= SIZE_DATA;

    // Enable Low-G accelerometer positive sign self-test (ST_XL[1:0] = 01 in
    // CTRL10)
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL10, 0x01);

    // Wait for stable output
    miosix::Thread::sleep(100);

    do
    {
        miosix::Thread::sleep(dataReadyWaitTime);
        byteValue = spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) &
                    0x01;
    } while (byteValue != 1);

    // Read and discard first self-test sample
    readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_A,
                LSM6DSV320XDefs::Registers::OUTX_H_A);
    readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_A,
                LSM6DSV320XDefs::Registers::OUTY_H_A);
    readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_A,
                LSM6DSV320XDefs::Registers::OUTZ_H_A);

    // Read self-test data
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        do
        {
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue =
                spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) & 0x01;
        } while (byteValue != 1);

        averageSF.lowG.accelerationX +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_A,
                        LSM6DSV320XDefs::Registers::OUTX_H_A) *
            0.061f;
        averageSF.lowG.accelerationY +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_A,
                        LSM6DSV320XDefs::Registers::OUTY_H_A) *
            0.061f;
        averageSF.lowG.accelerationZ +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_A,
                        LSM6DSV320XDefs::Registers::OUTZ_H_A) *
            0.061f;
    }
    averageSF.lowG.accelerationX /= SIZE_DATA;
    averageSF.lowG.accelerationY /= SIZE_DATA;
    averageSF.lowG.accelerationZ /= SIZE_DATA;

    const float deltaX =
        std::abs(averageSF.lowG.accelerationX - averageNormal.lowG.accelerationX);
    const float deltaY =
        std::abs(averageSF.lowG.accelerationY - averageNormal.lowG.accelerationY);
    const float deltaZ =
        std::abs(averageSF.lowG.accelerationZ - averageNormal.lowG.accelerationZ);

    if (LSM6DSV320XDefs::LOW_G_SELF_TEST_MIN <= deltaX &&
        deltaX <= LSM6DSV320XDefs::LOW_G_SELF_TEST_MAX &&
        LSM6DSV320XDefs::LOW_G_SELF_TEST_MIN <= deltaY &&
        deltaY <= LSM6DSV320XDefs::LOW_G_SELF_TEST_MAX &&
        LSM6DSV320XDefs::LOW_G_SELF_TEST_MIN <= deltaZ &&
        deltaZ <= LSM6DSV320XDefs::LOW_G_SELF_TEST_MAX)
    {
        returnValue = true;
    }
    else
    {
        LOG_ERR(logger, "Low-G accelerometer self-test failed!");
        returnValue = false;
    }

    // Disable self-test and power down Low-G accelerometer
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL10, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL1, 0x00);

    return returnValue;
}

bool LSM6DSV320X::selfTestGyr()
{
    bool returnValue        = false;
    uint8_t byteValue       = 0;
    uint8_t idx             = 0;
    const uint8_t SIZE_DATA = 5;
    SPITransaction spi{spiSlave};

    // Sleep time for data-ready polling at 240 Hz ODR (~4.2 ms)
    const unsigned int dataReadyWaitTime = 6;

    LSM6DSV320XData averageSF;
    LSM6DSV320XData averageNormal;

    // Power down Low-G accelerometer
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL1, 0x00);
    // Set gyroscope ODR = 240 Hz (0x07), High-Performance mode (0x00)
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL2, 0x07);
    // Set BDU and auto-increment
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL3, 0x44);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL4, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL5, 0x00);
    // Set gyroscope full scale = 2000 dps (0x04)
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL6, 0x04);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL7, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL8, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL9, 0x00);
    // Disable self-test and timestamp counter
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL10, 0x00);

    // Wait for stable output
    miosix::Thread::sleep(100);

    // Wait for gyroscope data ready (GDA = bit 1)
    do
    {
        miosix::Thread::sleep(dataReadyWaitTime);
        byteValue =
            (spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) & 0x02) >>
            1;
    } while (byteValue != 1);

    // Read and discard first sample
    readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_G,
                LSM6DSV320XDefs::Registers::OUTX_H_G);
    readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_G,
                LSM6DSV320XDefs::Registers::OUTY_H_G);
    readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_G,
                LSM6DSV320XDefs::Registers::OUTZ_H_G);

    // Read normal data (self-test disabled)
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        do
        {
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue =
                (spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) &
                 0x02) >>
                1;
        } while (byteValue != 1);

        averageNormal.gyroscope.angularSpeedX +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_G,
                        LSM6DSV320XDefs::Registers::OUTX_H_G) *
            0.070f;
        averageNormal.gyroscope.angularSpeedY +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_G,
                        LSM6DSV320XDefs::Registers::OUTY_H_G) *
            0.070f;
        averageNormal.gyroscope.angularSpeedZ +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_G,
                        LSM6DSV320XDefs::Registers::OUTZ_H_G) *
            0.070f;
    }
    averageNormal.gyroscope.angularSpeedX /= SIZE_DATA;
    averageNormal.gyroscope.angularSpeedY /= SIZE_DATA;
    averageNormal.gyroscope.angularSpeedZ /= SIZE_DATA;

    // Enable gyroscope positive sign self-test (ST_G[1:0] = 01 -> bits [3:2]
    // in CTRL10 = 0x04)
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL10, 0x04);

    // Wait for stable output
    miosix::Thread::sleep(100);

    do
    {
        miosix::Thread::sleep(dataReadyWaitTime);
        byteValue =
            (spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) & 0x02) >>
            1;
    } while (byteValue != 1);

    // Read and discard first self-test sample
    readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_G,
                LSM6DSV320XDefs::Registers::OUTX_H_G);
    readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_G,
                LSM6DSV320XDefs::Registers::OUTY_H_G);
    readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_G,
                LSM6DSV320XDefs::Registers::OUTZ_H_G);

    // Read self-test data
    for (idx = 0; idx < SIZE_DATA; ++idx)
    {
        do
        {
            miosix::Thread::sleep(dataReadyWaitTime);
            byteValue =
                (spi.readRegister(LSM6DSV320XDefs::Registers::STATUS_REG) &
                 0x02) >>
                1;
        } while (byteValue != 1);

        averageSF.gyroscope.angularSpeedX +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_G,
                        LSM6DSV320XDefs::Registers::OUTX_H_G) *
            0.070f;
        averageSF.gyroscope.angularSpeedY +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_G,
                        LSM6DSV320XDefs::Registers::OUTY_H_G) *
            0.070f;
        averageSF.gyroscope.angularSpeedZ +=
            readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_G,
                        LSM6DSV320XDefs::Registers::OUTZ_H_G) *
            0.070f;
    }
    averageSF.gyroscope.angularSpeedX /= SIZE_DATA;
    averageSF.gyroscope.angularSpeedY /= SIZE_DATA;
    averageSF.gyroscope.angularSpeedZ /= SIZE_DATA;

    const float deltaX = std::abs(averageSF.gyroscope.angularSpeedX -
                                  averageNormal.gyroscope.angularSpeedX);
    const float deltaY = std::abs(averageSF.gyroscope.angularSpeedY -
                                  averageNormal.gyroscope.angularSpeedY);
    const float deltaZ = std::abs(averageSF.gyroscope.angularSpeedZ -
                                  averageNormal.gyroscope.angularSpeedZ);

    if (LSM6DSV320XDefs::GYR_SELF_TEST_MIN <= deltaX &&
        deltaX <= LSM6DSV320XDefs::GYR_SELF_TEST_MAX &&
        LSM6DSV320XDefs::GYR_SELF_TEST_MIN <= deltaY &&
        deltaY <= LSM6DSV320XDefs::GYR_SELF_TEST_MAX &&
        LSM6DSV320XDefs::GYR_SELF_TEST_MIN <= deltaZ &&
        deltaZ <= LSM6DSV320XDefs::GYR_SELF_TEST_MAX)
    {
        returnValue = true;
    }
    else
    {
        LOG_ERR(logger, "Gyroscope self-test failed!");
        returnValue = false;
    }

    // Disable self-test and power down gyroscope
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL10, 0x00);
    spi.writeRegister(LSM6DSV320XDefs::Registers::CTRL2, 0x00);

    return returnValue;
}

bool LSM6DSV320X::checkWhoAmI()
{
    uint8_t regValue = 0;
    {
        SPITransaction transaction{spiSlave};
        regValue = transaction.readRegister(LSM6DSV320XDefs::Registers::WHO_AM_I);
    }

    return regValue == LSM6DSV320XDefs::WHO_AM_I_VALUE;
}

int16_t LSM6DSV320X::combineHighLowBits(uint8_t low, uint8_t high)
{
    int16_t ret = high;
    ret <<= 8;
    ret |= low;
    return ret;
}

uint16_t LSM6DSV320X::combineHighLowBitsUnsigned(uint8_t low, uint8_t high)
{
    uint16_t sample = high;
    sample <<= 8;
    sample |= low;
    return sample;
}

void LSM6DSV320X::getLowGData(LSM6DSV320XData& data)
{
    data.lowG.accelerationX =
        readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_A,
                    LSM6DSV320XDefs::Registers::OUTX_H_A) *
        sensitivityLowG;
    data.lowG.accelerationY =
        readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_A,
                    LSM6DSV320XDefs::Registers::OUTY_H_A) *
        sensitivityLowG;
    data.lowG.accelerationZ =
        readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_A,
                    LSM6DSV320XDefs::Registers::OUTZ_H_A) *
        sensitivityLowG;
}

void LSM6DSV320X::getHighGData(LSM6DSV320XData& data)
{
    data.highG.accelerationX =
        readFloat16(LSM6DSV320XDefs::Registers::UI_OUTX_L_A_OIS_HG,
                    LSM6DSV320XDefs::Registers::UI_OUTX_H_A_OIS_HG) *
        sensitivityHighG;
    data.highG.accelerationY =
        readFloat16(LSM6DSV320XDefs::Registers::UI_OUTY_L_A_OIS_HG,
                    LSM6DSV320XDefs::Registers::UI_OUTY_H_A_OIS_HG) *
        sensitivityHighG;
    data.highG.accelerationZ =
        readFloat16(LSM6DSV320XDefs::Registers::UI_OUTZ_L_A_OIS_HG,
                    LSM6DSV320XDefs::Registers::UI_OUTZ_H_A_OIS_HG) *
        sensitivityHighG;
}

void LSM6DSV320X::getGyroscopeData(LSM6DSV320XData& data)
{
    data.gyroscope.angularSpeedX =
        readFloat16(LSM6DSV320XDefs::Registers::OUTX_L_G,
                    LSM6DSV320XDefs::Registers::OUTX_H_G) *
        sensitivityGyr;
    data.gyroscope.angularSpeedY =
        readFloat16(LSM6DSV320XDefs::Registers::OUTY_L_G,
                    LSM6DSV320XDefs::Registers::OUTY_H_G) *
        sensitivityGyr;
    data.gyroscope.angularSpeedZ =
        readFloat16(LSM6DSV320XDefs::Registers::OUTZ_L_G,
                    LSM6DSV320XDefs::Registers::OUTZ_H_G) *
        sensitivityGyr;
}

void LSM6DSV320X::getTemperatureData(LSM6DSV320XData& data)
{
    data.temperature.temperature =
        readFloat16(LSM6DSV320XDefs::Registers::OUT_TEMP_L,
                    LSM6DSV320XDefs::Registers::OUT_TEMP_H) /
        256.0f;
    data.temperature.temperature += 25.0f;
}

uint32_t LSM6DSV320X::getSensorTimestamp()
{
    SPITransaction spi{spiSlave};

    uint32_t value = spi.readRegister(LSM6DSV320XDefs::Registers::TIMESTAMP0);
    value |=
        static_cast<uint32_t>(
            spi.readRegister(LSM6DSV320XDefs::Registers::TIMESTAMP1))
        << 8;
    value |=
        static_cast<uint32_t>(
            spi.readRegister(LSM6DSV320XDefs::Registers::TIMESTAMP2))
        << 16;
    value |=
        static_cast<uint32_t>(
            spi.readRegister(LSM6DSV320XDefs::Registers::TIMESTAMP3))
        << 24;

    return value;
}

LSM6DSV320XData LSM6DSV320X::getSensorData()
{
    D(assert(isInit && "init() was not called"));

    LSM6DSV320XData data;

    const uint64_t t                 = TimestampTimer::getTimestamp();
    data.lowG.accelerationTimestamp    = now;
    data.highG.accelerationTimestamp   = now;
    data.gyroscope.angularSpeedTimestamp = now;
    data.temperature.temperatureTimestamp  = now;

    getLowGData(data);
    getHighGData(data);
    getGyroscopeData(data);
    getTemperatureData(data);

    return data;
}

bool LSM6DSV320X::selfTest()
{
    D(assert(isInit && "init() was not called"));

    isInit = false;

    if (!selfTestLowG() || !selfTestGyr())
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        return false;
    }

    return init();
}

LSM6DSV320XData LSM6DSV320X::sampleImpl()
{
    D(assert(isInit && "init() was not called"));

    // Reset any errors
    lastError = SensorErrors::NO_ERRORS;

    if (config.fifoMode == LSM6DSV320XConfig::FIFO_MODE::BYPASS)
        return getSensorData();

    readFromFifo();

    if (lastFifoLevel > 0)
    {
        return lastFifo[lastFifoLevel - 1];
    }
    else
    {
        // No new data
        lastError = SensorErrors::NO_NEW_DATA;
        return lastValidSample;
    }
}

float LSM6DSV320X::readFloat16(LSM6DSV320XDefs::Registers lowReg,
                               LSM6DSV320XDefs::Registers highReg)
{
    SPITransaction transaction{spiSlave};

    uint8_t high = transaction.readRegister(highReg);
    uint8_t low  = transaction.readRegister(lowReg);

    int16_t sample = combineHighLowBits(low, high);
    return static_cast<float>(sample);
}

int16_t LSM6DSV320X::readInt16(LSM6DSV320XDefs::Registers lowReg,
                               LSM6DSV320XDefs::Registers highReg)
{
    SPITransaction transaction{spiSlave};

    uint8_t high = transaction.readRegister(highReg);
    uint8_t low  = transaction.readRegister(lowReg);

    return combineHighLowBits(low, high);
}

uint64_t LSM6DSV320X::convertTimestamp(const uint64_t sensorTimestamp)
{
    uint64_t deltaSensor = 0;
    if (sensorTimestamp >= sensorTimestamp0)
    {
        deltaSensor = sensorTimestamp - sensorTimestamp0;
    }
    else
    {
        deltaSensor =
            static_cast<uint64_t>(-1) - sensorTimestamp0 + sensorTimestamp + 1;
    }

    uint64_t delta =
        static_cast<uint64_t>(deltaSensor * sensorTimestampResolution);

    return timestamp0 + delta;
}

void LSM6DSV320X::correlateTimestamps()
{
    timestamp0       = TimestampTimer::getTimestamp();
    sensorTimestamp0 = getSensorTimestamp();
}

float LSM6DSV320X::getSensorTimestampResolution()
{
    SPITransaction spi{spiSlave};

    int8_t value = static_cast<int8_t>(
        spi.readRegister(LSM6DSV320XDefs::Registers::INTERNAL_FREQ_FINE));

    // TS_Res = 1 / (40000 + (0.0015 * INTERNAL_FREQ_FINE * 40000))
    float tsRes = 1.0f / (40000.0f + (0.0015f * value * 40000.0f));
    return tsRes * 1000.0f;
}

void LSM6DSV320X::readFromFifo()
{
    // Lock mutex for thread-safe FIFO reading
    miosix::Lock<miosix::FastMutex> l(mutex);

    SPITransaction spi{spiSlave};

    // Get number of unread FIFO words
    const uint16_t numSamples =
        std::min(unreadDataInFifo(), LSM6DSV320XDefs::SENSOR_FIFO_SIZE);

    if (numSamples == 0)
    {
        lastFifoLevel = 0;
        return;
    }

    LSM6DSV320XDefs::FifoTimeslotData
        timeslots[LSM6DSV320XDefs::FIFO_TIMESLOT_NUMBER] = {};

    // Read unread words from the hardware FIFO
    spi.readRegisters(LSM6DSV320XDefs::Registers::FIFO_DATA_OUT_TAG,
                      reinterpret_cast<uint8_t*>(rawFifo.data()),
                      numSamples * sizeof(LSM6DSV320XDefs::RawFifoData));

    uint16_t idxFifo = 0;

    for (uint16_t i = 0; i < numSamples; ++i)
    {
        const uint8_t sensorTag =
            rawFifo[i].tag & LSM6DSV320XDefs::FIFO_TAG_MASK;
        const uint8_t timeslotTag =
            (rawFifo[i].tag & LSM6DSV320XDefs::FIFO_TAG_COUNTER_MASK) >>
            LSM6DSV320XDefs::FIFO_TAG_COUNTER_SHIFT;

        const uint8_t xl = rawFifo[i].xLow;
        const uint8_t xh = rawFifo[i].xHigh;
        const uint8_t yl = rawFifo[i].yLow;
        const uint8_t yh = rawFifo[i].yHigh;
        const uint8_t zl = rawFifo[i].zLow;
        const uint8_t zh = rawFifo[i].zHigh;

        auto& slot = timeslots[timeslotTag];

        switch (static_cast<LSM6DSV320XDefs::FifoTag>(sensorTag))
        {
            case LSM6DSV320XDefs::FifoTag::GYROSCOPE:
            {
                slot.data.gyroscope.angularSpeedX =
                    static_cast<float>(combineHighLowBits(xl, xh)) *
                    sensitivityGyr;
                slot.data.gyroscope.angularSpeedY =
                    static_cast<float>(combineHighLowBits(yl, yh)) *
                    sensitivityGyr;
                slot.data.gyroscope.angularSpeedZ =
                    static_cast<float>(combineHighLowBits(zl, zh)) *
                    sensitivityGyr;

                slot.gyrPresent = true;
                break;
            }

            case LSM6DSV320XDefs::FifoTag::LOW_G_ACCELEROMETER:
            {
                slot.data.lowG.accelerationX =
                    static_cast<float>(combineHighLowBits(xl, xh)) *
                    sensitivityLowG;
                slot.data.lowG.accelerationY =
                    static_cast<float>(combineHighLowBits(yl, yh)) *
                    sensitivityLowG;
                slot.data.lowG.accelerationZ =
                    static_cast<float>(combineHighLowBits(zl, zh)) *
                    sensitivityLowG;

                slot.lowGPresent = true;
                break;
            }

            case LSM6DSV320XDefs::FifoTag::HIGH_G_ACCELEROMETER:
            {
                slot.data.highG.accelerationX =
                    static_cast<float>(combineHighLowBits(xl, xh)) *
                    sensitivityHighG;
                slot.data.highG.accelerationY =
                    static_cast<float>(combineHighLowBits(yl, yh)) *
                    sensitivityHighG;
                slot.data.highG.accelerationZ =
                    static_cast<float>(combineHighLowBits(zl, zh)) *
                    sensitivityHighG;

                slot.highGPresent = true;
                break;
            }

            case LSM6DSV320XDefs::FifoTag::TEMPERATURE:
            {
                slot.data.temperature.temperature =
                    static_cast<float>(combineHighLowBits(xl, xh)) / 256.0f +
                    25.0f;

                slot.tempPresent = true;
                break;
            }

            case LSM6DSV320XDefs::FifoTag::TIMESTAMP:
            {
                // Flush any previously accumulated sensor records in this
                // timeslot before updating the slot's timestamp
                if (slot.lowGPresent || slot.highGPresent || slot.gyrPresent ||
                    slot.tempPresent)
                {
                    pushIntoFifo(slot, idxFifo);
                }

                uint32_t t =
                    static_cast<uint32_t>(combineHighLowBitsUnsigned(xl, xh));
                t |= static_cast<uint32_t>(combineHighLowBitsUnsigned(yl, yh))
                     << 16;

                slot.slotTimestamp = convertTimestamp(static_cast<uint64_t>(t));
                break;
            }

            default:
                break;
        }
    }

    // Flush any remaining timeslots that were populated at the end of the FIFO
    // batch without being followed by another TIMESTAMP record
    for (uint8_t s = 0; s < LSM6DSV320XDefs::FIFO_TIMESLOT_NUMBER; ++s)
    {
        if (timeslots[s].lowGPresent || timeslots[s].highGPresent ||
            timeslots[s].gyrPresent || timeslots[s].tempPresent)
        {
            pushIntoFifo(timeslots[s], idxFifo);
        }
    }

    // Update timestamp base values
    correlateTimestamps();

    lastFifoLevel = idxFifo;
}

void LSM6DSV320X::pushIntoFifo(LSM6DSV320XDefs::FifoTimeslotData& timeslot,
                               uint16_t& fifoIdx)
{
    // If no sensor channel was updated or timestamp is invalid, reset and skip
    if ((!timeslot.lowGPresent && !timeslot.highGPresent &&
         !timeslot.gyrPresent && !timeslot.tempPresent) ||
        timeslot.slotTimestamp == 0)
    {
        timeslot.lowGPresent  = false;
        timeslot.highGPresent = false;
        timeslot.gyrPresent   = false;
        timeslot.tempPresent  = false;
        return;
    }

    // Populate each channel: use the newly received data if present in this
    // timeslot, otherwise fall back to `lastValidSample`
    if (timeslot.lowGPresent)
    {
        timeslot.data.lowG.accelerationTimestamp = timeslot.slotTimestamp;
    }
    else
    {
        timeslot.data.lowG = lastValidSample.lowG;
    }

    if (timeslot.highGPresent)
    {
        timeslot.data.highG.accelerationTimestamp = timeslot.slotTimestamp;
    }
    else
    {
        timeslot.data.highG = lastValidSample.highG;
    }

    if (timeslot.gyrPresent)
    {
        timeslot.data.gyroscope.angularSpeedTimestamp = timeslot.slotTimestamp;
    }
    else
    {
        timeslot.data.gyroscope = lastValidSample.gyroscope;
    }

    if (timeslot.tempPresent)
    {
        timeslot.data.temperature.temperatureTimestamp = timeslot.slotTimestamp;
    }
    else
    {
        timeslot.data.temperature = lastValidSample.temperature;
    }

    // Reset presence flags for future reuse of the slot
    timeslot.lowGPresent  = false;
    timeslot.highGPresent = false;
    timeslot.gyrPresent   = false;
    timeslot.tempPresent  = false;

    // Discard duplicate timestamps or prevent out-of-bounds writes
    if (fifoIdx >= LSM6DSV320XDefs::FIFO_SIZE ||
        (fifoIdx > 0 && timeslot.slotTimestamp ==
                            lastFifo[fifoIdx - 1].lowG.accelerationTimestamp &&
                            timeslot.slotTimestamp ==
                            lastFifo[fifoIdx - 1].highG.accelerationTimestamp &&
                            timeslot.slotTimestamp ==
                            lastFifo[fifoIdx - 1].gyroscope.angularSpeedTimestamp))
    {
        return;
    }

    // Push into driver FIFO and update lastValidSample
    lastFifo[fifoIdx] = timeslot.data;
    ++fifoIdx;

    lastValidSample = timeslot.data;
}

uint16_t LSM6DSV320X::unreadDataInFifo()
{
    SPITransaction spi{spiSlave};

    uint16_t unread = spi.readRegister(LSM6DSV320XDefs::Registers::FIFO_STATUS1);
    unread |= (static_cast<uint16_t>(
                   spi.readRegister(LSM6DSV320XDefs::Registers::FIFO_STATUS2) &
                   0x01)
               << 8);
    return unread;
}

}  // namespace Boardcore