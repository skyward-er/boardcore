#pragma once


#include "LSM6DSV320X.h"

#include <assert.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/Constants.h>
#include <utils/Debug.h>

#include <cmath>

namespace Boardcore
{


    // contructor, to be checked!!
LSM6DSV320X::LSM6DSV320X(SPIBusInterface& bus, miosix::GpioPin csPin,
                         SPIBusConfig busConfiguration, LSM6DSV320XConfig& configuration)
    : spiSlave(bus, csPin, busConfiguration), config(configuration)
{
    isInit = false;

    // check that the watermark value is suitable
    config.fifoWatermark = std::min(config.fifoWatermark, uint16_t(170));

    // check that ACC_ODR is set to HZ_1_6 only if OPERATING_MODE is equal to
    // NORMAL
    if (config.odrAcc == LSM6DSRXConfig::ACC_ODR::HZ_1_6)
        config.opModeAcc = LSM6DSRXConfig::OPERATING_MODE::NORMAL;


}

bool LSM6DSV320X::init()
{
    if (isInit) 
    {
        LOG_ERR(logger, "Sensor already initialized");
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }


    if (!checkWhoAmI())
    {
        LOG_ERR(logger, "WHO_AM_I check failed");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    // set bdu and multiple spi read/write
    {
        SPITransaction spiTransaction{spiSlave};

        uint8_t value = static_cast<uint8_t>(config.bdu) << 6;  // set bdu
        value |= 1 << 2;  // set multiple spi read/write

        spiTransaction.writeRegister(LSM6DSV320XDefs::CTRL3, value);
    }

    initLowG();
    initHighG();
    initGyroscope();
    initFifo();
    initInterrupts();

    // set timestamps (todo)

    isInit = true;
    lastError = SensorErrors::NO_ERRORS;
    return true;


}

void LSM6DSV320X::initLowG()
{
}

void LSM6DSV320X::initHighG()
{
}

void LSM6DSV320X::initGyroscope()
{
}

void LSM6DSV320X::initFifo()
{
}

void LSM6DSV320X::initInterrupts()
{
}

void