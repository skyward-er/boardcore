#include "FDC2214RGHR.h"

namespace Boardcore
{

FDC2214RGHR::FDC2214RGHR(I2C& i2c, I2CDriver::I2CSlaveConfig i2cConfig)
    : bus(i2c), slaveConfig(i2cConfig) {};

bool FDC2214RGHR::init()
{
    if (!checkWhoAmI())
    {
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    // TODO: Set registers

    initialized = true;
}

bool FDC2214RGHR::checkWhoAmI()
{
    uint16_t whoAmIValue;

    if (bus.readRegister16(slaveConfig, DEVICE_ID, whoAmIValue))
    {
        return whoAmIValue == DEVICE_ID_VAL;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }
}

bool FDC2214RGHR::setReferenceCount(FDC2214RGHR::Channel channel,
                                    uint16_t value)
{
    if (!initialized)
    {
        lastError = SensorErrors::NOT_INIT;
        return false;
    }

    if (!bus.writeRegister16(
            slaveConfig, RCOUNT_BASE + static_cast<uint8_t>(channel), value))
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }

    return true;
}

bool FDC2214RGHR::setSettleCount(FDC2214RGHR::Channel channel, uint16_t value)
{
    if (!initialized)
    {
        lastError = SensorErrors::NOT_INIT;
        return false;
    }

    if (!bus.writeRegister16(slaveConfig,
                             SETTLECOUNT_BASE + static_cast<uint8_t>(channel),
                             value))
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }

    return true;
}

bool FDC2214RGHR::setClockDividers(FDC2214RGHR::Channel channel, uint16_t value)
{
    if (!initialized)
    {
        lastError = SensorErrors::NOT_INIT;
        return false;
    }

    if (!bus.writeRegister16(
            slaveConfig, CLOCK_DIVIDERS_BASE + static_cast<uint8_t>(channel),
            value))
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }

    return true;
}

FDC2214RGHR::CapacityData sampleImpl()

}  // namespace Boardcore
