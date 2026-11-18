#include "FDC2214RGHR.h"

namespace Boardcore
{
bool FDC2214RGHR::init()
{
    if (!checkWhoAmI())
    {
        // TODO: andiamo a sovrascrivere il SensorErrors::BUS_FAULT, non è un
        // problema?
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }
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

}  // namespace Boardcore
