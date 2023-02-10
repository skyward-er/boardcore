/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "I2C.h"

namespace Boardcore
{

I2C::I2C(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda)
    : i2c(i2c, scl, sda)
{
}

bool I2C::read(const I2CDriver::I2CSlaveConfig &slaveConfig, void *buffer,
               size_t nBytes)
{
    i2c.flushBus();
    return i2c.read(slaveConfig, buffer, nBytes);
}

bool I2C::write(const I2CDriver::I2CSlaveConfig &slaveConfig,
                const void *buffer, size_t nBytes)
{
    i2c.flushBus();
    return i2c.write(slaveConfig, buffer, nBytes);
}

bool I2C::readRegister(const I2CDriver::I2CSlaveConfig &slaveConfig,
                       const uint8_t registerAddress, uint8_t &registerContent)
{
    i2c.flushBus();
    return i2c.write(slaveConfig, &registerAddress, 1, false) &&
           i2c.read(slaveConfig, &registerContent, 1);
}

bool I2C::probe(const I2CDriver::I2CSlaveConfig &slaveConfig)
{
    i2c.flushBus();
    return i2c.write(slaveConfig, nullptr, 0);
}

uint16_t I2C::getLastError() { return i2c.getLastError(); }

SyncedI2C::SyncedI2C(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda)
    : I2C(i2c, scl, sda)
{
}

bool SyncedI2C::read(const I2CDriver::I2CSlaveConfig &slaveConfig, void *buffer,
                     size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return I2C::read(slaveConfig, buffer, nBytes);
}

bool SyncedI2C::write(const I2CDriver::I2CSlaveConfig &slaveConfig,
                      const void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return I2C::write(slaveConfig, buffer, nBytes);
}

bool SyncedI2C::readRegister(const I2CDriver::I2CSlaveConfig &slaveConfig,
                             const uint8_t registerAddress,
                             uint8_t registerContent)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return I2C::readRegister(slaveConfig, registerAddress, registerContent);
}

bool SyncedI2C::probe(const I2CDriver::I2CSlaveConfig &slaveConfig)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return I2C::probe(slaveConfig);
}

uint16_t SyncedI2C::getLastError()
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return i2c.getLastError();
}

}  // namespace Boardcore