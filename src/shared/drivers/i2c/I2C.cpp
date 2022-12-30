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

I2C::I2C(I2C_TypeDef *i2c, I2CDriver::Speed speed,
         I2CDriver::Addressing addressing, miosix::GpioPin scl,
         miosix::GpioPin sda)
    : i2c(i2c, speed, addressing, scl, sda)
{
}

bool I2C::read(uint16_t slaveAddress, void *buffer, size_t nBytes)
{
    i2c.flushBus();
    return i2c.read(slaveAddress, buffer, nBytes);
}

bool I2C::write(uint16_t slaveAddress, const void *buffer, size_t nBytes)
{
    i2c.flushBus();
    return i2c.write(slaveAddress, buffer, nBytes);
}

bool I2C::readRegister(uint16_t slaveAddress, const uint8_t registerAddress,
                       uint8_t &registerContent)
{
    i2c.flushBus();
    return i2c.write(slaveAddress, &registerAddress, 1, false) &&
           i2c.read(slaveAddress, &registerContent, 1);
}

SyncedI2C::SyncedI2C(I2C_TypeDef *i2c, I2CDriver::Speed speed,
                     I2CDriver::Addressing addressing, miosix::GpioPin scl,
                     miosix::GpioPin sda)
    : I2C(i2c, speed, addressing, scl, sda)
{
}

bool SyncedI2C::read(uint16_t slaveAddress, void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return i2c.read(slaveAddress, buffer, nBytes);
}

bool SyncedI2C::write(uint16_t slaveAddress, const void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return i2c.write(slaveAddress, buffer, nBytes);
}

bool SyncedI2C::readRegister(uint16_t slaveAddress,
                             const uint8_t registerAddress,
                             uint8_t registerContent)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    return i2c.write(slaveAddress, &registerAddress, 1, false) &&
           i2c.read(slaveAddress, &registerContent, 1);
}

}  // namespace Boardcore