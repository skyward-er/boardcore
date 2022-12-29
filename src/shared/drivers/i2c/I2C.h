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

#include "I2CDriver.h"
namespace Boardcore
{
class I2C
{
public:
    I2C(I2C_TypeDef *i2c, I2CDriver::Speed speed,
        I2CDriver::Addressing addressing, miosix::GpioPin scl,
        miosix::GpioPin sda);

    [[nodiscard]] bool read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    [[nodiscard]] bool write(uint16_t slaveAddress, const void *buffer,
                             size_t nBytes);

    [[nodiscard]] bool readRegister(uint16_t slaveAddress,
                                    const uint8_t registerAddress,
                                    uint8_t &registerContent);

protected:
    I2CDriver i2c;
};

/**
 * @brief Thread safe version of the I2C driver.
 */
class SyncedI2C : public I2C
{
public:
    SyncedI2C(I2C_TypeDef *i2c, I2CDriver::Speed speed,
              I2CDriver::Addressing addressing, miosix::GpioPin scl,
              miosix::GpioPin sda);

    [[nodiscard]] bool read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    [[nodiscard]] bool write(uint16_t slaveAddress, const void *buffer,
                             size_t nBytes);

    [[nodiscard]] bool readRegister(uint16_t slaveAddress,
                                    const uint8_t registerAddress,
                                    uint8_t registerContent);

private:
    miosix::FastMutex mutex;  ///< mutex for rx/tx
};

}  // namespace Boardcore