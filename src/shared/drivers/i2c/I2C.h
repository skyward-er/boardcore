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

/**
 * @brief High level driver for the I2C peripherals.
 *
 * This driver is NOT thread safe. It implements high level functionalities such
 * as:
 * - automatic bus recovery before each operation.
 * - Method in order to read a one-byte register without issuing a stop
 * condition (unique transaction).
 */
class I2C
{
public:
    /**
     * @brief Constructor for the I2C high-level driver.
     * @param i2c structure that represents the I2C peripheral
     * @param speed the speed mode of the I2C communication
     * @param addressing The addressing mode used in the I2C communication
     * @param scl Serial clock GpioPin of the relative I2C peripheral
     * @param sda Serial data GpioPin of the relative I2C peripheral
     */
    I2C(I2C_TypeDef *i2c, I2CDriver::Speed speed,
        I2CDriver::Addressing addressing, miosix::GpioPin scl,
        miosix::GpioPin sda);

    /**
     * @brief Non blocking read operation to read nBytes. This method, if
     * necessary, flushes the bus before the read operation is performed. In
     * case of an error during the communication, this method returns false
     * immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes number of bytes to read.
     * @returns true if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    /**
     * @brief Non blocking write operation to write nBytes. This method, if
     * necessary, flushes the bus before the read operation is performed. In
     * case of an error during the communication, this method returns false
     * immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes number of bytes to send.
     * @returns true if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(uint16_t slaveAddress, const void *buffer,
                             size_t nBytes);

    /**
     * @brief Non blocking operation to read a 1-byte register from the device.
     * This method, if necessary, flushes the bus before the read operation is
     * performed. In case of an error during the communication, this method
     * returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param registerAddress byte that represents the address of the register
     * @param registerContent where to store the content of the register
     * @returns true if the write is successful, false otherwise.
     */
    [[nodiscard]] bool readRegister(uint16_t slaveAddress,
                                    const uint8_t registerAddress,
                                    uint8_t &registerContent);

protected:
    I2CDriver i2c;  ///< Instance of I2C low-level driver
};

/**
 * @brief Thread safe version of the I2C high-level driver.
 */
class SyncedI2C : public I2C
{
public:
    /**
     * @brief Constructor for the synced I2C high-level driver.
     * @param i2c structure that represents the I2C peripheral
     * @param speed the speed mode of the I2C communication
     * @param addressing The addressing mode used in the I2C communication
     * @param scl Serial clock GpioPin of the relative I2C peripheral
     * @param sda Serial data GpioPin of the relative I2C peripheral
     */
    SyncedI2C(I2C_TypeDef *i2c, I2CDriver::Speed speed,
              I2CDriver::Addressing addressing, miosix::GpioPin scl,
              miosix::GpioPin sda);

    /**
     * @brief Read operation to read nBytes. This method could have to wait that
     * no other thread is trying to do some operation on the bus. In case of an
     * error during the communication, this method returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes number of bytes to read.
     * @returns true if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    /**
     * @brief Write operation to write nBytes. This method could have to wait
     * that no other thread is trying to do some operation on the bus. In case
     * of an error during the communication, this method returns false
     * immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes number of bytes to send.
     * @returns true if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(uint16_t slaveAddress, const void *buffer,
                             size_t nBytes);

    /**
     * @brief Read a one-byte register from the device. This method could have
     * to wait that no other thread is trying to do some operation on the bus.
     * In case of an error during the communication, this method returns false
     * immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param registerAddress byte that represents the address of the register
     * @param registerContent where to store the content of the register
     * @returns true if the write is successful, false otherwise.
     */
    [[nodiscard]] bool readRegister(uint16_t slaveAddress,
                                    const uint8_t registerAddress,
                                    uint8_t registerContent);

private:
    miosix::FastMutex mutex;  ///< mutex for rx/tx
};

}  // namespace Boardcore