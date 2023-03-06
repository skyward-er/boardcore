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
 * - Automatic bus recovery before each operation;
 * - Method in order to read a one-byte register without issuing a stop
 * condition (unique transaction).
 */
class I2C
{
public:
    /**
     * @brief Constructor for the I2C high-level driver.
     *
     * It uses the low-level I2C constructor that internally initializes the
     * pins so that they are always set to ALTERNATE_OD mode with Alternate
     * Function 4 (the usual AF of I2C pins). Thanks to this we avoid the
     * possibility of short circuits between master and slaves when they both
     * drive the same bus on two different logical values.
     *
     * @param i2c Structure that represents the I2C peripheral.
     * @param scl Serial clock GpioPin of the relative I2C peripheral.
     * @param sda Serial data GpioPin of the relative I2C peripheral.
     */
    I2C(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda);

    /**
     * @brief Non blocking read operation to read nBytes.
     *
     * This method, if necessary, flushes the bus before the read operation is
     * performed. In case of an error during the communication, this method
     * returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes Number of bytes to read.
     * @returns True if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(const I2CDriver::I2CSlaveConfig &slaveConfig,
                            void *buffer, size_t nBytes);

    /**
     * @brief Non blocking write operation to write nBytes.
     *
     * This method, if necessary, flushes the bus before the read operation is
     * performed. In case of an error during the communication, this method
     * returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes Number of bytes to send.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(const I2CDriver::I2CSlaveConfig &slaveConfig,
                             const void *buffer, size_t nBytes);

    /**
     * @brief Non blocking operation to read a 1-byte register from a slave.
     *
     * This method, if necessary, flushes the bus before the read operation is
     * performed. In case of an error during the communication, this method
     * returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param registerAddress Byte that represents the address of the register.
     * @param registerContent Where to store the content of the register.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool readRegister(
        const I2CDriver::I2CSlaveConfig &slaveConfig,
        const uint8_t &registerAddress, uint8_t &registerContent);

    /**
     * @brief Non blocking operation to write a 1-byte register from a slave.
     *
     * This method, if necessary, flushes the bus before the write operation is
     * performed. In case of an error during the communication, this method
     * returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param registerAddress Byte that represents the address of the register.
     * @param registerContent The content to write on the register.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool writeRegister(
        const I2CDriver::I2CSlaveConfig &slaveConfig,
        const uint8_t &registerAddress, const uint8_t registerContent);

    /**
     * @brief Non blocking operation to read n-bytes from register from a slave.
     *
     * This method, if necessary, flushes the bus before the read operation is
     * performed. In case of an error during the communication, this method
     * returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param registerAddress Byte that represents the address of the register.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes Number of bytes to read.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool readFromRegister(
        const I2CDriver::I2CSlaveConfig &slaveConfig,
        const uint8_t &registerAddress, void *buffer, size_t nBytes);

    /**
     * @brief Non blocking operation to check if a slave is available.
     *
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @returns True if the device is available, false otherwise.
     */
    [[nodiscard]] bool probe(const I2CDriver::I2CSlaveConfig &slaveConfig);

    /**
     * @brief Returns the last errors happened in the communication.
     *
     * For checking if a specific error occurred in the last transaction you can
     * do `if(getLastError() & Errors::<error-to-check>)`. Do not use `==` to
     * check for errors because there could be more errors at once. To check if
     * no errors occurred use `if(getLastError() == Errors::NO_ERROR)` or simply
     * `if(!getLastError())`
     *
     * @return A bit sequence where the bits set correspond to the last errors
     * occurred in the peripheral (see the `I2CDriver::Errors` enum to get the
     * correspondence between bit position and error reported).
     */
    uint16_t getLastError();

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
     *
     * @param i2c Structure that represents the I2C peripheral.
     * @param scl Serial clock GpioPin of the relative I2C peripheral.
     * @param sda Serial data GpioPin of the relative I2C peripheral.
     */
    SyncedI2C(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda);

    /**
     * @brief Read operation to read nBytes.
     *
     * This method could have to wait that no other thread is trying to do some
     * operation on the bus. In case of an error during the communication, this
     * method returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes Number of bytes to read.
     * @returns True if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(const I2CDriver::I2CSlaveConfig &slaveConfig,
                            void *buffer, size_t nBytes);

    /**
     * @brief Write operation to write nBytes.
     *
     * This method could have to wait that no other thread is trying to do some
     * operation on the bus. In case of an error during the communication, this
     * method returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes Number of bytes to send.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(const I2CDriver::I2CSlaveConfig &slaveConfig,
                             const void *buffer, size_t nBytes);

    /**
     * @brief Read a one-byte register from the device.
     *
     * This method could have to wait that no other thread is trying to do some
     * operation on the bus. In case of an error during the communication, this
     * method returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param registerAddress Byte that represents the address of the register.
     * @param registerContent Where to store the content of the register.
     * @returns True if the read is successful, false otherwise.
     */
    [[nodiscard]] bool readRegister(
        const I2CDriver::I2CSlaveConfig &slaveConfig,
        const uint8_t &registerAddress, uint8_t &registerContent);

    /**
     * @brief Write a one-byte register from the device.
     *
     * This method could have to wait that no other thread is trying to do some
     * operation on the bus. In case of an error during the communication, this
     * method returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param registerAddress Byte that represents the address of the register.
     * @param registerContent The content to write on the register.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool writeRegister(
        const I2CDriver::I2CSlaveConfig &slaveConfig,
        const uint8_t &registerAddress, const uint8_t registerContent);

    /**
     * @brief Read n-bytes from register from a slave.
     *
     * This method could have to wait that no other thread is trying to do some
     * operation on the bus. In case of an error during the communication, this
     * method returns false immediately.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param registerAddress Byte that represents the address of the register.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes Number of bytes to read.
     * @returns True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool readFromRegister(
        const I2CDriver::I2CSlaveConfig &slaveConfig,
        const uint8_t &registerAddress, void *buffer, size_t nBytes);

    /**
     * @brief Check if a slave is available.
     *
     * This method could have to wait that no other thread is trying to do some
     * operation on the bus.
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @returns true if the device is available, false otherwise.
     */
    [[nodiscard]] bool probe(const I2CDriver::I2CSlaveConfig &slaveConfig);

    /**
     * @brief Returns the last errors happened in the communication.
     *
     * For checking if a specific error occurred in the last transaction you can
     * do `if(getLastError() & Errors::<error-to-check>)`. Do not use `==` to
     * check for errors because there could be more errors at once. To check if
     * no errors occurred use `if(getLastError() == Errors::NO_ERROR)` or simply
     * `if(!getLastError())`
     *
     * @return A bit sequence where the bits set correspond to the last errors
     * occurred in the peripheral (see the `I2CDriver::Errors` enum to get the
     * correspondence between bit position and error reported).
     */
    uint16_t getLastError();

private:
    miosix::FastMutex mutex;  ///< Mutex for rx/tx
};

}  // namespace Boardcore