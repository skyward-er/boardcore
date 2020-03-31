/**
 *
 *
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "SPIBusInterface.h"

/**
 * @brief Provides high-level access to the SPI Bus for a single transaction.
 * To make sure the bus is properly configured for the provided slave, you have
 * to create a new instance of this class for every transaction, as the bus is
 * configured upon instantiation.
 *
 * @warning DO NOT store an instance of this class for later use, as the bus may
 * be incorrectly configured by then.
 *
 * Example transaction:
 *
 * {
 *     // Transaction begin:
 *     SPITransaction spi(bus, cs, config); // Configures the bus with the
 *                                          // provided parameters.
 *
 *     spi.write(REG_EX, 0x56); // writes data to REG_EX
 *     uint8_t reg2 = spi.read(REG_EX_2); // reads from REG_EX_2
 *
 *     // ...As many read/writes as you wish...
 *
 *     // transaction end. SPITransaction object is destructed and the bus is
 *     // freed for use by someone else
 * }
 */
class SPITransaction
{
public:
    /**
     * @brief Instatiates a new SPITransaction, configuring the bus with the
     * provided parameters
     *
     * @param    slave     Slave to communicate with
     */
    SPITransaction(SPISlave slave);

    /**
     * @brief Instatiates a new SPITransaction, configuring the bus with the
     * provided parameters
     *
     * @param    bus       Bus to communicate on
     * @param    cs        Chip select of the slave to communicate to
     * @param    config    Configuration of the bus for the selected slave
     */
    SPITransaction(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config);

    ~SPITransaction();

    // Delete copy/move contructors/operators
    SPITransaction(const SPITransaction&) = delete;
    SPITransaction& operator=(const SPITransaction&) = delete;

    SPITransaction(SPITransaction&&) = delete;
    SPITransaction& operator=(SPITransaction&&) = delete;

    /**
     * @brief Writes a command \p cmd to the bus
     *
     * @param    cmd     Command to write on the bus
     */
    void write(uint8_t cmd);

    /**
     * @brief Writes \p val into the \p reg register
     *
     * @param    reg     Slave device register
     * @param    val     Value to be written in the register
     */
    void write(uint8_t reg, uint8_t val);

    /**
     * @brief Writes \p size bytes into the \p reg register
     *
     * @param    reg       Slave device register
     * @param    data      Data to be written
     * @param    size      Number of bytes to be written
     */
    void write(uint8_t reg, uint8_t* data, size_t size);

    /**
     * @brief Writes \p bytes on the bus
     *
     * @param    data      Bytes to be written
     * @param    size      Number of bytes to be written
     */
    void write(uint8_t* data, size_t size);

    /**
     * @brief Read the contents of the \p reg register
     *
     * @param    reg       Slave device register
     * @param    set_read_bit Wether to set the read bit to 1 (MSB of reg)
     *                        (default = true).
     */
    uint8_t read(uint8_t reg, bool set_read_bit = true);

    /**
     * @brief Reads \p size bytes from the \p reg register
     *
     * @param    reg    Slave device register
     * @param    data   Buffer where read bytes will be stored
     * @param    size   Number of bytes to read
     * @param    set_read_bit Wether to set the read bit to 1 (MSB of reg)
     *                        (default = true).
     */
    void read(uint8_t reg, uint8_t* data, size_t size,
              bool set_read_bit = true);

    /**
     * @brief Reads \p size bytes from the bus
     *
     * @param    data   Buffer where read bytes will be stored
     * @param    size   Number of bytes to read
     */
    void read(uint8_t* data, size_t size);

    /**
     * @brief Full duplex transfer: \p data is written on the bus and its
     *        contents are replaced with the received bytes.
     *
     * @param    data      Buffer containign data to be transfered
     * @param    size      Number of bytes to be transfer
     */
    void transfer(uint8_t* data, size_t size);

    /**
     * @brief Returns the underlying bus for low level access
     *
     * @return  SPIBusInterface associated with this transaction
     */
    SPIBusInterface& getBus() { return bus; }

private:
    SPIBusInterface& bus;
    GpioPin cs;
};