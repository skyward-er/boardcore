/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <miosix.h>

#include <cstdint>
#include <cstdio>

using miosix::delayUs;
using miosix::GpioPin;

/**
 * @brief SPI baud rate selection parameter.
 * SPI clock frequency will be equal to the SPI peripheral bus clock speed
 * divided by the value specified in this enum.
 *
 * Eg: DIV_2 --> spi clock freq = f_PCLK / 2
 *
 * See SPI->CR1 on the datasheet for further information.
 */
enum class SPIBaudRate
{
    DIV_2   = 0,
    DIV_4   = 1,
    DIV_8   = 2,
    DIV_16  = 3,
    DIV_32  = 4,
    DIV_64  = 5,
    DIV_128 = 6,
    DIV_256 = 7,
};

/**
 * @brief SPI Bus configuration for a specific slave.
 * See slave datasheet for information on how to populate this struct
 */
struct SPIBusConfig
{
    SPIBaudRate br = SPIBaudRate::DIV_2;  ///> Peripheral clock division
    uint8_t cpol   = 0;                   ///> Clock polarity (0 - 1)
    uint8_t cpha   = 0;                   ///> Clock phase (0 - 1)
    bool lsb_first = false;               ///> MSB or LSB first

    unsigned int cs_setup_time_us = 0;  ///> How long to wait before starting a
                                        ///> a trasmission after CS is set (us)
    unsigned int cs_hold_time_us = 0;   ///> How long to hold cs after the end
                                        ///> of a trasmission (us)

    // Custom comparison operator
    bool operator==(const SPIBusConfig& other) const
    {
        // Valid if the struct does not contain pointers!
        return memcmp(this, &other, sizeof(SPIBusConfig)) == 0;
    }

    bool operator!=(const SPIBusConfig& other) const
    {
        return !(*this == other);
    }
};

/**
 * @brief Interface for low level access of a SPI bus
 */
class SPIBusInterface
{
public:
    SPIBusInterface() {}

    ~SPIBusInterface() {}

    // Delete copy/move contructors/operators
    SPIBusInterface(const SPIBusInterface&) = delete;
    SPIBusInterface& operator=(const SPIBusInterface&) = delete;

    SPIBusInterface(SPIBusInterface&&) = delete;
    SPIBusInterface& operator=(SPIBusInterface&&) = delete;

    /**
     * @brief Writes a single \p byte to the bus.
     *
     * @param    byte Byte to write
     */
    virtual void write(uint8_t byte) = 0;

    /**
     * @brief Writes \p data to the bus.
     *
     * @param    data Buffer containing data to write
     * @param    size Number of bytes to write
     */
    virtual void write(uint8_t* data, size_t size) = 0;

    /**
     * @brief Reads a single byte from the bus.
     * @return Byte read from the bus
     */
    virtual uint8_t read() = 0;

    /**
     * @brief Reads \p size bytes from the SPI bus, putting them in \p data.
     *
     * @param    data Buffer to be filled with received data
     * @param    size Number of bytes to receive
     */
    virtual void read(uint8_t* data, size_t size) = 0;

    /**
     * @brief Full duplex transmission on the SPI bus.
     * A \p byte is written on the bus and a byte is read and returned
     *
     * @param    byte Byte to write
     * @return Data read from the bus
     */
    virtual uint8_t transfer(uint8_t byte) = 0;
    
    /**
     * @brief Full duplex transmission on the SPI bus.
     * \p data is written on the bus and its contents are then replaced with the
     * received bytes.
     *
     * @param    data Buffer containing data to transfer
     * @param    size Number of bytes to transfer
     */
    virtual void transfer(uint8_t* data, size_t size) = 0;

    /**
     * @brief Selects the slave
     *
     * @param    cs Chip select pin for the slave
     */
    virtual void select(GpioPin& cs) = 0;

    /**
     * @brief Deselects the slave
     *
     * @param    cs Chip select pin for the slave
     * @return
     */
    virtual void deselect(GpioPin& cs) = 0;

    /**
     * @brief Configures the bus with the provided configuration parameters.
     *
     * @param    config    Configuration parameters
     * @return
     */
    virtual void configure(SPIBusConfig config) = 0;
};

/**
 * @brief Contains information about a single SPI slave device.
 */
struct SPISlave
{
    SPIBusInterface& bus;  ///> Bus on which the slave is connected

    SPIBusConfig config;  ///> How the bus should be configured to communicate
                          ///> with the slave.
    GpioPin cs;           ///> Chip select pin

    SPISlave(SPIBusInterface& bus, GpioPin cs) : bus(bus), cs(cs) {}

    SPISlave(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config)
        : bus(bus), config(config), cs(cs)
    {
    }
};
