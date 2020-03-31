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

class SPITransaction;

/**
 * @brief SPI Clock divider.
 * SPI clock frequency will be equal to the SPI peripheral bus clock speed (see
 * datasheet) divided by the value specified in this enum.
 *
 * Eg: DIV_2 --> spi clock freq = f_PCLK / 2
 *
 * See register CR1  of the SPI peripheral on the reference manual for further
 * information.
 */
enum class SPIClockDivider : uint8_t
{
    DIV2   = 0x00,
    DIV4   = 0x08,
    DIV8   = 0x10,
    DIV16  = 0x18,
    DIV32  = 0x20,
    DIV64  = 0x28,
    DIV128 = 0x30,
    DIV256 = 0x38,
};

/**
 * @brief SPI Mode.
 * See slave device datasheet for information on which one to use.
 */
enum class SPIMode : uint8_t
{
    MODE0 = 0,  ///> CPOL = 0, CPHA = 0
    MODE1 = 1,  ///> CPOL = 0, CPHA = 1
    MODE2 = 2,  ///> CPOL = 1, CPHA = 0
    MODE3 = 3   ///> CPOL = 1, CPHA = 1
};

/**
 * @brief SPI Bit Order
 * See register CR1  of the SPI peripheral on the reference manual for further
 * information.
 */
enum class SPIBitOrder : uint8_t
{
    MSB_FIRST = 0,
    LSB_FIRST = 0x80
};

/**
 * @brief SPI Bus configuration for a specific slave.
 * See slave datasheet for information on how to populate this struct
 */
struct SPIBusConfig
{
    SPIClockDivider clock_div =
        SPIClockDivider::DIV2;               ///> Peripheral clock division
    SPIMode mode          = SPIMode::MODE0;  ///> Clock polarity (0 - 1)
    SPIBitOrder bit_order = SPIBitOrder::MSB_FIRST;  ///> MSB or LSB first

    unsigned int cs_setup_time_us = 0;  ///> How long to wait before starting a
                                        ///> a trasmission after CS is set (us)
    unsigned int cs_hold_time_us = 0;   ///> How long to hold cs after the end
                                        ///> of a trasmission (us)

    // Custom comparison operator
    bool operator==(const SPIBusConfig& other) const
    {
        // Compare member-by-member
        // clang-format off
        return  clock_div == other.clock_div 
             && mode == other.mode
             && bit_order == other.bit_order
             && cs_setup_time_us == other.cs_setup_time_us 
             && cs_setup_time_us == other.cs_hold_time_us;
        // clang-format on
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
    friend class SPITransaction;
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

private:
    bool locked = false; // For use by SPITransaction
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

    SPISlave(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config)
        : bus(bus), config(config), cs(cs)
    {
    }
};
