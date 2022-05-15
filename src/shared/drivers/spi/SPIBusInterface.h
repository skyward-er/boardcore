/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#pragma once

#include <interfaces-impl/gpio_impl.h>

#include "SPI.h"

#ifndef USE_MOCK_PERIPHERALS
using GpioType = miosix::GpioPin;
#else
#include <utils/TestUtils/MockGpioPin.h>
using GpioType = Boardcore::MockGpioPin;
#endif

namespace Boardcore
{

/**
 * @brief SPI Bus configuration for a specific slave.
 *
 * See slave datasheet for information on how to populate this struct.
 */
struct SPIBusConfig
{
    ///< Peripheral clock division
    SPI::ClockDivider clockDivider;

    ///< Clock polarity and phace configuration
    SPI::Mode mode;

    ///< MSB or LSB first
    SPI::BitOrder bitOrder;

    ///< How long to wait before starting a trasmission after CS is set (us)
    unsigned int csSetupTimeUs;

    ///< How long to hold cs after the end of a trasmission (us)
    unsigned int csHoldTimeUs;

    SPIBusConfig(SPI::ClockDivider clockDivider = SPI::ClockDivider::DIV_256,
                 SPI::Mode mode                 = SPI::Mode::MODE_0,
                 SPI::BitOrder bitOrder         = SPI::BitOrder::MSB_FIRST,
                 unsigned int csSetupTimeUs = 0, unsigned int csHoldTimeUs = 0)
        : clockDivider(clockDivider), mode(mode), bitOrder(bitOrder),
          csSetupTimeUs(csSetupTimeUs), csHoldTimeUs(csHoldTimeUs)
    {
    }

    /**
     * @brief Custom comparison operator.
     */
    bool operator==(const SPIBusConfig& other) const
    {
        return clockDivider == other.clockDivider && mode == other.mode &&
               bitOrder == other.bitOrder &&
               csSetupTimeUs == other.csSetupTimeUs &&
               csHoldTimeUs == other.csHoldTimeUs;
    }

    bool operator!=(const SPIBusConfig& other) const
    {
        return !(*this == other);
    }
};

/**
 * @brief Interface for low level access of a SPI bus as a master.
 */
class SPIBusInterface
{
public:
    SPIBusInterface() {}

    ///< Delete copy/move contructors/operators.
    SPIBusInterface(const SPIBusInterface&) = delete;
    SPIBusInterface& operator=(const SPIBusInterface&) = delete;
    SPIBusInterface(SPIBusInterface&&)                 = delete;
    SPIBusInterface& operator=(SPIBusInterface&&) = delete;

    /**
     * @brief Configures the bus with the provided configuration parameters.
     *
     * Call this before every transaction to configure the bus.
     *
     * @param config Configuration parameters.
     */
    virtual void configure(SPIBusConfig config) = 0;

    /**
     * @brief Selects the slave.
     *
     * @param cs Chip select pin for the slave.
     */
    virtual void select(GpioType& cs) = 0;

    /**
     * @brief Deselects the slave.
     *
     * @param cs Chip select pin for the slave.
     */
    virtual void deselect(GpioType& cs) = 0;

    // Read, write and transfer operations

    /**
     * @brief Reads a single byte from the bus.
     *
     * @return Byte read from the bus.
     */
    virtual uint8_t read() = 0;

    /**
     * @brief Reads a single half word from the bus.
     *
     * @return Half word read from the bus.
     */
    virtual uint16_t read16() = 0;

    /**
     * @brief Reads multiple bytes from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    virtual void read(uint8_t* data, size_t size) = 0;

    /**
     * @brief Reads multiple half words from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    virtual void read(uint16_t* data, size_t size) = 0;

    /**
     * @brief Writes a single byte to the bus.
     *
     * @param data Byte to write.
     */
    virtual void write(uint8_t data) = 0;

    /**
     * @brief Writes a single half word to the bus.
     *
     * @param data Half word to write.
     */
    virtual void write(uint16_t data) = 0;

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    virtual void write(uint8_t* data, size_t size) = 0;

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    virtual void write(uint16_t* data, size_t size) = 0;

    /**
     * @brief Full duplex transmission of one byte on the bus.
     *
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    virtual uint8_t transfer(uint8_t data) = 0;

    /**
     * @brief Full duplex transmission of one half word on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    virtual uint16_t transfer(uint16_t data) = 0;

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer.
     */
    virtual void transfer(uint8_t* data, size_t size) = 0;

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer.
     */
    virtual void transfer(uint16_t* data, size_t size) = 0;
};

/**
 * @brief Contains information about a single SPI slave device.
 */
struct SPISlave
{
    SPIBusInterface& bus;  ///< Bus on which the slave is connected.
    SPIBusConfig config;   ///< How the bus should be configured to communicate
                           ///< with the slave.
    GpioType cs;           ///< Chip select pin

    SPISlave(SPIBusInterface& bus, GpioType cs, SPIBusConfig config)
        : bus(bus), config(config), cs(cs)
    {
    }
};

/**
 * @brief RAII Interface for SPI bus acquisition
 *
 */
class SPIAcquireLock
{
public:
    explicit SPIAcquireLock(SPISlave slave)
        : SPIAcquireLock(slave.bus, slave.config)
    {
    }

    SPIAcquireLock(SPIBusInterface& bus, SPIBusConfig cfg) : bus(bus)
    {
        bus.configure(cfg);
    }

private:
    SPIBusInterface& bus;
};

/**
 * @brief RAII Interface for SPI chip selection.
 */
class SPISelectLock
{
public:
    explicit SPISelectLock(SPISlave slave) : SPISelectLock(slave.bus, slave.cs)
    {
    }

    SPISelectLock(SPIBusInterface& bus, GpioType cs) : bus(bus), cs(cs)
    {
        bus.select(cs);
    }

    ~SPISelectLock() { bus.deselect(cs); }

private:
    SPIBusInterface& bus;
    GpioType& cs;
};

}  // namespace Boardcore
