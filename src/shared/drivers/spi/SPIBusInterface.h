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
#include <stddef.h>

#include "SPIDefs.h"

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

    ///< Clock polarity and phase configuration
    SPI::Mode mode;

    ///< MSB or LSB first
    SPI::Order bitOrder;

    ///< MSB or LSB first
    SPI::Order byteOrder;

    ///< Write bit behaviour, default high when reading
    SPI::WriteBit writeBit;

    ///< How long to wait before starting a tranmission after CS is set (us)
    unsigned int csSetupTimeUs;

    ///< How long to hold cs after the end of a tranmission (us)
    unsigned int csHoldTimeUs;

    SPIBusConfig(SPI::ClockDivider clockDivider = SPI::ClockDivider::DIV_256,
                 SPI::Mode mode                 = SPI::Mode::MODE_0,
                 SPI::Order bitOrder            = SPI::Order::MSB_FIRST,
                 SPI::Order byteOrder           = SPI::Order::MSB_FIRST,
                 SPI::WriteBit writeBit         = SPI::WriteBit::NORMAL,
                 unsigned int csSetupTimeUs = 0, unsigned int csHoldTimeUs = 0)
        : clockDivider(clockDivider), mode(mode), bitOrder(bitOrder),
          byteOrder(byteOrder), writeBit(writeBit),
          csSetupTimeUs(csSetupTimeUs), csHoldTimeUs(csHoldTimeUs)
    {
    }

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

    ///< Delete copy/move constructors/operators.
    SPIBusInterface(const SPIBusInterface&)            = delete;
    SPIBusInterface& operator=(const SPIBusInterface&) = delete;
    SPIBusInterface(SPIBusInterface&&)                 = delete;
    SPIBusInterface& operator=(SPIBusInterface&&)      = delete;

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
    virtual void select(GpioType cs) = 0;

    /**
     * @brief Deselects the slave.
     *
     * @param cs Chip select pin for the slave.
     */
    virtual void deselect(GpioType cs) = 0;

    // Read, write and transfer operations

    /**
     * @brief Reads 8 bits from the bus.
     *
     * @return Byte read from the bus.
     */
    virtual uint8_t read() = 0;

    /**
     * @brief Reads 16 bits from the bus.
     *
     * @return Half word read from the bus.
     */
    virtual uint16_t read16() = 0;

    /**
     * @brief Reads 24 bits from the bus.
     *
     * @return Bytes read from the bus (MSB of the uint32_t value will be 0).
     */
    virtual uint32_t read24() = 0;

    /**
     * @brief Reads 32 bits from the bus.
     *
     * @return Word read from the bus.
     */
    virtual uint32_t read32() = 0;

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
    virtual void read16(uint16_t* data, size_t size) = 0;

    /**
     * @brief Writes 8 bits to the bus.
     *
     * @param data Byte to write.
     */
    virtual void write(uint8_t data) = 0;

    /**
     * @brief Writes 16 bits to the bus.
     *
     * @param data Half word to write.
     */
    virtual void write16(uint16_t data) = 0;

    /**
     * @brief Writes 24 bits to the bus.
     *
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     */
    virtual void write24(uint32_t data) = 0;

    /**
     * @brief Writes 32 bits to the bus.
     *
     * @param data Word to write.
     */
    virtual void write32(uint32_t data) = 0;

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    virtual void write(const uint8_t* data, size_t size) = 0;

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    virtual void write16(const uint16_t* data, size_t size) = 0;

    /**
     * @brief Full duplex transmission of 8 bits on the bus.
     *
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    virtual uint8_t transfer(uint8_t data) = 0;

    /**
     * @brief Full duplex transmission of 16 bits on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    virtual uint16_t transfer16(uint16_t data) = 0;

    /**
     * @brief Full duplex transmission of 24 bits on the bus.
     *
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     * @return Bytes read from the bus (the MSB of the uint32_t will be 0).
     */
    virtual uint32_t transfer24(uint32_t data) = 0;

    /**
     * @brief Full duplex transmission of 32 bits on the bus.
     *
     * @param data Word to write.
     * @return Half word read from the bus.
     */
    virtual uint32_t transfer32(uint32_t data) = 0;

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to transfer.
     * @param size Size of the buffer.
     */
    virtual void transfer(uint8_t* data, size_t size) = 0;

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to transfer.
     * @param size Size of the buffer.
     */
    virtual void transfer16(uint16_t* data, size_t size) = 0;
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

    SPISlave(SPIBusInterface& bus, GpioType cs, SPIBusConfig config = {})
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
