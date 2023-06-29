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
#include <utils/ClockUtils.h>

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
    SPI::ClockDivider clockDivider = SPI::ClockDivider::DIV_256;

    ///< Clock polarity and phase configuration
    SPI::Mode mode = SPI::Mode::MODE_0;

    ///< MSBit or LSBit first
    SPI::Order bitOrder = SPI::Order::MSB_FIRST;

    /**
     * @brief MSByte or LSByte first
     *
     * This parameter is used when reading and writing registers 16 bit wide or
     * more.
     *
     * A device features MSByte first ordering if the most significant byte is
     * at the lowest address. Example of a 24bit register:
     *   Address:  0x06  0x07  0x08
     *     value:  MSB   MID   LSB
     *
     * Conversely, an LSByte first ordering starts with the lowest significant
     * byte first.
     *
     * Also, in every device used since now, in multiple registers accesses, the
     * device always increments the address. So the user has always to provide
     * the lowest address.
     *
     * @warning This driver does not support devices which decrements registers
     * address during multiple registers accesses.
     */
    SPI::Order byteOrder = SPI::Order::MSB_FIRST;

    ///< Write bit behaviour, default high when reading
    SPI::WriteBit writeBit = SPI::WriteBit::NORMAL;

    ///< How long to wait before starting a tranmission after CS is set (us)
    unsigned int csSetupTimeUs = 0;

    ///< How long to hold cs after the end of a tranmission (us)
    unsigned int csHoldTimeUs = 0;

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

    /**
     * @brief Computes the clock divider for the provided maximum frequency.
     *
     * The computed divider has the lowest possible value which generates a
     * frequency not higher than the provided one.
     * The spi peripheral is needed to get the source clock frequency, which
     * depends on which APB bus the spi is connected to.
     */
    static SPI::ClockDivider computeDivider(SPI_TypeDef* spi, uint32_t maxFreq)
    {
        ClockUtils::APB apb;
        ClockUtils::getPeripheralBus(spi, apb);

        uint32_t sourceFreq   = ClockUtils::getAPBPeripheralsClock(apb);
        uint32_t idealDivider = sourceFreq / maxFreq;

        if (idealDivider <= 2)
            return SPI::ClockDivider::DIV_2;
        else if (idealDivider <= 4)
            return SPI::ClockDivider::DIV_4;
        else if (idealDivider <= 8)
            return SPI::ClockDivider::DIV_8;
        else if (idealDivider <= 16)
            return SPI::ClockDivider::DIV_16;
        else if (idealDivider <= 32)
            return SPI::ClockDivider::DIV_32;
        else if (idealDivider <= 64)
            return SPI::ClockDivider::DIV_64;
        else if (idealDivider <= 128)
            return SPI::ClockDivider::DIV_128;
        else
            return SPI::ClockDivider::DIV_256;
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
