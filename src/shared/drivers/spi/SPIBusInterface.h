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

#include <functional>
#include <type_traits>

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

    ///< MSBit or LSBit first
    SPI::Order bitOrder;

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
    SPI::Order byteOrder;

    ///< Write bit behaviour, default high when reading
    SPI::WriteBit writeBit;

    SPI::WriteBitPosition writeBitPosition;

    ///< How long to wait before starting a tranmission after CS is set (us)
    unsigned int csSetupTimeUs;

    ///< How long to hold cs after the end of a tranmission (us)
    unsigned int csHoldTimeUs;

    SPIBusConfig(
        SPI::ClockDivider clockDivider         = SPI::ClockDivider::DIV_256,
        SPI::Mode mode                         = SPI::Mode::MODE_0,
        SPI::Order bitOrder                    = SPI::Order::MSB_FIRST,
        SPI::Order byteOrder                   = SPI::Order::MSB_FIRST,
        SPI::WriteBit writeBit                 = SPI::WriteBit::NORMAL,
        SPI::WriteBitPosition writeBitPosition = SPI::WriteBitPosition::MSBit,
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
     * @brief Reads DataSize bits from the bus.
     *
     * @return DataSize read from the bus.
     */
    template <typename DataSize>
    DataSize read()
    {
        static_assert(std::is_same<DataSize, uint8_t>::value ||
                          std::is_same<DataSize, uint16_t>::value ||
                          std::is_same<DataSize, uint32_t>::value,
                      "DataSize can be one of uint8_t, uint16_t, uint32_t");
    };

    /**
     * @brief Reads multiple element of size DataSize from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    template <typename DataSize>
    void read(DataSize* data, size_t size)
    {
        static_assert(std::is_same<DataSize, uint8_t>::value ||
                          std::is_same<DataSize, uint16_t>::value ||
                          std::is_same<DataSize, uint32_t>::value,
                      "DataSize can be one of uint8_t, uint16_t, uint32_t");
    };

    /**
     * @brief Writes DataSize bits to the bus.
     *
     * @param data data to write.
     */
    template <typename DataSize>
    void write(DataSize data)
    {
        static_assert(std::is_same<DataSize, uint8_t>::value ||
                          std::is_same<DataSize, uint16_t>::value ||
                          std::is_same<DataSize, uint32_t>::value,
                      "DataSize can be one of uint8_t, uint16_t, uint32_t");
    };

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    template <typename DataSize>
    void write(const DataSize* data, size_t size)
    {
        static_assert(std::is_same<DataSize, uint8_t>::value ||
                          std::is_same<DataSize, uint16_t>::value ||
                          std::is_same<DataSize, uint32_t>::value,
                      "DataSize can be one of uint8_t, uint16_t, uint32_t");
    };

    /**
     * @brief Full duplex transmission of DataSize bits on the bus.
     *
     * @param data data to write.
     * @return data read from the bus.
     */
    template <typename DataSize>
    DataSize transfer(DataSize data)
    {
        static_assert(std::is_same<DataSize, uint8_t>::value ||
                          std::is_same<DataSize, uint16_t>::value ||
                          std::is_same<DataSize, uint32_t>::value,
                      "DataSize can be one of uint8_t, uint16_t, uint32_t");
    };

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to transfer.
     * @param size Size of the buffer.
     */
    template <typename DataSize>
    void transfer(DataSize* data, size_t size)
    {
        static_assert(std::is_same<DataSize, uint8_t>::value ||
                          std::is_same<DataSize, uint16_t>::value ||
                          std::is_same<DataSize, uint32_t>::value,
                      "DataSize can be one of uint8_t, uint16_t, uint32_t");
    };

    /**
     * @brief Retrieve the pointer to the peripheral currently used.
     */
    virtual SPI_TypeDef* getSpi() = 0;
};

/**
 * @brief Contains information about a single SPI slave device.
 */
template <typename AddressSize>
struct SPISlave
{
    static_assert(std::is_same<AddressSize, uint8_t>::value ||
                      std::is_same<AddressSize, uint16_t>::value ||
                      std::is_same<AddressSize, uint32_t>::value ||
                      std::is_same<AddressSize, void>::value,
                  "AddressSize can be one of uint8_t, uint16_t, uint32_t. void "
                  "is used when no registers are involved");

    SPIBusInterface& bus;  ///< Bus on which the slave is connected.
    SPIBusConfig config;   ///< How the bus should be configured to communicate
                           ///< with the slave.
    GpioType cs;           ///< Chip select pin

    AddressSize readWriteMask =
        0b1 << ((8 * sizeof(AddressSize)) -
                static_cast<AddressSize>(config.writeBitPosition));

    SPISlave(SPIBusInterface& bus, GpioType cs, SPIBusConfig config = {})
        : bus(bus), config(config), cs(cs)
    {
    }
};

/**
 * @brief RAII Interface for SPI bus acquisition
 *
 */
template <typename AddressSize>
class SPIAcquireLock
{
public:
    explicit SPIAcquireLock(SPISlave<AddressSize> slave)
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
template <typename AddressSize>
class SPISelectLock
{
public:
    explicit SPISelectLock(SPISlave<AddressSize> slave)
        : SPISelectLock(slave.bus, slave.cs)
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
