/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio, Niccolò Betto
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
#include <interfaces/delays.h>
#include <units/Frequency.h>
#include <utils/ClockUtils.h>

#include <cstddef>

#include "SPIDefs.h"

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
     * This parameter is used when reading registers 16 bit wide or more.
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
               bitOrder == other.bitOrder;
    }

    bool operator!=(const SPIBusConfig& other) const
    {
        return !(*this == other);
    }

    /**
     * @brief Computes the clock divider to use for the given target frequency.
     * The computed divider will yield the highest possible SPI clock frequency
     * which is not higher than the target frequency.
     * The spi peripheral is needed to get the source clock frequency, which
     * depends on which APB bus the spi is connected to.
     *
     * @note The target frequency will most likely not be achieved exactly,
     * because the clock divider can only be set to discrete power-of-2 values.
     *
     * @param spi SPI peripheral for which to compute the divider.
     * @param frequency Desired frequency for the SPI bus in hertz.
     * @return The optimal clock divider to use for the given frequency.
     */
    static SPI::ClockDivider computeDivider(SPI_TypeDef* spi,
                                            uint32_t targetFrequency)
    {
        auto apb = ClockUtils::getPeripheralBus(spi);

        uint32_t sourceFreq   = ClockUtils::getAPBPeripheralsClock(apb);
        uint32_t idealDivider = sourceFreq / targetFrequency;

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
    /**
     * @brief Retrieve the pointer to the peripheral currently used.
     */
    virtual SPI_TypeDef* getSpi() = 0;

    /**
     * @brief Configures the bus with the provided configuration parameters.
     * Call this before every transaction to configure the bus.
     *
     * @param config Configuration parameters.
     */
    virtual void configure(const SPIBusConfig& config) = 0;

    /**
     * @brief Transfer data on the bus, sending data from the provided transmit
     * buffer and writing received data to the provided receive buffer.
     *
     * The buffers may be the same, in which case the data will be
     * overwritten with received bytes.
     * The buffers may also both be nullptr, in which case a dummy
     * transfer will be performed.
     *
     * @param txData Buffer containing data to transfer, may be nullptr for
     * read-only transactions.
     * @param rxData Buffer to store received data, may be nullptr for
     * write-only transactions.
     * @param size Size of the buffers.
     */
    virtual void transfer(const uint8_t* txData, uint8_t* rxData,
                          size_t size) = 0;
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

    /**
     * @brief Selects the slave for SPI communication.
     */
    void select()
    {
        cs.low();

        if (config.csSetupTimeUs > 0)
            miosix::delayUs(config.csSetupTimeUs);
    }

    /**
     * @brief Deselects the slave for SPI communication.
     */
    void deselect()
    {
        if (config.csHoldTimeUs > 0)
            miosix::delayUs(config.csHoldTimeUs);

        cs.high();
    }

    bool isSelected() { return cs.value() == 0; }
};

/**
 * @brief RAII Interface for SPI chip selection.
 */
class SPISelectGuard
{
public:
    explicit SPISelectGuard(SPISlave& slave) : slave(slave), owns(true)
    {
        slave.select();
    }

    ~SPISelectGuard()
    {
        if (owns)
            slave.deselect();
    }

    void deselect()
    {
        slave.deselect();
        owns = false;
    }

    bool owns_selection() const { return owns; }

private:
    SPISlave& slave;
    bool owns;
};

}  // namespace Boardcore
