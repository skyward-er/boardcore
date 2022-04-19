/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <interfaces/delays.h>

#include "SPIBusInterface.h"

#ifndef USE_MOCK_PERIPHERALS
using SPIType = SPI_TypeDef;
#else
#include <test/FakeSpiTypedef.h>
using SPIType = FakeSpiTypedef;
#endif

namespace Boardcore
{

/**
 * @brief Main implementation of SPIBusInterface used for accessing the SPI
 * peripheral in master mode
 */
class SPIBus : public SPIBusInterface
{
public:
    SPIBus(SPIType* spi);

    ///< Delete copy/move contructors/operators.
    SPIBus(const SPIBus&) = delete;
    SPIBus& operator=(const SPIBus&) = delete;
    SPIBus(SPIBus&&)                 = delete;
    SPIBus& operator=(SPIBus&&) = delete;

    /**
     * @brief Configures and enables the bus with the provided configuration.
     *
     * Since this implementation is not syncronized, if configure() is called on
     * an already in use bus nothing will be done.
     *
     * Use SyncedSPIBus if you need to synchronize access to the bus.
     */
    void configure(SPIBusConfig newConfig) override;

    /**
     * @brief See SPIBusInterface::select().
     */
    void select(GpioType& cs) override;

    /**
     * @brief See SPIBusInterface::deselect().
     */
    void deselect(GpioType& cs) override;

    // Read, write and transfer operations

    /**
     * @brief Reads a single byte from the bus.
     *
     * @return Byte read from the bus.
     */
    uint8_t read() override;

    /**
     * @brief Reads a single half word from the bus.
     *
     * @return Half word read from the bus.
     */
    uint16_t read16() override;

    /**
     * @brief Reads multiple bytes from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief Reads multiple half words from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    void read(uint16_t* data, size_t size) override;

    /**
     * @brief Writes a single byte to the bus.
     *
     * @param data Byte to write.
     */
    void write(uint8_t data) override;

    /**
     * @brief Writes a single half word to the bus.
     *
     * @param data Half word to write.
     */
    void write(uint16_t data) override;

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    void write(uint8_t* data, size_t size) override;

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    void write(uint16_t* data, size_t size) override;

    /**
     * @brief Full duplex transmission of one byte on the bus.
     *
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    uint8_t transfer(uint8_t data) override;

    /**
     * @brief Full duplex transmission of one half word on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    uint16_t transfer(uint16_t data) override;

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer.
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer.
     */
    void transfer(uint16_t* data, size_t size) override;

private:
    SPI spi;
    SPIBusConfig config{};
    bool firstConfigApplied = false;
};

inline SPIBus::SPIBus(SPIType* spi) : spi(spi) {}

inline void SPIBus::configure(SPIBusConfig newConfig)
{
    // Do not reconfigure if already in the correct configuration.
    if (!firstConfigApplied || newConfig != config)
    {
        // Save the new configuration
        config             = newConfig;
        firstConfigApplied = true;

        // Wait until the peripheral is done before changing configuration
        spi.waitPeripheral();

        // Disable the peripheral
        spi.disable();

        // Configure clock polarity and phase
        spi.setMode(config.mode);

        // Configure clock frequency
        spi.setClockDiver(config.clockDivider);

        // Configure bit order
        spi.setBitOrder(config.bitOrder);

        // Configure chip select and master mode
        spi.enableSoftwareSlaveManagement();
        spi.enableInternalSlaveSelection();
        spi.setMasterConfiguration();

        // Enable the peripheral
        spi.enable();
    }
}

inline void SPIBus::select(GpioType& cs)
{
    cs.low();
    if (config.csSetupTimeUs > 0)
    {
        miosix::delayUs(config.csSetupTimeUs);
    }
}

inline void SPIBus::deselect(GpioType& cs)
{
    if (config.csHoldTimeUs > 0)
    {
        miosix::delayUs(config.csHoldTimeUs);
    }
    cs.high();
}

// Read, write and transfer operations

inline uint8_t SPIBus::read() { return spi.read(); }

inline uint16_t SPIBus::read16() { return spi.read16(); }

inline void SPIBus::read(uint8_t* data, size_t size) { spi.read(data, size); }

inline void SPIBus::read(uint16_t* data, size_t size) { spi.read(data, size); }

inline void SPIBus::write(uint8_t data) { spi.write(data); }

inline void SPIBus::write(uint16_t data) { spi.write(data); }

inline void SPIBus::write(uint8_t* data, size_t size) { spi.write(data, size); }

inline void SPIBus::write(uint16_t* data, size_t size)
{
    spi.write(data, size);
}

inline uint8_t SPIBus::transfer(uint8_t data) { return spi.transfer(data); }

inline uint16_t SPIBus::transfer(uint16_t data) { return spi.transfer(data); }

inline void SPIBus::transfer(uint8_t* data, size_t size)
{
    spi.transfer(data, size);
}

inline void SPIBus::transfer(uint16_t* data, size_t size)
{
    spi.transfer(data, size);
}

}  // namespace Boardcore
