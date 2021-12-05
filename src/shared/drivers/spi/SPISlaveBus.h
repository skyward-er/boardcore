/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "SPIBusInterface.h"
#include "SPISignalGenerator.h"

namespace Boardcore
{

/**
 * @brief This implementation of SPIBusInterface uses the spi peripheral in
 * slave mode and the spi signal generator to act as an spi master.
 */
class SPISlaveBus : public SPIBusInterface
{
public:
    SPISlaveBus(SPIType* spi, SPISignalGenerator signalGenerator);

    ///< Delete copy/move contructors/operators.
    SPISlaveBus(const SPISlaveBus&) = delete;
    SPISlaveBus& operator=(const SPISlaveBus&) = delete;
    SPISlaveBus(SPISlaveBus&&)                 = delete;
    SPISlaveBus& operator=(SPISlaveBus&&) = delete;

    /**
     * @brief Configures and enables the bus with the provided configuration.
     *
     * Since this implementation is not syncronized, if configure() is called on
     * an already in use bus nothing will be done.
     *
     * Use SyncedSPIBus if you need to synchronize access to the bus.
     */
    void configure(SPIBusConfig config) override;

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
     * @param size Size of the buffer in bytes.
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief Reads multiple half words from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer in bytes.
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
     * @param size Size of the buffer in bytes.
     */
    void write(uint8_t* data, size_t size) override;

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
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
     * @param size Size of the buffer in bytes.
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer in bytes.
     */
    void transfer(uint16_t* data, size_t size) override;

private:
    SPI spi;
    SPISignalGenerator signalGenerator;
    SPIBusConfig config{};
};

inline SPISlaveBus::SPISlaveBus(SPIType* spi,
                                SPISignalGenerator signalGenerator)
    : spi(spi), signalGenerator(signalGenerator)
{
}

inline void SPISlaveBus::configure(SPIBusConfig newConfig)
{
    // Save the new configuration
    config = newConfig;

    // Wait until the peripheral is done before changing configuration
    spi.waitPeripheral();

    // Disable the peripheral
    spi.disable();

    // Configure clock polarity and phase
    spi.setMode(config.mode);

    // Configure bit order
    spi.setBitOrder(config.bitOrder);

    // Enable the peripheral
    spi.enable();
}

inline void SPISlaveBus::select(GpioType& cs) {}

inline void SPISlaveBus::deselect(GpioType& cs) {}

// Read, write and transfer operations

inline uint8_t SPISlaveBus::read()
{
    signalGenerator.generateSingleTransaction(1);
    return spi.read();
}

inline uint16_t SPISlaveBus::read16()
{
    signalGenerator.generateSingleTransaction(2);
    return spi.read16();
}

inline void SPISlaveBus::read(uint8_t* data, size_t size)
{
    signalGenerator.generateSingleTransaction(size);
    spi.read(data, size);
}

inline void SPISlaveBus::read(uint16_t* data, size_t size)
{
    signalGenerator.generateSingleTransaction(size);
    spi.read(data, size);
}

inline void SPISlaveBus::write(uint8_t data)
{
    signalGenerator.generateSingleTransaction(1);
    spi.write(data);
}

inline void SPISlaveBus::write(uint16_t data)
{
    signalGenerator.generateSingleTransaction(2);
    spi.write(data);
}

inline void SPISlaveBus::write(uint8_t* data, size_t size)
{
    signalGenerator.generateSingleTransaction(size);
    spi.write(data, size);
}

inline void SPISlaveBus::write(uint16_t* data, size_t size)
{
    signalGenerator.generateSingleTransaction(size);
    spi.write(data, size);
}

inline uint8_t SPISlaveBus::transfer(uint8_t data)
{
    signalGenerator.generateSingleTransaction(1);
    return spi.transfer(data);
}

inline uint16_t SPISlaveBus::transfer(uint16_t data)
{
    signalGenerator.generateSingleTransaction(2);
    return spi.transfer(data);
}

inline void SPISlaveBus::transfer(uint8_t* data, size_t size)
{
    signalGenerator.generateSingleTransaction(size);
    spi.transfer(data, size);
}

inline void SPISlaveBus::transfer(uint16_t* data, size_t size)
{
    signalGenerator.generateSingleTransaction(size);
    spi.transfer(data, size);
}

}  // namespace Boardcore
