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
 * See datasheet for further information
 */
struct SPIBusConfig
{
    SPIBaudRate br = SPIBaudRate::DIV_2;  ///> Peripheral clock division
    uint8_t cpol   = 0;                   ///> Clock polarity
    uint8_t cpha   = 0;                   ///> Clock phase
    bool lsb_first = false;               ///> MSB or LSB first

    unsigned int cs_setup_time_us = 0;  ///> How long to wait before starting a
                                        ///> a trasmission after CS is set (us)
    unsigned int cs_hold_time_us = 0;   ///> How long to hold cs after the end
                                        ///> of a trasmission (us)
};

/**
 * @brief Interface for low level access of a SPI bus
 */
class SPIBusInterface
{
public:
    /**
     * @brief Writes \p data to the bus.
     *
     * @param    data Buffer containing data to write
     * @param    size Number of bytes to write
     */
    virtual void write(uint8_t* data, size_t size) = 0;

    /**
     * @brief Reads \p size bytes from the SPI bus, putting them in \p data.
     *
     * @param    data Buffer to be filled with received data
     * @param    size Number of bytes to receive
     */
    virtual void read(uint8_t* data, size_t size) = 0;

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
 * @brief Low level driver for communicating on a SPI Bus, provides
 *        SPIBusInterface.
 */
class SPIBus : public SPIBusInterface
{
public:
    /**
     * @brief Instantiates a new SPIBus
     *
     * @param    spi       Pointer to the SPI peripheral to be used
     */
    SPIBus(SPI_TypeDef* spi);

    /**
     * @brief See SPIBusInterface::write()
     */
    void write(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::read()
     */
    void read(uint8_t* data, size_t sizes) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::select()
     */
    void select(GpioPin& cs) override;

    /**
     * @brief See SPIBusInterface::deselect()
     */
    void deselect(GpioPin& cs) override;

    /**
     * @brief See SPIBusInterface::configure()
     */
    void configure(SPIBusConfig config) override;

private:
    /**
     * Writes a single byte on the SPI bus.
     *
     * @param    byte Pointer to the byte to be written
     */
    void write(uint8_t* byte);

    /**
     * Reads a single byte from the SPI bus.
     *
     * @param    byte Pointer to the byte where the read data will be stored
     */
    void read(uint8_t* byte);

    /**
     * Full duplex transfer. Writes a single byte on the SPI bus and replaces
     * its content with the received data
     *
     * @param    byte Pointer to the byte to be transfered
     */
    void transfer(uint8_t* byte);

    SPI_TypeDef* spi;

    SPIBusConfig last_config;
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
 *                                          // provided parameters
 *
 *     spi.write(REG_EX, 0x56); // writes data to REG_EX
 *     uint8_t reg2 = spi.read(REG_EX_2); // reads from REG_EX_2
 *
 *     // ...
 *
 *     // transaction end:
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
     */
    uint8_t read(uint8_t reg);

    /**
     * @brief Reads \p size bytes from the \p reg register
     *
     * @param    reg    Slave device register
     * @param    data   Buffer where read bytes will be stored
     * @param    size   Number of bytes to read
     */
    void read(uint8_t reg, uint8_t* data, size_t size);

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
    // No public copy constructor or operator =
    SPITransaction(const SPITransaction& l);
    SPITransaction& operator=(const SPITransaction& l);
};

// Defined here and not in the .cpp to make them inline
inline void SPIBus::write(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        write(data + i);
    }
}

inline void SPIBus::read(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        read(data + i);
    }
}

inline void SPIBus::transfer(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        transfer(data + i);
    }
}

inline void SPIBus::select(GpioPin& cs)
{
    cs.low();
    if (last_config.cs_setup_time_us > 0)
    {
        delayUs(last_config.cs_setup_time_us);
    }
}

inline void SPIBus::deselect(GpioPin& cs)
{
    if (last_config.cs_hold_time_us > 0)
    {
        delayUs(last_config.cs_hold_time_us);
    }
    cs.high();
}

inline void SPIBus::write(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    // Write the byte in the transmit buffer
    spi->DR = *byte;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Clear the RX buffer by accessing the DR register
    spi->DR;
}

inline void SPIBus::transfer(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    // Write the byte in the transmit buffer
    spi->DR = *byte;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}

inline void SPIBus::read(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    // Write the byte in the transmit buffer
    spi->DR = 0;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}
