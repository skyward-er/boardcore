/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

namespace Boardcore
{

/**
 * @brief Generic error codes that the spi transaction
 * with dma can generate.
 */
enum class SPITransactionDMAErrors : uint8_t
{
    // Dma errors

    NO_ERRORS             = static_cast<uint8_t>(DMAErrors::NO_ERRORS),
    DMA_TIMEOUT           = static_cast<uint8_t>(DMAErrors::TIMEOUT),
    DMA_FIFO_ERROR        = static_cast<uint8_t>(DMAErrors::FIFO_ERROR),
    DMA_TRANSFER_ERROR    = static_cast<uint8_t>(DMAErrors::TRANSFER_ERROR),
    DMA_DIRECT_MODE_ERROR = static_cast<uint8_t>(DMAErrors::DIRECT_MODE_ERROR),

    // New spi transaction dma errors

    SPI_TIMEOUT = static_cast<uint8_t>(DMAErrors::END_OF_BASE_ERRORS),
};

/**
 * @brief Provides high-level access to the SPI Bus for a single transaction.
 *
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
 *                                          // provided parameters.
 *
 *     spi.write(REG_EX, 0x56); // writes data to REG_EX
 *     uint8_t reg2 = spi.read(REG_EX_2); // reads from REG_EX_2
 *
 *     // ...As many read/writes as you wish...
 *
 *     // transaction end. SPITransaction object is destructed and the bus is
 *     // freed for use by someone else
 * }
 */
class SPITransaction
{
private:
    /**
     * @brief The timeout for the method `spiWaitForTransmissionComplete()`,
     * in nanoseconds.
     */
    static constexpr std::chrono::nanoseconds defaultTimeout =
        std::chrono::milliseconds(250);
    // static constexpr int64_t spiDefaultTimeoutNs =
    //     std::chrono::duration_cast<std::chrono::nanoseconds>(
    //         std::chrono::milliseconds(250))
    //         .count();
public:
    /**
     * @brief Instantiates a new SPITransaction, configuring the bus with the
     * provided parameters.
     *
     * @param slave Slave to communicate with.
     */
    explicit SPITransaction(const SPISlave& slave);

    // /**
    //  * @brief Instantiates a new SPITransaction, configuring the bus with the
    //  * provided parameters.
    //  *
    //  * @param bus Bus to communicate on.
    //  * @param cs Chip select of the slave to communicate to.
    //  * @param config Configuration of the bus for the selected slave.
    //  */
    // SPITransaction(SPIBusInterface &bus, GpioType cs, SPIBusConfig config);

    ///< Delete copy/move constructors/operators.
    SPITransaction(const SPITransaction&)            = delete;
    SPITransaction& operator=(const SPITransaction&) = delete;
    SPITransaction(SPITransaction&&)                 = delete;
    SPITransaction& operator=(SPITransaction&&)      = delete;

    /**
     * @brief Returns the underlying bus for low level access.
     *
     * @return SPIBusInterface associated with this transaction.
     */
    SPIBusInterface& getBus();

    /**
     * @brief Disable dma for the upcoming operations.
     */
    void disableDma();

    /**
     * @brief Enable dma for the upcoming operations, if possible.
     * @return True if the SPISlave is correctly configured with dma streams,
     * false otherwise.
     */
    bool enableDma();

    // Read, write and transfer operations

    /**
     * @brief Reads a single byte from the bus.
     *
     * @return Byte read from the bus.
     */
    uint8_t read();

    /**
     * @brief Reads a single half word from the bus.
     *
     * @return Half word read from the bus.
     */
    uint16_t read16();

    /**
     * @brief Reads 24 bits from the bus.
     *
     * @return Bytes read from the bus (MSB of the uint32_t value will be 0).
     */
    virtual uint32_t read24();

    /**
     * @brief Reads 32 bits from the bus.
     *
     * @return Word read from the bus.
     */
    virtual uint32_t read32();

    /**
     * @brief Reads multiple bytes from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer in bytes.
     */
    void read(uint8_t* data, size_t size);

    /**
     * @brief Reads multiple half words from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer in bytes.
     */
    void read16(uint16_t* data, size_t size);

    /**
     * @brief Writes a single byte to the bus.
     *
     * @param data Byte to write.
     */
    void write(uint8_t data);

    /**
     * @brief Writes a single half word to the bus.
     *
     * @param data Half word to write.
     */
    void write16(uint16_t data);

    /**
     * @brief Writes 24 bits to the bus.
     *
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     */
    virtual void write24(uint32_t data);

    /**
     * @brief Writes 32 bits to the bus.
     *
     * @param data Word to write.
     */
    virtual void write32(uint32_t data);

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
     */
    void write(uint8_t* data, size_t size);

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
     */
    void write16(uint16_t* data, size_t size);

    /**
     * @brief Full duplex transmission of one byte on the bus.
     *
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    uint8_t transfer(uint8_t data);

    /**
     * @brief Full duplex transmission of one half word on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    uint16_t transfer16(uint16_t data,
                        std::chrono::nanoseconds timeout = defaultTimeout);

    /**
     * @brief Full duplex transmission of 24 bits on the bus.
     *
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     * @return Bytes read from the bus (the MSB of the uint32_t will be 0).
     */
    virtual uint32_t transfer24(uint32_t data);

    /**
     * @brief Full duplex transmission of 32 bits on the bus.
     *
     * @param data Word to write.
     * @return Half word read from the bus.
     */
    virtual uint32_t transfer32(uint32_t data);

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to transfer.
     * @param size Size of the buffer in bytes.
     */
    void transfer(uint8_t* data, size_t size);

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to transfer.
     * @param size Size of the buffer in bytes.
     */
    void transfer16(uint16_t* data, size_t size);

    // Read, write and transfer operations with registers

    /**
     * @brief Reads an 8 bit register.
     *
     * @return Byte read from the register.
     */
    uint8_t readRegister(uint8_t reg,
                         std::chrono::nanoseconds timeout = defaultTimeout);

    /**
     * @brief Reads a 16 bit register.
     *
     * @return Byte read from the register.
     */
    uint16_t readRegister16(uint8_t reg);

    /**
     * @brief Reads a 24 bit register.
     *
     * @return Byte read from the register.
     */
    uint32_t readRegister24(uint8_t reg);

    /**
     * @brief Reads a 32 bit register.
     *
     * @return Byte read from the register.
     */
    uint32_t readRegister32(uint8_t reg);

    /**
     * @brief Reads multiple bytes starting from the specified register.
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer in bytes.
     */
    void readRegisters(uint8_t reg, uint8_t* data, size_t size);

    /**
     * @brief Writes an 8 bit register.
     *
     * @param reg Register address.
     * @param data Byte to write.
     */
    void writeRegister(uint8_t reg, uint8_t data);

    /**
     * @brief Writes a 16 bit register.
     *
     * @param reg Register address.
     * @param data Byte to write.
     */
    void writeRegister16(uint8_t reg, uint16_t data);

    /**
     * @brief Writes a 24 bit register.
     *
     * @param reg Register address.
     * @param data Byte to write.
     */
    void writeRegister24(uint8_t reg, uint32_t data);

    /**
     * @brief Writes a 32 bit register.
     *
     * @param reg Register address.
     * @param data Byte to write.
     */
    void writeRegister32(uint8_t reg, uint32_t data);

    /**
     * @brief Writes multiple bytes starting from the specified register.
     *
     * @param reg Register start address.
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
     */
    void writeRegisters(uint8_t reg, uint8_t* data, size_t size);

private:
    const SPISlave& slave;
    bool useDma;  ///< True if the DMA must be used during the transaction
    // SPIType* const spiPtr;
    SPI_TypeDef* const spiPtr;

    // Last error for the transmitting stream
    SPITransactionDMAErrors lastErrorTx = SPITransactionDMAErrors::NO_ERRORS;
    // Last error for the receiving stream
    SPITransactionDMAErrors lastErrorRx = SPITransactionDMAErrors::NO_ERRORS;

    /**
     * @brief Perform the dma transaction.
     * @warning The streams must be setup and ready to go.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return True if the operation was successful. False otherwise, sets
     * the last error.
     */
    bool dmaTransfer(const std::chrono::nanoseconds timeout);

    /**
     * @brief Wait until the spi peripheral has finished transmitting.
     * @return True if operation successful, false if the timeout
     * expired.
     */
    bool spiDmaWaitForTransmissionComplete();

    /**
     * @brief Setup the configuration struct with the default sender values
     * needed for an spi transaction.
     * @param txSetup The struct to be configured.
     * @param srcAddr Source address.
     * @param nBytes Number of bytes to be transmitted.
     */
    void defaultDmaTransmittingSetup(DMATransaction& txSetup, void* srcAddr,
                                     uint16_t nBytes);

    /**
     * @brief Setup the configuration struct with the default receiver values
     * needed for an spi transaction.
     * @param rxSetup The struct to be configured.
     * @param dstAddr Destination address.
     * @param nBytes Number of bytes to be received.
     */
    void defaultDmaReceivingSetup(DMATransaction& rxSetup, void* dstAddr,
                                  uint16_t nBytes);

    /**
     * @brief Setup the configuration struct with the default values needed
     * for an spi transaction.
     * @param streamSetup The struct to be configured.
     * @param dir Direction of the transaction.
     * @param srcAddr Source address.
     * @param dstAddr Destination address.
     * @param nBytes Number of bytes to be transmitted/received.
     * @param srcIncr Flag, true if the source address must be incremented.
     * @param dstIncr Flag, true if the destination address must be incremented.
     */
    void defaultDmaSetup(DMATransaction& streamSetup,
                         DMATransaction::Direction dir, void* srcAddr,
                         void* dstAddr, uint16_t nBytes, bool srcIncr,
                         bool dstIncr);
};

}  // namespace Boardcore
