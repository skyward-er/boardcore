/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <utils/Numeric.h>

#include <type_traits>

#include "SPIBusInterface.h"

namespace Boardcore
{

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
 *     SPITransaction spi(spiSlave); // Configures the bus with the
 *                                   // slave parameters.
 *
 *     spi.writeRegister(REG_EX, 0x56); // writes data to REG_EX
 *     uint8_t reg2;
 *     spi.readRegister(REG_EX_2, reg2); // reads from REG_EX_2 into reg2
 *
 *     // ...As many read/writes as you wish...
 *
 *     // transaction end. SPITransaction object is destructed and the bus is
 *     // freed for use by someone else
 * }
 */
class SPITransaction
{
public:
    /**
     * @brief Instantiates a new SPITransaction, using the provided select guard
     * to manage the slave selection.
     * @note Slave selection will be managed by this class only if the provided
     * guard does not already own the selection.
     * @param slave Slave to communicate with.
     * @param selectGuard Guard object which will manage the slave selection for
     * this transaction.
     */
    SPITransaction(SPISlave& slave, const SPISelectGuard& guard)
        : slave(slave), externallySelected(guard.owns_selection())
    {
        slave.bus.configure(slave.config);
    }

    /**
     * @brief Instantiates a new SPITransaction, configuring the bus with the
     * provided parameters.
     * @param slave Slave to communicate with.
     */
    explicit SPITransaction(SPISlave& slave)
        : slave(slave), externallySelected(false)
    {
        slave.bus.configure(slave.config);
    }

    ///< Delete copy/move constructors/operators.
    SPITransaction(const SPITransaction&)            = delete;
    SPITransaction& operator=(const SPITransaction&) = delete;
    SPITransaction(SPITransaction&&)                 = delete;
    SPITransaction& operator=(SPITransaction&&)      = delete;

    /**
     * @return The SPIBusInterface associated with this transaction.
     */
    SPIBusInterface& getBus() { return slave.bus; }

    /**
     * Re-export of base bus functions with automatic slave selection
     * management.
     */

    void transfer(uint8_t* data, size_t size)
    {
        transferImpl(data, data, size);
    }

    void read(uint8_t* data, size_t size) { transferImpl(nullptr, data, size); }

    void write(const uint8_t* data, size_t size)
    {
        transferImpl(data, nullptr, size);
    }

    /**
     * Optimized routines for small transfers, implemented with a single
     * transfer and automatically handling endianess.
     */

    /**
     * @brief Full duplex transmission of one byte on the bus.
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    uint8_t transfer(uint8_t data);

    /**
     * @brief Full duplex transmission of one half word on the bus.
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    uint16_t transfer16(uint16_t data);

    /**
     * @brief Full duplex transmission of 24 bits on the bus.
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     * @return Bytes read from the bus (the MSB of the uint32_t will be 0).
     */
    uint32_t transfer24(uint32_t data);

    /**
     * @brief Full duplex transmission of 32 bits on the bus.
     *
     * @param data Word to write.
     * @return Half word read from the bus.
     */
    uint32_t transfer32(uint32_t data);

    /**
     * Optimized routines for small reads, implemented with a single
     * transfer and automatically handling endianess.
     */

    /**
     * @brief Reads a single byte from the bus.
     * @return Byte read from the bus.
     */
    uint8_t read();

    /**
     * @brief Reads a single half word from the bus.
     * @return Half word read from the bus.
     */
    uint16_t read16();

    /**
     * @brief Reads 24 bits from the bus.
     * @return Bytes read from the bus (MSB of the uint32_t value will be 0).
     */
    uint32_t read24();

    /**
     * @brief Reads 32 bits from the bus.
     * @return Word read from the bus.
     */
    uint32_t read32();

    /**
     * Optimized routines for small writes, implemented with a single
     * transfer and automatically handling endianess.
     */

    /**
     * @brief Writes a single byte to the bus.
     * @param data Byte to write.
     */
    void write(uint8_t data);

    /**
     * @brief Writes a single half word to the bus.
     * @param data Half word to write.
     */
    void write16(uint16_t data);

    /**
     * @brief Writes 24 bits to the bus.
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     */
    void write24(uint32_t data);

    /**
     * @brief Writes 32 bits to the bus.
     * @param data Word to write.
     */
    void write32(uint32_t data);

    /**
     * Optimized routines for small register reads, implemented with a single
     * transfer and automatically handling endianess.
     */

    /**
     * @brief Reads an 8 bit register.
     * @return Byte read from the register.
     */
    uint8_t readRegister(uint8_t reg);

    /**
     * @brief Reads a 16 bit register.
     * @return Half word read from the register.
     */
    uint16_t readRegister16(uint8_t reg);

    /**
     * @brief Reads a 24 bit register.
     * @return Bytes read from the register (the MSB of the uint32_t will be 0).
     */
    uint32_t readRegister24(uint8_t reg);

    /**
     * @brief Reads a 32 bit register.
     * @return Word read from the register.
     */
    uint32_t readRegister32(uint8_t reg);

    /**
     * Optimized routines for small register writes, implemented with a single
     * transfer and automatically handling endianess.
     */

    /**
     * @brief Writes an 8 bit register.
     * @param reg Register address.
     * @param data Byte to write.
     */
    void writeRegister(uint8_t reg, uint8_t data);

    /**
     * @brief Writes a 16 bit register.
     * @param reg Register address.
     * @param data Half word to write.
     */
    void writeRegister16(uint8_t reg, uint16_t data);

    /**
     * @brief Writes a 24 bit register.
     * @param reg Register address.
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     */
    void writeRegister24(uint8_t reg, uint32_t data);

    /**
     * @brief Writes a 32 bit register.
     * @param reg Register address.
     * @param data Word to write.
     */
    void writeRegister32(uint8_t reg, uint32_t data);

    /**
     * Routines for arbitrary length register reads/writes.
     */

    /**
     * @brief Reads multiple bytes starting from the specified register,
     * for slaves that support auto-incrementing the register address.
     * @note This function does not handle endianess, byte order must be managed
     * by the user!
     * @param data Buffer to store received data.
     * @param size Size of the buffer in bytes.
     */
    void readRegisters(uint8_t reg, uint8_t* data, size_t size);

    /**
     * @brief Writes multiple bytes starting from the specified register,
     * for slaves that support auto-incrementing the register address.
     * @note This function does not handle endianess, byte order must be managed
     * by the user!
     * @param reg Register start address.
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
     */
    void writeRegisters(uint8_t reg, const uint8_t* data, size_t size);

private:
    /**
     * @brief Internal transfer function which handles slave selection and
     * deselection if not already managed by an external guard.
     */
    void transferImpl(const uint8_t* txData, uint8_t* rxData, size_t size)
    {
        TransactionSelectGuard guard(slave, !externallySelected);
        slave.bus.transfer(txData, rxData, size);
    }

    /**
     * @brief RAII helper class to manage slave selection for each transfer,
     * if not already managed by an external guard.
     */
    class TransactionSelectGuard
    {
    public:
        TransactionSelectGuard(SPISlave& slave, bool shouldSelect)
            : slave(slave), shouldSelect(shouldSelect)
        {
            if (shouldSelect)
                slave.select();
        }
        ~TransactionSelectGuard()
        {
            if (shouldSelect)
                slave.deselect();
        }

    private:
        SPISlave& slave;
        bool shouldSelect = true;
    };

    SPISlave& slave;

    /**
     * @brief Whether the slave is already selected by an external guard or not,
     * at the time of this transaction's instantiation.
     * This allows users of this class to manage slave selection themselves if
     * they want/need to.
     *
     * If true, slave is assumed to be selected for the entire lifetime of this
     * transaction object, and no attempt to select/deselect it will be made by
     * this class.
     * If false, slave selection will be managed by this class for each
     * transfer.
     */
    bool externallySelected = false;
};

}  // namespace Boardcore
