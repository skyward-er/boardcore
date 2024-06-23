/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Valerio Flamminii
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
#include <miosix.h>
#include <utils/ClockUtils.h>

#pragma once

/*  driver of MX25R3235FM1IL0 flash memory chip
 *  model of flash memory on compute unit: MX25R3235FM1IL0 4MB, device ID:
 * 0xC22816 FLASH memory space organisation:
 *   - number of byte in the memory: 4.194.304 >> about 4 MB
 *   - "pages"   of 256 Byte each   - number of pages:   16.384
 *   - "sector"  of about 4  KByte  - number of sector:   1.024
 *   - "block32" of about 32 KByte  - number of block32:    128
 *   - "block64" of about 64 KByte  - number of block64:     64
 *
 * NOTES:
 * MX25R3235FM1IL0 flash memory can perform only two types of operations:
 * - "erase" = it set to '1' every bit of selected area.
 * - "program" = it set to '0' only some bits of selected area, in order to
 * write the data value expected. This means that in order to store some data on
 * this chip, you'll need to erase the selected area and then program that area
 * with the desired data.
 * you should also be aware that erasing a memory area could result in a data
 * loss. for example, to store a byte of data, you should: 1 erase selected
 * sector - sectorErase(uint32_t address); 2 program the byte -
 * byteProgram(uint8_t data, uint32_t address, bool verify); if you want to
 * store a vector of data bytes, you should use write(...) function, this
 * method will take care about all erasing and programming sequences.
 * IT'S ALWAYS RECOMMENDED TO CHECK ON RESULTS OF METHODS (true/false) TO BE
 * SURE THAT AN ACTION HAS BEEN PROPERLY PERFORMED!
 */

namespace FlashMemory
{

// device ID of the flash
static constexpr uint32_t DEVICE_ID = 0xC22816;

static constexpr uint32_t PAGES_PER_SECTOR    = 16;
static constexpr uint32_t SECTORS_PER_BLOCK32 = 8;

static constexpr uint32_t BLOCK32_NUM = 128;
static constexpr uint32_t BLOCK64_NUM = 64;
static constexpr uint32_t SECTORS_NUM = 1024;
static constexpr uint32_t PAGES_NUM   = 16384;
static constexpr uint32_t BYTES_NUM   = 4194304;

// sizes of each area in byte
static constexpr uint32_t PAGE_SIZE    = 256;
static constexpr uint32_t SECTOR_SIZE  = PAGE_SIZE * PAGES_PER_SECTOR;
static constexpr uint32_t BLOCK32_SIZE = SECTOR_SIZE * SECTORS_PER_BLOCK32;
static constexpr uint32_t BLOCK64_SIZE = BLOCK32_SIZE * 2;

// memory size in byte (is about 4 MB)
static constexpr uint32_t MEMORY_SIZE = BLOCK64_SIZE * BLOCK64_NUM;

};  // namespace FlashMemory

class QspiFlash
{

public:
    /**
     * @brief QspiFlash class constructor
     */
    QspiFlash(QUADSPI_TypeDef* qspi);

    /**
     * @brief Initialise QUADSPI peripheral in order to communicate with the
     * memory
     * @warning IT MUST BE EXECUTED BEFORE ANY OTHER OPERATION WITH THE MEMORY !
     */
    void init();

    /**
     * @brief read the unique ID of flash
     * @returns device ID
     */
    uint32_t readID();

    /**
     * @brief check if the flash is working properly by checking on its own
     * device ID
     * @return true/false - if the test has been successful or not
     * @warning it's recommended to check if flash is working before any
     * operation !
     */
    bool test();

    /**
     * @brief write an entire vector on flash, starting by a specific
     * sector. if vector size exceed the size of a single sector, the vector
     * will be memorized on consecutive sectors of the memory, starting by
     * sectorNum
     *
     * @param vector array to be saved in memory
     * @param size size of array
     * @param sectorNum number of the starting sector [0 - 1024]
     * @param verify double check on saved data,
     * true  = double check enabled,
     * false = double ckeck disabled,
     * It may slow down the execution still it's a recommended option.
     *
     * @warning each sector needed will be erased and then reprogrammed with
     * vector data bytes, it means that you might lose some data stored into the
     * sectors involved in the process.
     *
     * @return true/false - if the whole operation has been successful.
     */
    bool write(const uint8_t* vector, const size_t size, uint32_t sectorNum,
               bool verify);

    /**
     * @brief copy a whole sector (4KB) from flash into a vector whose size is
     * greater or equal to the size of a sector in memory.
     * @param vector array which will store the sector.
     * @param size size of array
     * @param sectorNum number of the sector which has to be copied into the
     * vector [0 - 1024]
     * @return true/false - if the operation has been successful.
     * @warning vector's size must be enough to contain a whole sector!
     * @warning This function modify the vector passed!
     */
    bool readSector(uint8_t* vector, const size_t size, uint32_t sectorNum);

    /**
     * @brief program a page (256 bytes) on memory at its starting address
     * specified. if vector size is smaller than a whole page size there will be
     * no effects on other bytes of the page.
     * @param vector vector of data bytes to be programmed - its size must not
     * be bigger than a page size (256 byte).
     * @param size size of array
     * @param startAddress starting address of the page to be programmed. must
     * be a starting address!
     * @param verify double check on programmed data. true = enabled, false =
     * disabled. It may slow down the execution still it's a recommended option.
     * @return true/false - if the operation has been successful.
     * @warning startAddress must be a starting address of a page!
     * This function is only intended to program some bytes on flash by properly
     * resetting some bits to "0". If you want to store a page of data you'll
     * need to erase the sector associated with this page and then reprogram the
     * page.
     */
    bool pageProgram(const uint8_t* vector, const size_t size,
                     uint32_t startAddress, bool verify);

    /**
     * @brief erase the entire memory chip. since this operation is not so
     * reliable and it may take a lot of time, you should use block32Erase()
     * and block64Erase() methods.
     * @return true/false - if the operation has been successful
     * @warning THIS OPERATION WILL TAKE A WHILE !! (at least 1 min)
     */
    bool chipErase();

    /**
     * @brief erase a specific sector (4KB)
     * @param address generic address (24 bit) of the sector address space.
     * @return true/false - if the operation has been successful
     */
    bool sectorErase(uint32_t address);

    /**
     * @brief erase a specific block (32KB)
     * @param address generic address (24 bit) of the block address space.
     * @warning THIS OPERATION COULD TAKE A WHILE !
     * @return true/false - if the operation has been successful
     */
    bool block32Erase(uint32_t address);

    /**
     * @brief erase a specific block (64KB)
     * @param address generic address (24 bit) of the block address space.
     * @warning THIS OPERATION COULD TAKE A WHILE!
     * @return true/false - if the operation has been successful
     */
    bool block64Erase(uint32_t address);

    /**
     * @brief read a single byte at a specific address
     * @param address byte address (24 bit), it can be any address
     * @return byte value
     */
    uint8_t readByte(uint32_t address);

    /**
     * @brief program a single byte at a specific address
     * @param data byte value
     * @param address byte address (24 bit), it can be any address
     * @param verify double check on programmed byte. true = enabled, false =
     * disabled.
     * @return true/false - if the operation has been successful
     * @warning This function is only intended to program some bytes on flash by
     * resetting some bits to "0". If you want to store a byte of data, you'll
     * need to perform an erase operation and then program the byte.
     */
    bool byteProgram(uint8_t data, uint32_t address, bool verify);

    /**
     * @brief make the flash go back to power-on default state. if the device
     * is performing any program/erase operation, that operation will be deleted
     * and some data could be lost!
     * @warning THIS FUNCTION MAY TAKE A WHILE !
     */
    void softwareReset();

    /**
     * @brief check if the memory is currently executing any operation like
     * program/erase
     * @return true/false - if there's an operation in progress or not.
     */
    bool isInProgress();

private:
    // pointer to access low-level QUADSPI peripheral registers
    QUADSPI_TypeDef* Qspi;

    /**
     * @brief check result of last erase operation
     * @return true = success / false = fail
     */
    bool checkErase();

    /**
     * @brief check result of last program operation
     * @return true = success / false = fail
     */
    bool checkProgram();

    /**
     * @brief enable modifying (program/erase) data on memory
     * @return true/false if write enable command has been properly performed by
     * the flash
     * @warning This function issue a "write enable" command to the flash. In
     * order to modify any data on memory you have to issue a "write enable"
     * command, also you should always issue a "write disable" command, when you
     * have done modifying data.
     */
    bool writeEnable();

    /**
     * @brief disable modifying (program/erase) data on memory
     * @return true/false if write disable command has been properly performed
     * by the flash
     * @warning This function issue a "write disable" command to the flash. In
     * order to modify any data on memory you have to issue a "write enable"
     * command, also you should always issue a "write disable" command, when you
     * have done modifying data.
     */
    bool writeDisable();

    /**
     * @brief read memory's status register
     * @return status register value (8 bit)
     */
    uint8_t readStatusReg();

    /**
     * @brief read memory's security register
     * @return security register value (8 bit)
     */
    uint8_t readSecurityReg();

    /* QuadSpi peripheral utility methods */
    // enable QUADSPI peripheral
    void enable();

    // disable QUADSPI peripheral
    void disable();

    /**
     * @brief abort any ongoing operation and reset current configuration of
     * QUADSPI peripheral
     * @return true/false - if abort has been successful
     */
    bool abortReset();

    /**
     * @brief wait till the current operation on QUADSPI peripheral is ended
     * @return true/false - if timeout expires
     */
    bool waitBusy();

    /**
     * @brief wait till all the expected bytes have been transferred by QUADSPI
     * peripheral
     * @return true/false - if timeout expires
     */
    bool waitTransfer();

    // flag device initialised
    bool initialised = false;

    // foundamental flash memory commands
    enum Commands
    {
        // read unique ID of the memory
        READ_ID = 0x9F,

        // write enable, needs to be executed before writing any data
        WRITE_ENABLE = 0x06,

        // write disable
        WRITE_DISABLE = 0x04,

        // read status register
        READ_STATUS_REG = 0x05,

        // write status register
        WRITE_STATUS_REG = 0x01,

        // read security register
        READ_SECURITY_REG = 0x2B,

        // read configuration register
        READ_CONFIG_REGISTER = 0x15,

        // read data bytes from memory
        READ = 0x03,

        // write a page on memory.
        PAGE_PROGRAM = 0x02,

        // erase a specific sector of the memory
        SECTOR_ERASE = 0x20,

        // erase a specific block of 32KB on the memory
        BLOCK_32_ERASE = 0x52,

        // erase a specific block of 64KB on the memory
        BLOCK_64_ERASE = 0xD8,

        // erase all data on the chip - THIS COULD TAKE A LONG TIME !
        ERASE_CHIP = 0xC7,

        // reset enable command
        RESET_ENABLE = 0x66,

        // reset memory, reset enable command should be executed first
        RESET_MEMORY = 0x99
    };
};