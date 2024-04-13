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

using namespace miosix;
using namespace Boardcore;

/*  driver for MX25R3235FM1IL0 flash memory chip
 *  model of flash memory on compute unit: MX25R3235FM1IL0 4MB, device ID:
 * 0xC22816 FLASH memory space organisation:
 *   - number of byte in the memory: 4.194.304 >> about 4 MB
 *   - "pages"   of 256 Byte each   - number of pages:   16.384
 *   - "sector"  of about 4  KByte  - number of sector:   1.024
 *   - "block32" of about 32 KByte  - number of block32:    128
 *   - "block64" of about 64 KByte  - number of block64:     64
 */

namespace FlashMemory
{

// device id of the flash
static const uint32_t DEVICE_ID = 0xC22816;

static const uint32_t PAGES_PER_SECTOR    = 16;
static const uint32_t SECTORS_PER_BLOCK32 = 8;

static const uint32_t BLOCK32_NUM = 128;
static const uint32_t BLOCK64_NUM = 64;
static const uint32_t SECTORS_NUM = 1024;
static const uint32_t PAGES_NUM   = 16384;
static const uint32_t BYTES_NUM   = 4194304;

// sizes of each area in byte
static const uint32_t PAGE_SIZE    = 256;
static const uint32_t SECTOR_SIZE  = PAGE_SIZE * PAGES_PER_SECTOR;
static const uint32_t BLOCK32_SIZE = SECTOR_SIZE * SECTORS_PER_BLOCK32;
static const uint32_t BLOCK64_SIZE = BLOCK32_SIZE * 2;

// memory size in byte
static const uint32_t MEMORY_SIZE = BLOCK64_SIZE * BLOCK64_NUM;  // about 4 MB

};  // namespace FlashMemory

// QUADSPI peripheral utility-stuff
namespace QSPI
{

// enable/disable quadspi
void enable();
void disable();

// init peripheral clock and GPIO
void init();

// abort any ongoing operation and reset configuration register (CCR)
void abort_reset();

// wait till the current operation is ended
void waitBusy();

// wait till all the expected bytes have been transferred
void waitTransfer();

}  // namespace QSPI

class qspi_flash
{

public:
    // constructor
    qspi_flash();

    // init qspi peripheral which is connected to the flash
    void init();

    // read unique device ID
    uint32_t readID();

    // test the communication with the device by checking on its ID
    bool test();

    // write a vector into a sector of the flash
    bool write_vector(std::vector<uint8_t>& vector, uint32_t sector_num,
                      bool verify_write);

    // read an std::vector starting by a specific sectro number
    bool read_sector(std::vector<uint8_t>& vector, uint32_t sector_num);

    // program a page into the flash memory
    bool page_program(std::vector<uint8_t>& vector, uint32_t start_address,
                      bool verify);

    // erase the entire memory chip - THIS OPERATION WILL TAKE A WHILE!!
    bool chip_erase();

    // erase the sector which contains the address (24 bit) specified
    bool sector_erase(uint32_t address);

    // erase a block (32K) which contains the address (24 bit) specified
    bool block32_erase(uint32_t address);

    // erase a block (32K) which contains the address (24 bit) specified
    bool block64_erase(uint32_t address);

    // read a byte at a specific address (24 bit) in memory
    uint8_t read_byte(uint32_t address);

    // program a byte at a specific address (24 bit) in memory
    bool byte_program(uint8_t data, uint32_t address, bool verify);

    // ATTENTION it may take a while! - makes the flash return to power-on
    // default state
    void software_reset();

    // check last erase operation result
    bool check_erase();

    // check last program operation result
    bool check_program();

    // enable writing
    void write_enable();

    // read status register of the flash memory
    uint8_t read_status_reg();

    // disable writing
    void write_disable();

    // read security register of the flash
    uint8_t read_security_reg();

private:
    // wait till flash has executed the current operation
    void waitProgress();

    // check if flash is executing some operation (program/erase or write
    // registers)
    bool isInProgress();

    // true = quadspi initialised - false = not initialised yet 
    bool initialised = false; 

    // most important flash memory commands
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