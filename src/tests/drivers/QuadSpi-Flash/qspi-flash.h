#include <miosix.h>
#include <utils/ClockUtils.h>

#pragma once

using namespace miosix;
using namespace Boardcore;


/*  model of flash memory on compute unit: MX25R3235FM1IL0 4MB
 *   FLASH memory space organisation:
 *   - number of byte in the memory: 4.194.304 >> about 4 MB
 *   - "pages"   of 256 Byte each   - number of pages:   16.384
 *   - "sector"  of about 4  KByte  - number of sector:   1.024
 *   - "block32" of about 32 KByte  - number of block32:    128
 *   - "block64" of about 64 KByte  - number of block64:     64
*/

namespace FlashMemory {

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

};


// QUADSPI peripheral utility-stuff
namespace QSPI {

    // enable/ disbale quadspi
    void enable(); 
    void disable(); 
    
    // init peripheral clock and GPIO 
    void init(); 

    // wait till the current operation is ended
    void waitBusy(); 
    
    // wait till all the expected bytes have been transferred 
    void waitTransfer(); 

}


class qspi_flash {

public:

    // constructor 
    qspi_flash();

    // enable writing on memory 
    void write_enable(); 

    // read unique device ID
    uint32_t readID();

private:  
    
    // disable writing on memory 
    void write_disable(); 

    // read status register of the flash memory 
    uint8_t read_status_reg();  
    
    // most important flash memory commands 
    enum Commands
    {
        // read unique ID of the memory 
        READ_ID = 0x9F,

        // write enable, needs to be executed before modify any data 
        WRITE_ENABLE  = 0x06,
        
        // write disable
        WRITE_DISABLE = 0x04,

        // read status register 
        READ_STATUS_REG = 0x05,

        // write status register 
		WRITE_STATUS_REG = 0x01,

        // read configuration register 
        READ_CONFIG_REGISTER = 0x15,

        // read data from memory  
        READ = 0x03,

        // write a page on memory.
        PROGRAM_PAGE = 0x02,

        // erase a specific sector of the memory 
        SECTOR_ERASE = 0x20, 

        // erase all data on the chip - THIS COULD TAKE A LONG TIME !
        ERASE_CHIP = 0xC7,

        // reset enable command
        RESET_ENABLE = 0x66,

        // reset memory, reset enable command should be executed first
        RESET_MEMORY = 0x99
    };

    // important bits to be checked in the memory status register 
    enum bits_status_reg {
        WEL_POS = 1, 
        WIP_POS = 0
    };

};