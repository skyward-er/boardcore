/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Matteo Piazzolla
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

#include <Singleton.h>
#include <diagnostic/FaultCounter.h>
#include <diagnostic/NewLogger.h>
#include <drivers/memory/FlashDriverInclude.h>
#include <miosix.h>

using logging::logger;

// Forward declaration
namespace testing
{
namespace flashmemorytests
{
template <typename>
class FlashTest;
}
}  // namespace testing

namespace flashmemory
{

/*
 16 pagine da 256 byte
 1000000 pagine in un banco
 62500 sottosettori
 */
static const uint16_t CONTROLLER_VERSION = 1;
static const uint16_t BOARD_ID           = 0xAB;  // TBD

// BLOCK_SIZE deve essere un divisore di SUBSECTOR_SIZE
static const uint32_t BLOCK_SIZE = PAGE_SIZE;

/** Number of flash data blocks that fit in each sector.
 *  First block of each sector is reserved for the header.
 *  In the first sector, the entire first subsector is reserved for the header.
 */
static const uint32_t BLOCKS_PER_SECTOR = SECTOR_SIZE / BLOCK_SIZE;
static const uint32_t BLOCKS_NUM        = BLOCKS_PER_SECTOR * SECTORS_NUM;

static const uint32_t FIRST_BOOT_KEY = 0xF135B007;
static const uint32_t READ_ONLY_KEY  = 0x3EAD0271;

// How many sectors should we attempt to write in before we give up?
static const int MAX_SECTOR_ITERATIONS = 50;

struct FlashHeader
{
    const uint16_t version  = CONTROLLER_VERSION;
    const uint16_t board_id = BOARD_ID;

    uint32_t first_boot  = FIRST_BOOT_KEY;
    uint32_t start_index = 0;
    uint32_t read_only   = 0;

    bool operator==(const FlashHeader& other) const
    {
        return memcmp(this, &other, sizeof(FlashHeader)) == 0;
    }

    bool operator!=(const FlashHeader& other) const
    {
        return memcmp(this, &other, sizeof(FlashHeader)) != 0;
    }
};

static const uint8_t SECTOR_EMPTY   = 0x55;
static const uint8_t SECTOR_WRITTEN = 0x44;
static const uint8_t SECTOR_CORRUPT = 0x00;

struct SectorHeader
{
    uint8_t content = SECTOR_CORRUPT;  // Either one of SECTOR_EMPTY,
    // SECTOR_WRITTEN, SECTOR_CORRUPT
    bool operator==(const SectorHeader& other) const
    {
        return memcmp(this, &other, sizeof(SectorHeader)) == 0;
    }
};

static const uint32_t FLASH_BUFFER_SIZE =
    (BLOCK_SIZE) - (sizeof(uint32_t) * 2 + sizeof(uint16_t));

struct FlashDataBlock
{
    uint32_t id;
    uint32_t timestamp;
    uint8_t buffer[FLASH_BUFFER_SIZE];
    uint16_t crc;

    bool operator==(const FlashDataBlock& other) const
    {
        return memcmp(this, &other, sizeof(FlashDataBlock)) == 0;
    }

    bool operator!=(const FlashDataBlock& other) const
    {
        return memcmp(this, &other, sizeof(FlashDataBlock)) != 0;
    }
};

template <typename MemoryBus>
class FlashController : Singleton<FlashController<MemoryBus>>
{
    friend class Singleton<FlashController<MemoryBus>>;

    template <typename>
    friend class testing::flashmemorytests::FlashTest;

    typedef FlashController<MemoryBus> FlashControllerType;
    typedef FlashDriver<MemoryBus> FlashDriverType;

public:
    enum class Status
    {
        WRITE_READY          = 0,
        READ_ONLY            = 1,
        INITIALIZATION_ERROR = 2,
        WRITE_ERROR          = 3
    };

    void init()
    {
        status_      = Status::INITIALIZATION_ERROR;
        block_index_ = SUBSECTOR_SIZE / BLOCK_SIZE;

        FlashHeader header;

        if (!readFlashHeader(&header))
        {
            logger.critical(logtag(), "Cannot read flash header.");
            sFaultCounterMgr->Increment(Fault::F_MASTER_FLASH_CNTRL_FAULT);
            return;
        }

        if (header.read_only == READ_ONLY_KEY)
        {
            flash_->setReadOnly();
            status_ = Status::READ_ONLY;
        }
        else
        {
            if (header.first_boot == FIRST_BOOT_KEY)
            {
                // Update the first boot key
                header.first_boot = 0;

                if (!writeFlashHeader(header, false))
                {
                    logger.critical(logtag(),
                                    "Could not update first boot value.");
                    sFaultCounterMgr->Increment(
                        Fault::F_MASTER_FLASH_CNTRL_FAULT);
                    return;
                }

                if (header.start_index >= block_index_)
                {
                    block_index_ = header.start_index;
                }
                status_ = Status::WRITE_READY;
            }
            else
            {
                if (recoverFromReboot())
                {
                    status_ = Status::WRITE_READY;
                }
                else
                {
                    logger.critical(logtag(),
                                    "Couldn't find sector to write in.");
                    sFaultCounterMgr->Increment(
                        Fault::F_MASTER_FLASH_CNTRL_FAULT);
                }
            }
        }
    }

    FlashController::Status getStatus() { return status_; }

    uint32_t getBlockIndex() { return block_index_; }

    /**
     * @brief Writes a block onto the flash memory.
     *
     * @param block The block to be written on the flash
     */
    bool writeDataBlock(FlashDataBlock* block_buf)
    {

        if (status_ != Status::WRITE_READY)
        {
            logger.error(logtag(), "Writing is not enabled.");
            return false;
        }

        if (block_index_ >= BLOCKS_NUM)
        {
            logger.critical(logtag(), "Memory is full.");
            sFaultCounterMgr->Increment(Fault::F_MASTER_FLASH_CNTRL_FAULT);
            status_ = Status::WRITE_ERROR;
            return false;
        }

        // Enforce size <= BLOCK_SIZE
        size_t size = sizeof(FlashDataBlock) < BLOCK_SIZE
                          ? sizeof(FlashDataBlock)
                          : BLOCK_SIZE;

        // Beginning of a new sector. Check if we can write in it and update its
        // header
        if (block_index_ % BLOCKS_PER_SECTOR == 0)
        {
            if (!gotoNewSector())
            {
                logger.critical(logtag(), "Could not find sector to write in.");
                status_ = Status::WRITE_ERROR;
                sFaultCounterMgr->Increment(Fault::F_MASTER_FLASH_CNTRL_FAULT);
                return false;
            }
        }

        uint8_t result;
        flash_->write(&result, block_index_ * BLOCK_SIZE,
                      reinterpret_cast<uint8_t*>(block_buf), size);
        block_index_++;

        return result == RESULT_OK;
    }

    static bool readFlashHeader(FlashHeader* header)
    {
        uint8_t result;
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();
        flash->read(&result, 0, (uint8_t*)header, sizeof(FlashHeader));
        return result == RESULT_OK;
    }

    /**
     * Saves (and overwrites) the header in the first address on the memory.
     * @param header
     */
    static bool writeFlashHeader(const FlashHeader& header, bool erase = true)
    {
        uint8_t result;
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();
        if (erase)
        {
            flash->enableErase();
            flash->eraseSubsector(&result, 0);
            if (result != OpResultFlags::RESULT_OK)
            {
                return false;
            }
        }
        uint32_t size = sizeof(FlashHeader) > SUBSECTOR_SIZE
                            ? SUBSECTOR_SIZE
                            : sizeof(FlashHeader);

        flash->writeAndCheck(&result, 0, (uint8_t*)(&header), size);
        return result == RESULT_OK;
    }

    static bool readBlock(uint32_t block_index, uint8_t* buffer)
    {
        uint8_t result;
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();

        flash->read(&result, block_index * BLOCK_SIZE, buffer, BLOCK_SIZE);
        return result == RESULT_OK;
    }

    static bool readDataBlock(uint32_t block_index, FlashDataBlock* block)
    {
        // Flash header & sector headers are not *data* blocks.
        if (block_index % BLOCKS_PER_SECTOR == 0 ||
            block_index < SUBSECTOR_SIZE / BLOCK_SIZE)
        {
            return false;
        }

        readBlock(block_index, reinterpret_cast<uint8_t*>(block));
        return true;
    }

    /**
     * Saves (and overwrites) the header in the first address on the memory.
     * @param header
     */
    static bool writeSectorHeader(uint32_t sector, const SectorHeader& header)
    {
        uint8_t result;
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();

        uint32_t size = sizeof(SectorHeader) > BLOCK_SIZE
                            ? BLOCK_SIZE
                            : sizeof(SectorHeader);

        flash->writeAndCheck(&result, sector * SECTOR_SIZE, (uint8_t*)(&header),
                             size);

        return result == RESULT_OK;
    }

    static bool readSectorHeader(uint32_t sector, SectorHeader* header)
    {
        return readSectorHeader(sector, (uint8_t*)header);
    }

    static bool readSectorHeader(uint32_t sector, uint8_t* header_buff)
    {
        uint8_t result;
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();
        flash->read(&result, SECTOR_SIZE * sector, header_buff,
                    sizeof(SectorHeader));
        return result == RESULT_OK;
    }

    static bool format()
    {
        uint8_t result;
        logger.warning(logtag(), "THE MEMORY WILL BE FORMATTED IN 5 SECONDS");
        // Thread::sleep(5000);
        logger.info(logtag(), "Erasing...");
        // TODO: Flash leds for a few seconds as a warning
        eraseMemory(&result);
        if (result != OpResultFlags::RESULT_OK)
        {
            logger.error(logtag(), "Error: result=", logging::hex(result));
            return false;
        }
        logger.info(logtag(), "Writing flash header...");
        FlashHeader header = {};
        header.first_boot  = FIRST_BOOT_KEY;
        // header.magic_word = MAGIC_WORD;

        if (!writeFlashHeader(header, false))
        {
            logger.error(logtag(), "Error: result=", logging::hex(result));
            return false;
        }
        logger.info(logtag(), "Writing sector headers...");
        SectorHeader sheader;
        sheader.content = SECTOR_EMPTY;

        for (uint32_t sector = 1; sector < SECTORS_NUM; sector++)
        {
            if (!writeSectorHeader(sector, sheader))
            {
                logger.error(logtag(), "Error: result=", logging::hex(result),
                             " sector: ", sector);
                return false;
            }
        }
        logger.info(logtag(), "Success!");

        return true;
    }

    /**
     * @brief Formats the memory.
     * Use this only if the filesystem is in a good state: this fuction
     * reads every sector header and erases only sector where
     * header content != SECTOR_EMPTY
     *
     * @warning Use this only for testing!
     */
    static bool fastFormat()
    {
        logger.warning(logtag(), "!!!THE MEMORY IS ABOUT TO BE FORMATTED!!!\n");

        logger.info(logtag(), "Formatting...\n");
        // TODO: Flash leds for a few seconds as a warning
        // Always erase the first sector
        if (!erase(0, SUBSECTORS_PER_SECTOR))
        {
            logger.error(logtag(), "Couldn't erase sector 0");
            return false;
        }

        // Then erase only the written/broken sectors
        uint32_t s;
        for (s = 1; s < SECTORS_NUM; s++)
        {
            SectorHeader header;
            bool read_result = readSectorHeader(s, &header);
            if (read_result && header.content != SECTOR_EMPTY)
            {
                // Erase data in this sector
                if (!erase(s * SUBSECTORS_PER_SECTOR,
                           (s + 1) * SUBSECTORS_PER_SECTOR))
                {
                    logger.error(logtag(), "Couldn't erase sector ", s);
                    return false;
                }

                // Rewrite the header
                SectorHeader sheader;
                sheader.content = SECTOR_EMPTY;
                if (!writeSectorHeader(s, sheader))
                {
                    logger.error("Error writing sector header in sector ", s);
                    return false;
                }
            }
            else if (!read_result)
            {
                logger.error(logtag(), "Error reading sector header in sector ",
                             s);
                return false;
            }
        }

        // Format the memory
        FlashHeader header{};
        header.first_boot = FIRST_BOOT_KEY;

        if (!writeFlashHeader(header, false))
        {
            logger.error(logtag(), "Couldn't write flash header.");
            return false;
        }

        return true;
    }

    static bool eraseMemory(uint8_t* result)
    {
        return erase(0, SUBSECTORS_NUM);
    }

    /**
     * Erases the memory between the given subsectors
     * Only erases a subsector if data has been written into it,
     * to avoid wasting erase cycles.
     *
     * @brief Erases The memory between the given bounds.
     * @param result Erase successful or not
     * @param from Address belonging to the first subsector to be erased
     * @param to Address belonging to the last subsector to be erased
     * @return
     */
    static bool erase(uint32_t from_subsector, uint32_t to_subsector)
    {
        uint8_t result;
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();
        uint8_t* buffer        = new uint8_t[PAGE_SIZE];

        std::stringstream
            stream;  // To save strings to write in a single log line
        stream << "Erasing subsectors: ";

        // Erase from the start of the subsector corresponding to "from"
        uint32_t from_addr = from_subsector * SUBSECTOR_SIZE;

        // To the end of the subsector corresponding to "to".
        uint32_t to_addr = to_subsector * SUBSECTOR_SIZE;

        // Read memory in blocks of PAGE_SIZE, if a byte is != 0xFF, erase the
        // subsector and start reading again from the next one.
        while (from_addr < to_addr)
        {
            flash->read(&result, from_addr, buffer, PAGE_SIZE);

            if (result == OpResultFlags::RESULT_OK)
            {
                bool erase = false;
                for (uint32_t i = 0; i < PAGE_SIZE; i++)
                {
                    erase = buffer[i] != 0xFF;

                    if (erase)
                        break;
                }

                if (erase)
                {
                    stream << from_addr / SUBSECTOR_SIZE << ", ";

                    flash->enableErase();
                    flash->eraseSubsector(&result, from_addr);
                    if (result == OpResultFlags::RESULT_OK)
                    {
                        // Move to the beginning of the next subsector
                        from_addr =
                            (from_addr / SUBSECTOR_SIZE + 1) * SUBSECTOR_SIZE;
                    }
                    else
                    {
                        logger.error(logtag(),
                                     "Error erasing subsector: result=0x",
                                     logging::hex(result));
                        break;
                    }
                }
                else
                {
                    from_addr = from_addr + PAGE_SIZE;
                }
            }
            else
            {
                logger.error(logtag(), "Error reading memory: result=0x",
                             logging::hex(result));
                break;
            }
        }

        stream << "--";

        logger.info(logtag(), stream.str());
        delete buffer;

        return from_addr >= to_addr;
    }

    static bool isMemoryClean(uint8_t* result)
    {
        FlashDriverType* flash = Singleton<FlashDriverType>::getInstance();
        uint8_t buffer[BLOCK_SIZE];

        // Check from the start of the subsector corresponding to "from"
        uint32_t addr = SUBSECTOR_SIZE;

        bool clean = true;
        while (addr < MEMORY_SIZE)
        {
            if (addr % SECTOR_SIZE == 0)
            {
                SectorHeader header;
                bool read_result =
                    readSectorHeader(addr / SECTOR_SIZE, &header);
                if (!read_result || header.content != SECTOR_EMPTY)
                {
                    logger.warning(logtag(), "Wrong sector header @", addr);
                    clean = false;
                }
            }
            else
            {
                flash->read(result, addr, buffer, BLOCK_SIZE);
                if (*result == OpResultFlags::RESULT_OK)
                {
                    for (uint32_t i = 0; i < BLOCK_SIZE; i++)
                    {
                        if (buffer[i] != 0xFF)
                        {
                            logger.warning(logtag(), "Dirty block @", addr);
                            clean = false;
                            break;
                        }
                    }
                }
                else
                {
                    logger.warning(logtag(), "Error reading at %d\n", addr);
                    break;
                }
            }

            addr = addr + BLOCK_SIZE;
        }

        if (clean)
        {
            logger.info(logtag(), "Memory is clean.");
        }
        return clean;
    }

private:
    FlashController() { flash_ = Singleton<FlashDriverType>::getInstance(); }

    /**
     * @brief Restores the state after a reboot.
     *
     * Restores the state after a reboot.
     * Sequentially search for the first empty sector and update block_index_
     * accordingly
     *
     * @return Empty sector found or not
     */
    bool recoverFromReboot()
    {
        // Start from the second sector
        uint32_t sector = 1;
        SectorHeader header;

        do
        {
            bool r = readSectorHeader(sector, &header);
            if (r && header.content == SECTOR_EMPTY)
            {
                block_index_ = sector * BLOCKS_PER_SECTOR;
                return true;
            }
        } while (++sector < SECTORS_NUM);

        return false;
    }

    /**
     * Updates block_index_ to point to the next available sector.
     *
     * @return True if a new sector is found
     */
    bool gotoNewSector()
    {
        uint32_t sector = block_index_ / BLOCKS_PER_SECTOR;
        SectorHeader readHeader, writeHeader;

        // Look for an empty sector
        bool read_result;
        int i = 0;
        do
        {
            read_result = readSectorHeader(sector, &readHeader);

        } while (((read_result && readHeader.content != SECTOR_EMPTY) ||
                  !read_result) &&
                 ++sector < SECTORS_NUM && ++i < MAX_SECTOR_ITERATIONS);

        block_index_ = sector * BLOCKS_PER_SECTOR;

        if (read_result && readHeader.content == SECTOR_EMPTY)
        {
            writeHeader.content = SECTOR_WRITTEN;

            if (!writeSectorHeader(sector, writeHeader))
            {
                logger.error(logtag(), "Error writing sector header.");
                return false;
            }
            block_index_++;
        }
        else
        {
            logger.error(logtag(),
                         "Error: could not find empty sector to write in.");
            return false;
        }

        return true;
    }

    static std::string logtag() { return "FlashCTRL"; }

    FlashDriverType* flash_;
    uint32_t block_index_ = 0;
    // Value will be changed if initialization is successful
    Status status_ = Status::INITIALIZATION_ERROR;
};

}  // namespace flashmemory
