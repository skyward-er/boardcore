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

#ifndef FLASHCONTROLLERTESTS_H
#define FLASHCONTROLLERTESTS_H

#include <Common.h>
#include <helper/MultiFlashController.h>
#include <diagnostic/NewLogger.h>
#include <drivers/timer.h>
#include <Tests.h>
#include <vector>
#include <limits>
#include <string>

using std::vector;
using logging::logger;
using namespace flashmemory;

namespace testing
{
namespace flashmemorytests
{

template<typename MemorySPI>
class FlashTest : public Test
{
 public:

  FlashTest(MultiFlashController* multi)
      : Test(), multi(multi), controller(multi->flash0_), driver(
          controller->flash_)
  {

  }
 protected:
  typedef FlashController<MemorySPI> FlashControllerType;

  void controller_setBlockIndex(uint32_t block_index)
  {
    controller->block_index_ = block_index;
  }

  uint32_t controller_getBlockIndex()
  {
    return controller->block_index_;
  }

  bool controller_format()
  {
#ifdef FLASH_TEST
    return FlashControllerType::format();
#else
    return FlashControllerType::fastFormat();
#endif
  }

  void controller_init()
  {
    controller->init();
  }

  bool controller_readSectorHeader(uint32_t sector, SectorHeader* header)
  {
    return FlashControllerType::readSectorHeader(sector, header);
  }

  bool controller_readBlock(uint32_t block_index, uint8_t* buffer)
  {
    return FlashControllerType::readBlock(block_index, buffer);
  }

  bool controller_readDataBlock(uint32_t block_index, FlashDataBlock* block)
  {
    return FlashControllerType::readDataBlock(block_index, block);
  }

  bool controller_writeBlock(FlashDataBlock* block)
  {
    return controller->writeDataBlock(block);
  }

  bool controller_writeSectorHeader(uint32_t sector, SectorHeader& sheader)
  {
    return FlashControllerType::writeSectorHeader(sector, sheader);
  }

  MultiFlashController* multi;
  FlashController<MemorySPI>* controller;
  FlashDriver<MemorySPI>* driver;
};

/*
 * Copy this to create a new FlashTest
 *

 template<typename MemorySPI>
 class EmptyFlashTest : public FlashTest<MemorySPI>
 {
 typedef FlashTest<MemorySPI> Base;

 public:

 EmptyFlashTest(MultiFlashController* multi)
 : Base::FlashTest(multi)
 {

 }

 bool setup()
 {
 return true;
 }

 bool run()
 {
 return true;
 }

 string name()
 {
 return "EmptyFlashTest";
 }
 };

 */

class CompareFlashHeaderTest : public Test
{
 public:

  CompareFlashHeaderTest()
      : Test()
  {

  }

  bool run()
  {
    bool success = true;
    FlashHeader header1, header2;

    bool test = header1 == header2;
    success = success && test;
    logger.info(logtag(), "Compare equal headers: ", (test ? "OK" : "Error"));

    header1.read_only = 54;

    test = header1 == header2;
    success = success && (test == false);
    logger.info(logtag(), "Compare different headers: ",
                (test == false ? "OK" : "Error"));

    return success;
  }

  string name()
  {
    return "CompareFlashHeaderTest";
  }

 private:
  static string logtag()
  {
    return "CompareFHTest";
  }
};

template<typename MemorySPI>
class MeasureRebootTimeTest : public FlashTest<MemorySPI>
{
  typedef FlashTest<MemorySPI> Base;
  typedef Timer32<2> Timer2;
 public:
  MeasureRebootTimeTest(MultiFlashController* multi)
      : Base::FlashTest(multi)
  {

  }

  bool run()
  {
#ifdef FLASH_TEST
    logger.warning(logtag(), "You are using the mockup memory!");
#endif
    logger.info(logtag(), "Running 100 read repetitions!");
    uint8_t result;
    SectorHeader header;
    float avg, min = std::numeric_limits<float>::max(), max = 0;
    Timer2::start();
    for (int j = 0; j < 100; j++)
    {
      uint32_t t0 = Timer2::tick();
      for (uint32_t i = 0; i < SECTORS_NUM; i++)
      {
        Base::driver->read(&result, i * SECTOR_SIZE, (uint8_t*) (&header),
                           sizeof(SectorHeader));
        if (result != RESULT_OK)
        {
          break;
        }
      }
      uint32_t t1 = Timer2::tick();
      float t = Timer2::milliSeconds(t1 - t0);
      avg += t;
      if (t < min)
      {
        min = t;
      } else if (t > max)
      {
        max = t;
      }
    }
    Timer2::stop();
    avg = avg / 100;
    if (result == RESULT_OK)
    {
      logger.info(logtag(), "Avg time: ", avg, " ms");
      logger.info(logtag(), "Min time: ", min, " ms, Max time: ", max, " ms");
      return true;
    } else
    {
      return false;
    }
  }

  string name()
  {
    return "MeasureWorstRebootTimeTest";
  }

 private:
  static string logtag()
  {
    return "RebootTimeTest";
  }
};

template<typename MemorySPI>
class FlashHeaderTest : public FlashTest<MemorySPI>
{
  typedef FlashTest<MemorySPI> Base;
  //using Base::driver;
  //using Base::controller;

 public:

  FlashHeaderTest(MultiFlashController* multi)
      : Base::FlashTest(multi)
  {

  }

  bool setup()
  {
    return cleanFirstSubsector();
  }

  bool run()
  {
    FlashHeader orig_header, read_header;
    generateRandomHeader(&orig_header);
    orig_header.first_boot = FIRST_BOOT_KEY;
    logger.info(logtag(), "--SUBTEST 1: Write & read over clean memory");

    if (!writeAndReadHeader(orig_header, read_header, false))
    {
      return false;
    }
    if (orig_header != read_header)
    {
      logger.error(logtag(), "Original & written headers are not the same!");
      return false;
    }
    logger.info(logtag(), "Success.");

    orig_header.first_boot = 0;
    logger.info(logtag(), "--SUBTEST 2: Overwrite first_boot");

    if (!writeAndReadHeader(orig_header, read_header, false))
    {
      return false;
    }
    if (orig_header != read_header)
    {
      logger.error(logtag(), "Original & written headers are not the same!");
      return false;
    }
    logger.info(logtag(), "Success.");

    logger.info(logtag(), "--SUBTEST 3: Write & read over dirty memory");
    FlashHeader dirtywrite_header;
    generateRandomHeader(&dirtywrite_header);

    if (Base::FlashControllerType::writeFlashHeader(dirtywrite_header, false))
    {
      logger.error(
          logtag(),
          "Function writeFlashHeader should have returned RESULT_CHECK_FAIL!");
      return false;
    }
    logger.info(logtag(), "Success.");

    logger.info(logtag(),
                "--SUBTEST 4: Write with erase & read over dirty memory");
    generateRandomHeader(&dirtywrite_header);

    if (!writeAndReadHeader(dirtywrite_header, read_header, true))
    {
      return false;
    }
    if (dirtywrite_header != read_header)
    {
      logger.error(logtag(), "Original & written headers are not the same!");
      return false;
    }
    logger.info(logtag(), "Success.");
    return true;
  }

  string name()
  {
    return "WriteFlashHeaderTest";
  }

 private:
  static string logtag()
  {
    return "WrtFHeaderTest";
  }

  bool cleanFirstSubsector()
  {
    uint8_t result;
    Base::driver->enableErase();
    Base::driver->eraseSubsector(&result, 0);

    return result == RESULT_OK;
  }

  void generateRandomHeader(FlashHeader* header)
  {
    header->start_index = (uint32_t) rand();
    header->read_only = (uint32_t) rand();
    header->first_boot = (uint32_t) rand();
  }

  bool writeAndReadHeader(FlashHeader& write_header, FlashHeader& read_header,
                          bool erase)
  {
    if (!Base::FlashControllerType::writeFlashHeader(write_header, erase))
    {
      logger.error(logtag(), "Error writing header");
      return false;
    }

    if (!Base::FlashControllerType::readFlashHeader(&read_header))
    {
      logger.error(logtag(), "Error reading header");
      return false;
    }

    return true;
  }
};

template<typename MemorySPI>
class FlashBlockTest : public FlashTest<MemorySPI>
{
  typedef FlashTest<MemorySPI> Base;

 public:

  FlashBlockTest(MultiFlashController* multi)
      : Base::FlashTest(multi)
  {

  }

  bool setup()
  {
    if (Base::controller_format())
    {
      Base::controller_init();
      return true;
    }
    return false;
  }

  bool run()
  {
    logger.info(logtag(), "--SUBTEST 1: Just write and read");
    if (!subtest1())
      return false;
    logger.info(logtag(), "Success.");

    logger.info(logtag(), "--SUBTEST 2: Write near the end of a sector");
    if (!subtest2())
      return false;
    logger.info(logtag(), "Success.");

    logger.info(logtag(), "--SUBTEST 3: Write at the end of the memory");
    if (!subtest3())
      return false;
    logger.info(logtag(), "Success.");

    return true;
  }

  string name()
  {
    return "FlashBlockTest";
  }

 private:
  static string logtag()
  {
    return "FBlockTest";
  }

  bool subtest1()
  {
    uint32_t block_index = SUBSECTOR_SIZE / BLOCK_SIZE;

    Base::controller_setBlockIndex(block_index);
    FlashDataBlock write_block, read_block;
    generateDataBlock(&write_block);

    if (!writeBlock(&write_block))
    {
      logger.error(logtag(), "Cannot write block");
      return false;
    }

    if (!readBlock(block_index, &read_block))
    {
      logger.error(logtag(), "Cannot read block");
      return false;
    }

    if (read_block != write_block)
    {
      logger.error(logtag(), "Written & read block do not match!");
      return false;
    }
    return true;
  }

  bool subtest2()
  {
    uint32_t block_index = SECTOR_SIZE / BLOCK_SIZE - 1;

    Base::controller_setBlockIndex(block_index);

    FlashDataBlock write_blocks[2];
    FlashDataBlock read_blocks[2];
    for (int i = 0; i < 2; i++)
    {
      generateDataBlock(&write_blocks[i]);
      if (!writeBlock(&write_blocks[i]))
      {
        logger.error(logtag(), "Cannot write block ", i + 1);
        return false;
      }
    }

    SectorHeader sheader;
    if (!Base::controller_readSectorHeader(1, &sheader))
    {
      logger.error(logtag(), "Cannot read sector header");
      return false;
    }

    if (sheader.content != SECTOR_WRITTEN)
    {
      logger.error(logtag(), "Sector header not correctly written");
      logger.error(logtag(), "Sector header content: 0x%02X; should be: 0x%02X",
                   sheader.content, SECTOR_WRITTEN);
      return false;
    }

    if (!readBlock(block_index, &read_blocks[0]))
    {
      logger.error(logtag(), "Cannot read block 1");
      return false;
    }

    if (!readBlock(block_index + 2, &read_blocks[1]))
    {
      logger.error(logtag(), "Cannot read block 2");
      return false;
    }

    if (!compareBlocks(write_blocks, read_blocks, 2))
    {
      logger.error(logtag(), "Written & read blocks do not match.");
      return false;
    }
    return true;
  }

  bool subtest3()
  {
    //Last block of the memory
    uint32_t block_index = MEMORY_SIZE / BLOCK_SIZE - 1;
    Base::controller_setBlockIndex(block_index);

    //Keep the filesystem consistent.
    SectorHeader sheader;
    sheader.content = SECTOR_WRITTEN;
    Base::controller_writeSectorHeader(MEMORY_SIZE / SECTOR_SIZE - 1, sheader);

    FlashDataBlock write_blocks[2];
    FlashDataBlock read_blocks[2];

    for (int i = 0; i < 2; i++)
    {
      generateDataBlock(&write_blocks[i]);
    }

    if (!writeBlock(&write_blocks[0]))
    {
      logger.error(logtag(),
                   "Write block failed on the last available address!");
      return false;
    }

    if (writeBlock(&write_blocks[1]))
    {
      logger.error(
          logtag(),
          "Write block didn't fail while writing outside of the memory!");
      return false;
    }
    if (!readBlock(block_index, &read_blocks[0]))
    {
      logger.error(logtag(), "Cannot read block 1");
      return false;
    }

    if (readBlock(block_index + 2, &read_blocks[1]))
    {
      logger.error(
          logtag(),
          "readBlock didn't fail while reading outside of the memory!");
      return false;
    }

    if (read_blocks[0] != write_blocks[0])
    {
      logger.error(logtag(), "Written & read blocks do not match.");
      return false;
    }

    return true;
  }

  bool writeBlock(FlashDataBlock* block)
  {
    return Base::controller_writeBlock(block);
  }

  bool readBlock(uint32_t block_index, FlashDataBlock* block)
  {
    return Base::controller_readBlock(block_index, (uint8_t*) (block));
  }

  bool compareBlocks(FlashDataBlock* blocks1, FlashDataBlock* blocks2,
                     size_t num_blocks)
  {
    for (size_t i = 0; i < num_blocks; i++)
    {
      if (blocks1[i] != blocks2[i])
      {
        return false;
      }
    }
    return true;
  }

  void generateDataBlock(FlashDataBlock* generated)
  {
    generated->id = (uint32_t) rand();
    generated->crc = (uint16_t) rand();
    generated->timestamp = (uint32_t) rand();

    for (int i = 0; i < FLASH_BUFFER_SIZE; i++)
    {
      generated->buffer[i] = (uint8_t) (rand() % 256);
    }
  }
};

template<typename MemorySPI>
class MultiFlashCTRLTest : public FlashTest<MemorySPI>
{
  typedef FlashTest<MemorySPI> Base;

 public:

  MultiFlashCTRLTest(MultiFlashController* multi)
      : Base::FlashTest(multi)
  {

  }

  bool setup()
  {
    if (!Base::controller_format())
    {
      return false;
    }
    Base::controller_init();
    if (Base::controller->getStatus()
        != Base::FlashControllerType::Status::WRITE_READY)
    {
      logger.error(logtag(), "Error initializing flash controller.");
      return false;
    }
    return true;
  }

  bool run()
  {
    logger.info(logtag(), "--SUBTEST 1: Write @ beginning");
    if (!subtest1())
      return false;
    logger.info(logtag(), "Success.");

    return true;
  }

  string name()
  {
    return "EmptyFlashTest";
  }

 private:
  bool subtest1()
  {
    uint32_t index = Base::controller_getBlockIndex();

    uint8_t rand_buff1[FLASH_BUFFER_SIZE];
    uint8_t rand_buff2[FLASH_BUFFER_SIZE];
    Base::multi->addData(rand_buff1, FLASH_BUFFER_SIZE);
    Base::multi->addData(rand_buff2, FLASH_BUFFER_SIZE);

    Thread::sleep(500); //Wait for data to be written

    logger.info(logtag(), "Reading blocks.");
    FlashDataBlock read_block;

    if(!Base::controller_readDataBlock(index, &read_block))
    {
      logger.error(logtag(), "Error reading first block.");
      return false;
    }

    if(memcmp(rand_buff1, &(read_block.buffer), FLASH_BUFFER_SIZE) != 0)
    {
      logger.error(logtag(), "Read block 1 is not the same as written one.");
      return false;
    }

    if(!Base::controller_readDataBlock(index+1, &read_block))
    {
      logger.error(logtag(), "Error reading second block.");
      return false;
    }

    if(memcmp(rand_buff2, &(read_block.buffer), FLASH_BUFFER_SIZE) != 0)
    {
      logger.error(logtag(), "Read block 2 is not the same as written one.");
      return false;
    }

    return true;
  }

  static string logtag()
  {
    return "MultiFlashCTRLTest";
  }
};

}   //namespace flashmemorytests
}   //namespace testing

#endif /*FLASHCONTROLLERTESTS_H*/
