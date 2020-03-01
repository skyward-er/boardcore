/*
 * MultiFlashController.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: luca
 */

#include "MultiFlashController.h"
#include <Singleton.h>
#include <diagnostic/NewLogger.h>

using logging::logger;
/* stub crc class */

namespace flashmemory
{
class CRCHelper
{
 public:
  static uint16_t calcCRC16(uint8_t* data, uint32_t size)
  {
    return 0xABCD;
  }
};

//miosix::Queue<FlashDataBlock,20> MultiFlashController::block_queue_;

MultiFlashController::MultiFlashController()
    : ActiveObject::ActiveObject()
{
  flash0_ = Singleton<FlashController<MemoryBus0>>::getInstance();

  //block_queue_ = new miosix::Queue<FlashDataBlock, 5>();
}

MultiFlashController::~MultiFlashController()
{
}

/**
 *   @brief  Infinite loop that process the block queue.
 */
void MultiFlashController::run()
{
  FlashDataBlock to_flash;
  while (true)
  {
    block_queue_.waitUntilNotEmpty();

    block_queue_.get(to_flash);

    bool result = true;

    result = result && flash0_->writeDataBlock(&to_flash);

    //TODO: Also write on 2nd and 3rd flash memory
    //result = result && flash1_->writeBlock(&to_flash);
    //result = result && flash1_->writeBlock(&to_flash);

    if (!result)
    {
      logger.error(logtag(), "Error writing blocks!");
    }

  }
}

/**
 *   @brief  Insert into the fixed queue a new FlashDataBlock to save in flash.
 *
 *   @param  buffer data do be saved
 *   @param  size the size of the data (must be < FLASH_BUFFER_SIZE)
 */
void MultiFlashController::addData(const uint8_t* buffer, size_t size)
{
  /* limit data to FLASH_BUFFER_SIZE */
  uint16_t safe_size = (size <= FLASH_BUFFER_SIZE ? size : FLASH_BUFFER_SIZE);
  FlashDataBlock to_save = { };

  // create the Flash Data Block.
  to_save.id = block_counter++;
  to_save.timestamp = miosix::getTick();
  memcpy((uint8_t*) to_save.buffer, (uint8_t*) buffer, safe_size);

  //TODO crc of the whole block with toSave.crc = 0x0000?
  to_save.crc = CRCHelper::calcCRC16((uint8_t*) to_save.buffer, safe_size);

  block_queue_.put(to_save);
}

}  //namespace flashmemory
