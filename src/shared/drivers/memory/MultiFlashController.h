/*
 * MultiFlashController.h
 *
 *  Created on: Sep 13, 2017
 *      Author: luca
 */
#ifndef MULTIFLASHCONTROLLER_H
#define MULTIFLASHCONTROLLER_H

#include <Common.h>
#include <ActiveObject.h>
#include <drivers/BusTemplate.h>
#include "FlashController.h"

using miosix::Gpio;

//Forward declaration
namespace testing
{
namespace flashmemorytests
{
template<typename >
class FlashTest;
}
}

namespace flashmemory
{

class MultiFlashController : public Singleton<MultiFlashController>, ActiveObject
{
  friend class Singleton<MultiFlashController>;

  template<typename >
  friend class testing::flashmemorytests::FlashTest;

  typedef Gpio<GPIOA_BASE, 5> GpioSck;
  typedef Gpio<GPIOA_BASE, 6> GpioMiso;
  typedef Gpio<GPIOA_BASE, 7> GpioMosi;
  typedef BusSPI<1, GpioMosi, GpioMiso, GpioSck> bus;

  typedef Gpio<GPIOC_BASE, 15> CS_FLASH0;

 public:
  typedef ProtocolSPI<bus, CS_FLASH0> MemoryBus0;

  virtual ~MultiFlashController();

  void addData(const uint8_t* buffer, size_t size);

 protected:

  void run() override;

 private:
  MultiFlashController();

  static std::string logtag()
  {
    return "MultiFlashCTRL";
  }

  uint32_t block_counter = 0;
  miosix::Queue<FlashDataBlock,20> block_queue_;

  FlashController<MemoryBus0>* flash0_;
  //FlashController<spi_flash0> flash1; //Change to spi_flash1
  //FlashController<spi_flash0> flash2; //Change to spi_flash2
};

}  //namespace flashmemory

#endif /* MULTIFLASHCONTROLLER_H */
