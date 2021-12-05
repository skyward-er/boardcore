/* Copyright (c) 2017 Skyward Experimental Rocketry
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

#include <ActiveObject.h>
#include <Common.h>
#include <drivers/BusTemplate.h>

#include "FlashController.h"

using miosix::Gpio;

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

class MultiFlashController : public Singleton<MultiFlashController>,
                             ActiveObject
{
    friend class Singleton<MultiFlashController>;

    template <typename>
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

    static std::string logtag() { return "MultiFlashCTRL"; }

    uint32_t block_counter = 0;
    miosix::Queue<FlashDataBlock, 20> block_queue_;

    FlashController<MemoryBus0>* flash0_;
    // FlashController<spi_flash0> flash1; //Change to spi_flash1
    // FlashController<spi_flash0> flash2; //Change to spi_flash2
};

}  // namespace flashmemory
