/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "interfaces/gpio.h"

#define MIOSIX_RADIO_DIO0_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_RADIO_DIO1_IRQ EXTI12_IRQHandlerImpl
#define MIOSIX_RADIO_DIO3_IRQ EXTI13_IRQHandlerImpl
#define MIOSIX_RADIO_SPI SPI1

namespace miosix
{

namespace interfaces
{
// Miosix radio SPI
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1
}  // namespace interfaces

namespace btns
{
using ignition = Gpio<GPIOB_BASE, 4>;
using filling  = Gpio<GPIOE_BASE, 6>;
using venting  = Gpio<GPIOE_BASE, 4>;
using release  = Gpio<GPIOG_BASE, 9>;
using detach   = Gpio<GPIOD_BASE, 7>;
using tars     = Gpio<GPIOD_BASE, 5>;
using arm      = Gpio<GPIOE_BASE, 2>;
}  // namespace btns

namespace radio
{
namespace spi
{
using namespace miosix::interfaces::spi1;
}

using cs   = Gpio<GPIOF_BASE, 6>;
using dio0 = Gpio<GPIOB_BASE, 1>;
using dio1 = Gpio<GPIOD_BASE, 12>;
using dio3 = Gpio<GPIOD_BASE, 13>;
using txEn = Gpio<GPIOG_BASE, 2>;
using rxEn = Gpio<GPIOG_BASE, 3>;
using nrst = Gpio<GPIOB_BASE, 0>;
}  // namespace radio

namespace ui
{
using buzzer   = Gpio<GPIOB_BASE, 7>;
using armedLed = Gpio<GPIOC_BASE, 13>;
using redLed   = Gpio<GPIOG_BASE, 14>;
}  // namespace ui

}  // namespace miosix
