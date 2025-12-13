/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Davide Mor, Ettore Pane
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

#define MIOSIX_BUZZER_TIM TIM3
#define MIOSIX_BUZZER_CHANNEL CHANNEL_3

#define MIOSIX_RADIO_DIO0_IRQ EXTI5_IRQHandlerImpl
#define MIOSIX_RADIO_DIO1_IRQ EXTI4_IRQHandlerImpl
#define MIOSIX_RADIO_DIO3_IRQ EXTI10_IRQHandlerImpl

#define MIOSIX_ETHERNET_IRQ EXTI1_IRQHandlerImpl

namespace miosix
{

namespace interfaces
{
// Ethernet
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// Radio
namespace spi6
{
using sck  = Gpio<GPIOG_BASE, 13>;
using miso = Gpio<GPIOG_BASE, 12>;
using mosi = Gpio<GPIOG_BASE, 14>;
}  // namespace spi6

// Miosix UART
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// Serial
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 3>;
using rx = Gpio<GPIOA_BASE, 2>;
}  // namespace usart2

// Serial
namespace uart4
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace uart4

namespace timers
{
using tim3ch3 = Gpio<GPIOB_BASE, 0>;  // Buzzer
}  // namespace timers

}  // namespace interfaces

namespace ethernet
{
using cs   = Gpio<GPIOA_BASE, 4>;
using intr = Gpio<GPIOC_BASE, 1>;
using nrst = Gpio<GPIOC_BASE, 3>;
}  // namespace ethernet

namespace radio
{
using cs   = Gpio<GPIOB_BASE, 7>;
using dio0 = Gpio<GPIOC_BASE, 5>;
using dio1 = Gpio<GPIOD_BASE, 4>;
using dio3 = Gpio<GPIOG_BASE, 10>;
using txEn = Gpio<GPIOA_BASE, 12>;
using rxEn = Gpio<GPIOA_BASE, 11>;
}  // namespace radio

namespace btns
{
using ox_filling  = Gpio<GPIOD_BASE, 6>;
using ox_release  = Gpio<GPIOD_BASE, 5>;
using prz_filling = Gpio<GPIOA_BASE, 15>;
using prz_release = Gpio<GPIOG_BASE, 7>;

using prz_ox     = Gpio<GPIOG_BASE, 6>;
using prz_fuel   = Gpio<GPIOE_BASE, 3>;
using ox_venting = Gpio<GPIOD_BASE, 11>;
using detach     = Gpio<GPIOG_BASE, 3>;

using spare_0 = Gpio<GPIOB_BASE, 12>;
using spare_1 = Gpio<GPIOB_BASE, 15>;
using spare_2 = Gpio<GPIOG_BASE, 9>;
using spare_3 = Gpio<GPIOB_BASE, 13>;
using spare_4 = Gpio<GPIOB_BASE, 1>;
using spare_5 = Gpio<GPIOB_BASE, 11>;

using prz_3way = Gpio<GPIOD_BASE, 3>;
using tars1    = Gpio<GPIOE_BASE, 6>;
using tars3    = Gpio<GPIOB_BASE, 4>;

using ignition = Gpio<GPIOB_BASE, 14>;  // Port: IGN1
using arm      = Gpio<GPIOD_BASE, 13>;  // Port: ARM
}  // namespace btns

namespace ui
{
using armedLed = Gpio<GPIOD_BASE, 12>;  // Port: IGN1
using buzzer   = interfaces::timers::tim3ch3;
}  // namespace ui

}  // namespace miosix
