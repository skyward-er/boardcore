/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#define MIOSIX_ETHERNET_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_ETHERNET_SPI SPI4

namespace miosix
{
namespace interfaces
{
// RADIO 1
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// FREE SPI
namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

// ETHERNET
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

namespace can2
{
using rx = Gpio<GPIOB_BASE, 12>;
using tx = Gpio<GPIOB_BASE, 13>;
}  // namespace can2

// DBG
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// FREE USART
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

namespace timers
{
using tim3ch2 = Gpio<GPIOC_BASE, 7>;   // step 1
using tim1ch4 = Gpio<GPIOA_BASE, 11>;  // count 1
using tim4ch1 = Gpio<GPIOD_BASE, 12>;  // step 2
using tim8ch4 = Gpio<GPIOC_BASE, 9>;   // count 2
}  // namespace timers

}  // namespace interfaces

namespace radio
{
using sck       = interfaces::spi1::sck;
using miso      = interfaces::spi1::miso;
using mosi      = interfaces::spi1::mosi;
using cs        = Gpio<GPIOA_BASE, 4>;
using dio0      = Gpio<GPIOC_BASE, 6>;
using dio1      = Gpio<GPIOD_BASE, 4>;
using dio3      = Gpio<GPIOD_BASE, 5>;
using rx_enable = Gpio<GPIOB_BASE, 9>;
using tx_enable = Gpio<GPIOB_BASE, 8>;
}  // namespace radio

namespace stepper1
{
using enable     = Gpio<GPIOA_BASE, 8>;
using direction  = Gpio<GPIOA_BASE, 12>;
using pulseTimer = interfaces::timers::tim3ch2;
using countTimer = interfaces::timers::tim1ch4;
}  // namespace stepper1

namespace stepper2
{
using enable     = Gpio<GPIOB_BASE, 14>;
using direction  = Gpio<GPIOG_BASE, 7>;
using pulseTimer = interfaces::timers::tim4ch1;
using countTimer = interfaces::timers::tim8ch4;
}  // namespace stepper2

namespace ethernet
{
namespace spi
{
using sck  = miosix::interfaces::spi4::sck;
using miso = miosix::interfaces::spi4::miso;
using mosi = miosix::interfaces::spi4::mosi;
}  // namespace spi

using cs   = Gpio<GPIOE_BASE, 4>;
using intr = Gpio<GPIOC_BASE, 1>;
using nrst = Gpio<GPIOB_BASE, 1>;
}  // namespace ethernet

}  // namespace miosix