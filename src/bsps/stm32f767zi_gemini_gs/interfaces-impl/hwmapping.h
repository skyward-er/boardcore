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

#ifndef HWMAPPING_H
#define HWMAPPING_H

#include "interfaces/gpio.h"

#define MIOSIX_RADIO1_DIO0_IRQ EXTI8_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO1_IRQ EXTI10_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO2_IRQ EXTI11_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO3_IRQ EXTI12_IRQHandlerImpl
#define MIOSIX_RADIO1_SPI SPI3

#define MIOSIX_RADIO2_DIO0_IRQ EXTI6_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO1_IRQ EXTI4_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO2_IRQ EXTI7_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO3_IRQ EXTI5_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO4_IRQ EXTI2_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO5_IRQ EXTI3_IRQHandlerImpl
#define MIOSIX_RADIO2_SPI SPI1

#define MIOSIX_ETHERNET_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_ETHERNET_SPI SPI4

// Remember to modify pins of leds
namespace miosix
{

namespace interfaces
{

// Radio 1
namespace spi3
{
using sck  = Gpio<GPIOB_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOD_BASE, 6>;
}  // namespace spi3

// Radio 2
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// Ethernet module
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

}  // namespace interfaces

namespace radio1
{
namespace spi
{
using sck  = miosix::interfaces::spi3::sck;
using miso = miosix::interfaces::spi3::miso;
using mosi = miosix::interfaces::spi3::mosi;
}  // namespace spi

using cs   = Gpio<GPIOA_BASE, 15>;
using dio0 = Gpio<GPIOC_BASE, 8>;
using dio1 = Gpio<GPIOC_BASE, 10>;
using dio3 = Gpio<GPIOC_BASE, 12>;
using txen = Gpio<GPIOG_BASE, 12>;
using rxen = Gpio<GPIOG_BASE, 14>;
using nrst = Gpio<GPIOA_BASE, 1>;
}  // namespace radio1

namespace radio2
{
namespace spi
{
using sck  = miosix::interfaces::spi1::sck;
using miso = miosix::interfaces::spi1::miso;
using mosi = miosix::interfaces::spi1::mosi;
}  // namespace spi

using cs   = Gpio<GPIOA_BASE, 4>;
using dio0 = Gpio<GPIOC_BASE, 6>;
using dio1 = Gpio<GPIOD_BASE, 4>;
using dio3 = Gpio<GPIOD_BASE, 5>;
using txen = Gpio<GPIOB_BASE, 8>;
using rxen = Gpio<GPIOB_BASE, 9>;
using nrst = Gpio<GPIOA_BASE, 0>;
}  // namespace radio2

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

#endif  // HWMAPPING_H
