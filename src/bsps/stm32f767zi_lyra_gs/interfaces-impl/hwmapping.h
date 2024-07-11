/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#define MIOSIX_RADIO1_DIO0_IRQ EXTI6_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO1_IRQ EXTI4_IRQHandlerImpl
#define MIOSIX_RADIO1_DIO3_IRQ EXTI5_IRQHandlerImpl
#define MIOSIX_RADIO1_SPI SPI1

#define MIOSIX_RADIO2_DIO0_IRQ EXTI9_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO1_IRQ EXTI10_IRQHandlerImpl
#define MIOSIX_RADIO2_DIO3_IRQ EXTI11_IRQHandlerImpl
#define MIOSIX_RADIO2_SPI SPI2

#define MIOSIX_ETHERNET_IRQ EXTI1_IRQHandlerImpl
#define MIOSIX_ETHERNET_SPI SPI4

// Remember to modify pins of leds
namespace miosix
{

// Compute units leds
using userLed1 = Gpio<GPIOC_BASE, 14>;
using userLed2 = Gpio<GPIOC_BASE, 13>;
using userLed3 = Gpio<GPIOC_BASE, 2>;
using userLed4 = Gpio<GPIOC_BASE, 15>;

namespace interfaces
{

// Radio 1
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// Radio 2
namespace spi2
{
using sck  = Gpio<GPIOD_BASE, 3>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOC_BASE, 3>;
}  // namespace spi2

// Ethernet module
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

// USART VN300
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

// UART
namespace uart4
{
using tx = Gpio<GPIOA_BASE, 1>;
using rx = Gpio<GPIOA_BASE, 0>;
}  // namespace uart4

namespace timers
{
using tim1ch1  = Gpio<GPIOA_BASE, 8>;   //< step StepperHorizontal
using tim3ch2  = Gpio<GPIOC_BASE, 7>;   //< count StepperHorizontal
using tim4ch1  = Gpio<GPIOD_BASE, 12>;  //< step StepperVertical
using tim8ch1  = Gpio<GPIOC_BASE, 6>;   //< count StepperVertical
using tim10ch1 = Gpio<GPIOB_BASE, 8>;   //< yellow LED CommandBox
using tim2ch4  = Gpio<GPIOB_BASE, 11>;  //< red LED CommandBox
using tim2ch1  = Gpio<GPIOA_BASE, 15>;  //< blue LED CommandBox
}  // namespace timers

}  // namespace interfaces

// Command box control switches (ARP)
namespace commBox
{
using switchArm    = Gpio<GPIOB_BASE, 7>;
using switchActive = Gpio<GPIOE_BASE, 3>;
using ledTimY1     = interfaces::timers::tim2ch1;   //< yellow LED
using ledTimR2     = interfaces::timers::tim2ch4;   //< red LED
using ledTimB3     = interfaces::timers::tim10ch1;  //< blue LED
}  // namespace commBox

namespace sensors
{
namespace VN300
{
using tx = interfaces::usart2::tx;
using rx = interfaces::usart2::rx;
}  // namespace VN300
}  // namespace sensors

// Stepper Horizontal
namespace stepper1
{
using enable     = Gpio<GPIOD_BASE, 13>;
using direction  = Gpio<GPIOB_BASE, 13>;
using pulseTimer = interfaces::timers::tim1ch1;
using countTimer = interfaces::timers::tim3ch2;
}  // namespace stepper1

// Stepper Vertical
namespace stepper2
{
using enable     = Gpio<GPIOD_BASE, 7>;
using direction  = Gpio<GPIOB_BASE, 4>;
using pulseTimer = interfaces::timers::tim4ch1;
using countTimer = interfaces::timers::tim8ch1;
}  // namespace stepper2

namespace radio1
{
namespace spi
{
using sck  = miosix::interfaces::spi1::sck;
using miso = miosix::interfaces::spi1::miso;
using mosi = miosix::interfaces::spi1::mosi;
}  // namespace spi

using cs   = Gpio<GPIOA_BASE, 4>;
using dio0 = Gpio<GPIOG_BASE, 6>;
using dio1 = Gpio<GPIOD_BASE, 4>;
using dio3 = Gpio<GPIOD_BASE, 5>;
using txen = Gpio<GPIOG_BASE, 3>;
using rxen = Gpio<GPIOG_BASE, 7>;
using nrst = Gpio<GPIOG_BASE, 12>;
}  // namespace radio1

namespace radio = radio1;

namespace radio2
{
namespace spi
{
using sck  = miosix::interfaces::spi2::sck;
using miso = miosix::interfaces::spi2::miso;
using mosi = miosix::interfaces::spi2::mosi;
}  // namespace spi

using cs   = Gpio<GPIOB_BASE, 12>;
using dio0 = Gpio<GPIOG_BASE, 9>;
using dio1 = Gpio<GPIOG_BASE, 10>;
using dio3 = Gpio<GPIOG_BASE, 11>;
using txen = Gpio<GPIOG_BASE, 13>;
using rxen = Gpio<GPIOG_BASE, 14>;
using nrst = Gpio<GPIOB_BASE, 2>;
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

namespace dipSwitch
{
using sh  = Gpio<GPIOC_BASE, 4>;
using clk = Gpio<GPIOC_BASE, 5>;
using qh  = Gpio<GPIOB_BASE, 15>;
}  // namespace dipSwitch

}  // namespace miosix
