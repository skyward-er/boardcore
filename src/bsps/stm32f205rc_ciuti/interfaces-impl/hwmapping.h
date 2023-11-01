/* Copyright (c) 2021 Skyward Experimental Rocketry
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

namespace miosix
{

namespace interfaces
{

namespace spi1
{
using cs   = Gpio<GPIOA_BASE, 4>;
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOA_BASE, 6>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// LIS331HH and AD8403
namespace spi2
{
using sck  = Gpio<GPIOB_BASE, 13>;
using miso = Gpio<GPIOB_BASE, 14>;
using mosi = Gpio<GPIOB_BASE, 15>;
}  // namespace spi2

// Debug
namespace usart1
{
using tx = Gpio<GPIOA_BASE, 9>;
using rx = Gpio<GPIOA_BASE, 10>;
}  // namespace usart1

// Pogo pin
namespace usart2
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace usart2

// Pogo pin
namespace usart3
{
using tx = Gpio<GPIOB_BASE, 10>;
using rx = Gpio<GPIOB_BASE, 11>;
}  // namespace usart3

namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

}  // namespace interfaces

namespace devices
{

namespace lis331hh
{
using cs = Gpio<GPIOC_BASE, 4>;
}

namespace ad5204
{
using cs = Gpio<GPIOC_BASE, 0>;
}

namespace ina188
{
using vsense1 = Gpio<GPIOA_BASE, 0>;
using vsense2 = Gpio<GPIOA_BASE, 1>;
using mosfet1 = Gpio<GPIOC_BASE, 3>;
using mosfet2 = Gpio<GPIOC_BASE, 2>;
}  // namespace ina188

using vbat = Gpio<GPIOC_BASE, 5>;

namespace buttons
{
using bypass = Gpio<GPIOC_BASE, 7>;
using record = Gpio<GPIOA_BASE, 8>;
}  // namespace buttons

namespace buzzer
{
using drive = Gpio<GPIOC_BASE, 6>;  // PWM TIM8_CH1
}

namespace leds
{
using led1 = Gpio<GPIOB_BASE, 0>;  // TIM3_CH3
using led2 = Gpio<GPIOB_BASE, 5>;
using led3 = Gpio<GPIOB_BASE, 6>;
using led4 = Gpio<GPIOB_BASE, 7>;
}  // namespace leds

}  // namespace devices

}  // namespace miosix

#endif  // HWMAPPING_H
