/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

// Remember to modify pins of leds
namespace miosix
{

namespace interfaces
{

// Debug - USART4
namespace debug
{
using tx = Gpio<GPIOA_BASE, 0>;
using rx = Gpio<GPIOA_BASE, 1>;
}  // namespace debug

// Cam 1 - USART2
namespace cam1
{
using tx = Gpio<GPIOA_BASE, 2>;
using rx = Gpio<GPIOA_BASE, 3>;
}  // namespace cam1

// Cam 2 - UART7
namespace cam2
{
using tx = Gpio<GPIOF_BASE, 7>;
}  // namespace cam2

// Cam 3 - UART7
namespace cam3
{
using tx = Gpio<GPIOE_BASE, 8>;
}  // namespace cam3

using camMosfet = Gpio<GPIOE_BASE, 7>;

namespace can1
{
using rx = Gpio<GPIOA_BASE, 11>;
using tx = Gpio<GPIOA_BASE, 12>;
}  // namespace can1

}  // namespace interfaces

namespace leds
{
using led1 = Gpio<GPIOC_BASE, 15>;
using led2 = Gpio<GPIOB_BASE, 4>;
using led3 = Gpio<GPIOB_BASE, 5>;
using led4 = Gpio<GPIOB_BASE, 6>;
using led5 = Gpio<GPIOB_BASE, 7>;
}  // namespace leds

}  // namespace miosix

#endif  // HWMAPPING_H
