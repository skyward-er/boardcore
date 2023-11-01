/* Copyright (c) 2022 Skyward Experimental Rocketry
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

// CC3135
namespace spi1
{
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOA_BASE, 7>;
}  // namespace spi1

// RA-01
namespace spi4
{
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
}  // namespace spi4

}  // namespace interfaces

namespace peripherals
{

namespace ra01
{

namespace pc13
{
using cs   = Gpio<GPIOC_BASE, 13>;
using dio0 = Gpio<GPIOF_BASE, 6>;
using dio1 = Gpio<GPIOA_BASE, 4>;
using dio3 = Gpio<GPIOC_BASE, 11>;
using nrst = Gpio<GPIOC_BASE, 14>;
}  // namespace pc13

namespace pe4
{
using cs   = Gpio<GPIOE_BASE, 4>;
using dio0 = Gpio<GPIOE_BASE, 3>;
using nrst = Gpio<GPIOG_BASE, 2>;
}  // namespace pe4

}  // namespace ra01

namespace cc3135
{
using cs   = Gpio<GPIOD_BASE, 4>;
using hib  = Gpio<GPIOG_BASE, 3>;
using intr = Gpio<GPIOD_BASE, 5>;
using nrst = Gpio<GPIOB_BASE, 7>;
}  // namespace cc3135

}  // namespace peripherals

}  // namespace miosix

#endif  // HWMAPPING_H
